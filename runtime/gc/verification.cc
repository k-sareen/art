/*
 * Copyright (C) 2017 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "verification-inl.h"

#include <iomanip>
#include <sstream>

#include "art_field-inl.h"
#include "base/file_utils.h"
#include "base/logging.h"
#include "mirror/class-inl.h"
#include "mirror/object-refvisitor-inl.h"
#include "mmtk-art/mmtk_is_marked_visitor.h"

namespace art HIDDEN {
namespace gc {

std::string Verification::DumpRAMAroundAddress(uintptr_t addr, uintptr_t bytes) const {
  uintptr_t* dump_start = reinterpret_cast<uintptr_t*>(addr - bytes);
  uintptr_t* dump_end = reinterpret_cast<uintptr_t*>(addr + bytes);
  std::ostringstream oss;
  oss << " adjacent_ram=";

  {
    // Check if the RAM is accessible.
    android::base::unique_fd read_fd, write_fd;
    if (!android::base::Pipe(&read_fd, &write_fd)) {
      LOG(WARNING) << "Could not create pipe, RAM being dumped may be unaccessible";
    } else {
      size_t count = 2 * bytes;
      if (write(write_fd.get(), dump_start, count) != static_cast<ssize_t>(count)) {
        oss << "unaccessible";
        dump_start = dump_end;
      }
    }
  }

  for (const uintptr_t* p = dump_start; p < dump_end; ++p) {
    if (p == reinterpret_cast<uintptr_t*>(addr)) {
      // Marker of where the address is.
      oss << "| ";
    }
    oss << std::hex << std::setfill('0') << std::setw(sizeof(uintptr_t) * 2) << *p << " ";
  }
  return oss.str();
}

std::string Verification::DumpObjectInfo(const void* addr, const char* tag) const {
  std::ostringstream oss;
  oss << tag << "=" << addr;
  if (IsValidHeapObjectAddress(addr)) {
    mirror::Object* obj = reinterpret_cast<mirror::Object*>(const_cast<void*>(addr));
    mirror::Class* klass = obj->GetClass<kVerifyNone, kWithoutReadBarrier>();
    oss << " klass=" << klass;
    if (IsValidClass(klass)) {
      oss << "(" << klass->PrettyClass() << ")";
      if (klass->IsArrayClass<kVerifyNone>()) {
        oss << " length=" << obj->AsArray<kVerifyNone>()->GetLength();
      }
    } else {
      oss << " <invalid address>";
    }
    space::Space* const space = heap_->FindSpaceFromAddress(addr);
    if (space != nullptr) {
      oss << " space=" << *space;
    }
#if !ART_USE_MMTK
    if (gUseWriteBarrier) {
      accounting::CardTable* card_table = heap_->GetCardTable();
      if (card_table->AddrIsInCardTable(addr)) {
        oss << " card=" << static_cast<size_t>(
            card_table->GetCard(reinterpret_cast<const mirror::Object*>(addr)));
      }
    }
#endif  // !ART_USE_MMTK
    // Dump adjacent RAM.
    oss << DumpRAMAroundAddress(reinterpret_cast<uintptr_t>(addr), 4 * kObjectAlignment);
  } else {
    oss << " <invalid address>";
  }
  return oss.str();
}

void Verification::LogHeapCorruption(ObjPtr<mirror::Object> holder,
                                     MemberOffset offset,
                                     mirror::Object* ref,
                                     bool fatal) const {
  // Highest priority logging first.
  // Buffer the output in the string stream since it is more important than the stack traces
  // and we want it to have log priority. The stack traces are printed from Runtime::Abort
  // which is called from LOG(FATAL) but before the abort message.
  std::ostringstream oss;
  oss << "GC tried to mark invalid reference " << ref << std::endl;
  oss << DumpObjectInfo(ref, "ref") << "\n";
  oss << DumpObjectInfo(holder.Ptr(), "holder") << "\n";
  if (holder != nullptr) {
    mirror::Class* holder_klass = holder->GetClass<kVerifyNone, kWithoutReadBarrier>();
    if (IsValidClass(holder_klass)) {
      oss << " field_offset=" << offset.Uint32Value();
      ArtField* field = holder->FindFieldByOffset(offset);
      if (field != nullptr) {
        oss << " name=" << field->GetName();
      }
    }
    mirror::HeapReference<mirror::Object>* addr = holder->GetFieldObjectReferenceAddr(offset);
    oss << " reference addr"
        << DumpRAMAroundAddress(reinterpret_cast<uintptr_t>(addr), 4 * kObjectAlignment);
  }
  Runtime::Current()->GetHeap()->DumpSpaces(oss);
  MemMap::DumpMaps(oss, /* terse= */ true);

  if (fatal) {
    LOG(FATAL) << oss.str();
  } else {
    LOG(FATAL_WITHOUT_ABORT) << oss.str();
  }
}

bool Verification::IsAddressInHeapSpace(const void* addr, space::Space** out_space) const {
#if !ART_USE_MMTK
  space::Space* const space = heap_->FindSpaceFromAddress(addr);
  if (space != nullptr) {
    if (out_space != nullptr) {
      *out_space = space;
    }
    return true;
  }
  return false;
#else
  UNUSED(out_space);
  // Check in case the object address is in the boot image
  space::Space* const space = heap_->FindSpaceFromAddress(addr);
  return (space != nullptr) || heap_->GetThirdPartyHeap()->IsObjectInHeapSpace(addr);
#endif  // !ART_USE_MMTK
}

bool Verification::IsValidHeapObjectAddress(const void* addr, space::Space** out_space) const {
  return IsAligned<kObjectAlignment>(addr) && IsAddressInHeapSpace(addr, out_space);
}

// Use for visiting the GcRoots held live by ArtFields, ArtMethods, and ClassLoaders.
class Verification::BFSFindReachable {
 public:
  explicit BFSFindReachable(ObjectSet* visited) : visited_(visited) {}

  void operator()(mirror::Object* obj, MemberOffset offset, [[maybe_unused]] bool is_static) const
      REQUIRES_SHARED(Locks::mutator_lock_) {
    ArtField* field = obj->FindFieldByOffset(offset);
    Visit(obj->GetFieldObject<mirror::Object>(offset),
          field != nullptr ? field->GetName() : "");
  }

  void VisitRootIfNonNull(mirror::CompressedReference<mirror::Object>* root) const
      REQUIRES_SHARED(Locks::mutator_lock_) {
    if (!root->IsNull()) {
      VisitRoot(root);
    }
  }

  void VisitRoot(mirror::CompressedReference<mirror::Object>* root) const
      REQUIRES_SHARED(Locks::mutator_lock_) {
    Visit(root->AsMirrorPtr(), "!nativeRoot");
  }

  void Visit(mirror::Object* ref, const std::string& field_name) const
      REQUIRES_SHARED(Locks::mutator_lock_) {
    if (ref != nullptr && visited_->insert(ref).second) {
      new_visited_.emplace_back(ref, field_name);
    }
  }

  const WorkQueue& NewlyVisited() const {
    return new_visited_;
  }

 private:
  ObjectSet* visited_;
  mutable WorkQueue new_visited_;
};

class Verification::CollectRootVisitor : public SingleRootVisitor {
 public:
  CollectRootVisitor(ObjectSet* visited, WorkQueue* work) : visited_(visited), work_(work) {}

  void VisitRoot(mirror::Object* obj, const RootInfo& info)
      override REQUIRES_SHARED(Locks::mutator_lock_) {
    if (obj != nullptr && visited_->insert(obj).second) {
      std::ostringstream oss;
      oss << info.ToString() << " = " << obj << "(" << mirror::Object::PrettyTypeOf(obj) << ")";
      work_->emplace_back(obj, oss.str());
    }
  }

 private:
  ObjectSet* const visited_;
  WorkQueue* const work_;
};

class CollectRootVectorVisitor : public SingleRootVisitor {
 public:
  CollectRootVectorVisitor(ObjectSet* visited,
                           std::deque<
                              std::tuple<
                                mirror::Object*,
                                std::string,
                                std::vector<mirror::Object*>
                              >
                            >* work) : visited_(visited), work_(work) {}

  void VisitRoot(mirror::Object* obj, const RootInfo& info)
      override REQUIRES_SHARED(Locks::mutator_lock_) {
    if (obj != nullptr && visited_->insert(obj).second) {
      std::vector v = {obj};
      std::ostringstream oss;
      oss << info.ToString() << " = " << obj << "(" << mirror::Object::PrettyTypeOf(obj) << ")";
      work_->emplace_back(obj, oss.str(), v);
    }
  }

 private:
  ObjectSet* const visited_;
  std::deque<std::tuple<mirror::Object*, std::string, std::vector<mirror::Object*>>>* const work_;
};

std::string Verification::FirstPathFromRootSet(ObjPtr<mirror::Object> target) const {
  Runtime* const runtime =  Runtime::Current();
  std::set<mirror::Object*> visited;
  std::deque<std::pair<mirror::Object*, std::string>> work;
  {
    CollectRootVisitor root_visitor(&visited, &work);
    runtime->VisitRoots(&root_visitor, kVisitRootFlagAllRoots);
  }
  while (!work.empty()) {
    auto pair = work.front();
    work.pop_front();
    if (pair.first == target) {
      return pair.second;
    }
    BFSFindReachable visitor(&visited);
    pair.first->VisitReferences(visitor, VoidFunctor());
    for (auto&& pair2 : visitor.NewlyVisited()) {
      std::ostringstream oss;
      mirror::Object* obj = pair2.first;
      oss << pair.second << " -> " << obj << "(" << mirror::Object::PrettyTypeOf(obj) << ")." << pair2.second;
      work.emplace_back(obj, oss.str());
    }
  }
  return "<no path found>";
}

std::pair<std::vector<mirror::Object*>, std::string> Verification::FirstPathFromRootSetVector(ObjPtr<mirror::Object> target) const {
  Runtime* const runtime =  Runtime::Current();
  std::set<mirror::Object*> visited;
  std::deque<std::tuple<mirror::Object*, std::string, std::vector<mirror::Object*>>> work;
  std::vector<mirror::Object*> empty_vec;
  {
    CollectRootVectorVisitor root_visitor(&visited, &work);
    runtime->VisitRoots(&root_visitor, kVisitRootFlagAllRoots);
  }
  while (!work.empty()) {
    auto [current_object, path, vec] = work.front();
    work.pop_front();
    if (current_object == target) {
      return std::make_pair(vec, path);
    }
    BFSFindReachable visitor(&visited);
    current_object->VisitReferences(visitor, VoidFunctor());
    vec.emplace_back(current_object);
    for (auto&& pair2 : visitor.NewlyVisited()) {
      std::ostringstream oss;
      mirror::Object* obj = pair2.first;
      oss << path << " -> " << obj << "(" << mirror::Object::PrettyTypeOf(obj) << ")." << pair2.second;
      work.emplace_back(obj, oss.str(), vec);
    }
  }
  return std::make_pair(empty_vec, "<no path found>");
}

void Verification::SanityPreGC() const {
  if (live_ != nullptr) {
    delete live_;
  }
  if (queue_ != nullptr) {
    delete queue_;
  }

  Runtime* runtime = Runtime::Current();
  live_ = new std::set<mirror::Object*>();
  queue_ = new std::deque<std::pair<mirror::Object*, std::string>>();

  {
    CollectRootVisitor root_visitor(live_, queue_);
    runtime->VisitRoots(&root_visitor, kVisitRootFlagAllRoots);
  }

  while (!queue_->empty()) {
    auto pair = queue_->front();
    queue_->pop_front();
    BFSFindReachable visitor(live_);
    pair.first->VisitReferences(visitor, VoidFunctor());
    for (auto&& pair2 : visitor.NewlyVisited()) {
      std::ostringstream oss;
      mirror::Object* obj = pair2.first;
      oss << pair.second << " -> " << obj << "(" << mirror::Object::PrettyTypeOf(obj) << ")." << pair2.second;
      queue_->emplace_back(obj, oss.str());
    }
  }
}

void Verification::SanityPostGC() const {
  bool failed = false;
  IsMarkedVisitor* is_marked_visitor = new third_party_heap::MmtkIsMarkedVisitor();
  for (auto obj : *live_) {
    if (is_marked_visitor->IsMarked(obj) == nullptr) {
      failed = true;
      auto [vec, path] = FirstPathFromRootSetVector(obj);
      LOG(FATAL_WITHOUT_ABORT) << "SanityPostGC: Found live object "
                 << obj
                 << " that is not marked by MMTk!"
                 << "\n"
                 << FirstPathFromRootSet(obj)
                 << "\n"
                 << "Dumping memory around missing object "
                 << obj
                 << DumpRAMAroundAddress((uintptr_t)obj, 128);
      for (auto path_obj : vec) {
        LOG(FATAL_WITHOUT_ABORT) << "Object "
                                 << path_obj
                                 << " marked "
                                 << (is_marked_visitor->IsMarked(path_obj) != nullptr)
                                 << "\nDumping memory around "
                                 << path_obj
                                 << "\n"
                                 << DumpRAMAroundAddress((uintptr_t)path_obj, 128);
      }

      if (failed) {
        LOG(FATAL) << "SanityPostGC: Aborting early.";
      }
    }
  }

  if (failed) {
    LOG(FATAL) << "SanityPostGC: Aborting";
  }
}

}  // namespace gc
}  // namespace art
