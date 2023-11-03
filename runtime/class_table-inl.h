/*
 * Copyright (C) 2015 The Android Open Source Project
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

#ifndef ART_RUNTIME_CLASS_TABLE_INL_H_
#define ART_RUNTIME_CLASS_TABLE_INL_H_

#include "class_table.h"

#include "base/mutex-inl.h"
#include "dex/utf.h"
#include "gc_root-inl.h"
#include "mirror/class.h"
#include "oat_file.h"
#include "obj_ptr-inl.h"

namespace art {

inline ClassTable::TableSlot::TableSlot(ObjPtr<mirror::Class> klass)
    : TableSlot(klass, klass->DescriptorHash()) {}

inline uint32_t ClassTable::ClassDescriptorHash::operator()(const TableSlot& slot) const {
  // No read barriers needed, we're reading a chain of constant references for comparison with null
  // and retrieval of constant primitive data. See `ReadBarrierOption` and `Class::DescriptorHash()`.
  return slot.Read<kWithoutReadBarrier>()->DescriptorHash();
}

inline uint32_t ClassTable::ClassDescriptorHash::operator()(const DescriptorHashPair& pair) const {
  DCHECK_EQ(ComputeModifiedUtf8Hash(pair.first), pair.second);
  return pair.second;
}

inline bool ClassTable::ClassDescriptorEquals::operator()(const TableSlot& a,
                                                          const TableSlot& b) const {
  // No read barrier needed, we're reading a chain of constant references for comparison
  // with null and retrieval of constant primitive data. See ReadBarrierOption.
  if (a.Hash() != b.Hash()) {
    std::string temp;
    DCHECK(!a.Read<kWithoutReadBarrier>()->DescriptorEquals(
        b.Read<kWithoutReadBarrier>()->GetDescriptor(&temp)));
    return false;
  }
  std::string temp;
  return a.Read<kWithoutReadBarrier>()->DescriptorEquals(
      b.Read<kWithoutReadBarrier>()->GetDescriptor(&temp));
}

inline bool ClassTable::ClassDescriptorEquals::operator()(const TableSlot& a,
                                                          const DescriptorHashPair& b) const {
  // No read barrier needed, we're reading a chain of constant references for comparison
  // with null and retrieval of constant primitive data. See ReadBarrierOption.
  if (a.Hash() != ((uint8_t) b.second)) {
    DCHECK(!a.Read<kWithoutReadBarrier>()->DescriptorEquals(b.first));
    return false;
  }
  return a.Read<kWithoutReadBarrier>()->DescriptorEquals(b.first);
}

template<class Visitor>
void ClassTable::VisitRoots(Visitor& visitor) {
  ReaderMutexLock mu(Thread::Current(), lock_);
  for (ClassSet& class_set : classes_) {
    for (TableSlot& table_slot : class_set) {
      table_slot.VisitRoot(visitor);
    }
  }
  for (GcRoot<mirror::Object>& root : strong_roots_) {
    visitor.VisitRoot(root.AddressWithoutBarrier());
  }
  for (const OatFile* oat_file : oat_files_) {
    for (GcRoot<mirror::Object>& root : oat_file->GetBssGcRoots()) {
      visitor.VisitRootIfNonNull(root.AddressWithoutBarrier());
    }
  }
}

template<class Visitor>
void ClassTable::VisitRoots(const Visitor& visitor) {
  ReaderMutexLock mu(Thread::Current(), lock_);
  for (ClassSet& class_set : classes_) {
    for (TableSlot& table_slot : class_set) {
      table_slot.VisitRoot(visitor);
    }
  }
  for (GcRoot<mirror::Object>& root : strong_roots_) {
    visitor.VisitRoot(root.AddressWithoutBarrier());
  }
  for (const OatFile* oat_file : oat_files_) {
    for (GcRoot<mirror::Object>& root : oat_file->GetBssGcRoots()) {
      visitor.VisitRootIfNonNull(root.AddressWithoutBarrier());
    }
  }
}

template <typename Visitor>
class ClassTable::TableSlot::ClassAndRootVisitor {
 public:
  explicit ClassAndRootVisitor(Visitor& visitor) : visitor_(visitor) {}

  void VisitRoot(mirror::CompressedReference<mirror::Object>* klass) const
      REQUIRES_SHARED(Locks::mutator_lock_) {
    DCHECK(!klass->IsNull());
    // Visit roots in the klass object
    visitor_(klass->AsMirrorPtr());
    // Visit the GC-root holding klass' reference
    visitor_.VisitRoot(klass);
  }

 private:
  Visitor& visitor_;
};

template <typename Visitor>
void ClassTable::VisitClassesAndRoots(Visitor& visitor) {
  TableSlot::ClassAndRootVisitor class_visitor(visitor);
  ReaderMutexLock mu(Thread::Current(), lock_);
  for (ClassSet& class_set : classes_) {
    for (TableSlot& table_slot : class_set) {
      table_slot.VisitRoot(class_visitor);
    }
  }
  for (GcRoot<mirror::Object>& root : strong_roots_) {
    visitor.VisitRoot(root.AddressWithoutBarrier());
  }
  for (const OatFile* oat_file : oat_files_) {
    for (GcRoot<mirror::Object>& root : oat_file->GetBssGcRoots()) {
      visitor.VisitRootIfNonNull(root.AddressWithoutBarrier());
    }
  }
}

template <ReadBarrierOption kReadBarrierOption, typename Visitor>
bool ClassTable::Visit(Visitor& visitor) {
  ReaderMutexLock mu(Thread::Current(), lock_);
  for (ClassSet& class_set : classes_) {
    for (TableSlot& table_slot : class_set) {
      if (!visitor(table_slot.Read<kReadBarrierOption>())) {
        return false;
      }
    }
  }
  return true;
}

template <ReadBarrierOption kReadBarrierOption, typename Visitor>
bool ClassTable::Visit(const Visitor& visitor) {
  ReaderMutexLock mu(Thread::Current(), lock_);
  for (ClassSet& class_set : classes_) {
    for (TableSlot& table_slot : class_set) {
      if (!visitor(table_slot.Read<kReadBarrierOption>())) {
        return false;
      }
    }
  }
  return true;
}

inline bool ClassTable::TableSlot::IsNull() const {
  return Read<kWithoutReadBarrier>() == nullptr;
}

template<ReadBarrierOption kReadBarrierOption>
inline ObjPtr<mirror::Class> ClassTable::TableSlot::Read() const {
  return klass_.Read<kReadBarrierOption>();
}

template<typename Visitor>
inline void ClassTable::TableSlot::VisitRoot(const Visitor& visitor) const {
  visitor.VisitRoot(klass_.AddressWithoutBarrier());
}

inline ClassTable::TableSlot::TableSlot(ObjPtr<mirror::Class> klass, uint32_t descriptor_hash)
    : klass_(klass), descriptor_hash_((uint8_t) descriptor_hash) {
  DCHECK_EQ(descriptor_hash, klass->DescriptorHash());
}

inline ClassTable::TableSlot::TableSlot(uint32_t ptr, uint32_t descriptor_hash)
    : klass_(reinterpret_cast<mirror::Class*>(ptr)), descriptor_hash_((uint8_t) descriptor_hash) {
  DCHECK_ALIGNED(ptr, kObjectAlignment);
}

template <typename Filter>
inline void ClassTable::RemoveStrongRoots(const Filter& filter) {
  WriterMutexLock mu(Thread::Current(), lock_);
  strong_roots_.erase(std::remove_if(strong_roots_.begin(), strong_roots_.end(), filter),
                      strong_roots_.end());
}

inline ObjPtr<mirror::Class> ClassTable::LookupByDescriptor(ObjPtr<mirror::Class> klass) {
  uint32_t hash = klass->DescriptorHash();
  std::string temp;
  const char* descriptor = klass->GetDescriptor(&temp);
  return Lookup(descriptor, hash);
}

inline size_t ClassTable::Size() const {
  ReaderMutexLock mu(Thread::Current(), lock_);
  return classes_.size();
}

}  // namespace art

#endif  // ART_RUNTIME_CLASS_TABLE_INL_H_
