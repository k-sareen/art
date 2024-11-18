/*
 * Copyright (C) 2023 The Android Open Source Project
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

#include "gc/third_party_heap.h"

#if ART_USE_MMTK_EXTREME_ASSERT
#include <mutex>
#include <unordered_set>
#endif  // ART_USE_MMTK_EXTREME_ASSERT

#include "base/globals.h"
#include "class_table.h"
#include "gc/verification.h"
#include "gc/verification-inl.h"
#include "mirror/class-refvisitor-inl.h"
#include "mirror/object-inl.h"
#include "mmtk_root_visitor.h"
#include "mmtk.h"

namespace art {
namespace gc {
namespace third_party_heap {

MmtkRootVisitor::MmtkRootVisitor(SlotsClosure closure, ThirdPartyHeap* tp_heap) : closure_(closure), cursor_(0), tp_heap_(tp_heap) {
  RustBuffer buf = closure_.invoke(NULL, 0, 0);
  buffer_ = buf.buf;
  capacity_ = buf.capacity;
}

MmtkRootVisitor::~MmtkRootVisitor() {
  if (cursor_ > 0) {
    FlushBuffer();
  }

  if (buffer_ != NULL) {
    mmtk_release_rust_buffer(buffer_, cursor_, capacity_);
  }
}

void MmtkRootVisitor::VisitRoots(mirror::Object*** roots,
                size_t count,
                [[maybe_unused]] const RootInfo& info) {
  for (size_t i = 0; i < count; ++i) {
    auto* root = roots[i];
    if (kIsDebugBuild) {
      mirror::Object* obj = reinterpret_cast<StackReference<mirror::Object>*>(root)->AsMirrorPtr();
      auto ref = StackReference<mirror::Object>::FromMirrorPtr(obj);
      const art::gc::Verification* verification = art::Runtime::Current()->GetHeap()->GetVerification();
      CHECK(verification->IsValidObject(ref.AsMirrorPtr()))
        << "MmtkRootVisitor "
        << ref.AsMirrorPtr()
        << " "
        << info.ToString()
        << " is not a valid object!"
        << verification->DumpRAMAroundAddress((uintptr_t)ref.AsMirrorPtr(), 128);
    }
#if !ART_USE_MMTK_EXTREME_ASSERT
    buffer_[cursor_++] = (void*) root; // ref.AsMirrorPtr();
#else
    DCHECK(tp_heap_->slot_set_ != nullptr);
    std::pair<std::unordered_set<void*>::iterator, bool> ret;
    {
      std::unique_lock mu(tp_heap_->slot_set_mutex_);
      ret = tp_heap_->slot_set_->insert((void*) root);
    }
    if (ret.second) {
      buffer_[cursor_++] = (void*) root; // ref.AsMirrorPtr();
    }
#endif  // !ART_USE_MMTK_EXTREME_ASSERT
    if (cursor_ >= capacity_) {
      FlushBuffer();
    }
  }
}

void MmtkRootVisitor::VisitRoots(mirror::CompressedReference<mirror::Object>** roots,
                size_t count,
                [[maybe_unused]] const RootInfo& info) {
  for (size_t i = 0; i < count; ++i) {
    if (kIsDebugBuild) {
      auto* obj = roots[i]->AsMirrorPtr();
      const art::gc::Verification* verification = art::Runtime::Current()->GetHeap()->GetVerification();
      CHECK(verification->IsValidObject(obj))
        << "MmtkRootVisitor "
        << obj
        << " "
        << info.ToString()
        << " is not a valid object!"
        << verification->DumpRAMAroundAddress((uintptr_t)obj, 128);
    }
#if !ART_USE_MMTK_EXTREME_ASSERT
    buffer_[cursor_++] = (void*) roots[i]; // root;
#else
    DCHECK(tp_heap_->slot_set_ != nullptr);
    std::pair<std::unordered_set<void*>::iterator, bool> ret;
    {
      std::unique_lock mu(tp_heap_->slot_set_mutex_);
      ret = tp_heap_->slot_set_->insert((void*) roots[i]);
    }
    if (ret.second) {
      buffer_[cursor_++] = (void*) roots[i]; // ref.AsMirrorPtr();
    }
#endif  // !ART_USE_MMTK_EXTREME_ASSERT
    if (cursor_ >= capacity_) {
      FlushBuffer();
    }
  }
}

void MmtkRootVisitor::FlushBuffer() {
  if (cursor_ > 0) {
    RustBuffer buf = closure_.invoke(buffer_, cursor_, capacity_);
    buffer_ = buf.buf;
    capacity_ = buf.capacity;
    cursor_ = 0;
  }
}

void MmtkRootVisitor::Visit(ObjPtr<mirror::ClassLoader> class_loader) {
  ClassTable* const class_table = class_loader->GetClassTable();
  if (class_table != nullptr) {
    class_table->VisitRoots(*this);
  }
}

void MmtkRootVisitor::Visit(ObjPtr<mirror::DexCache> dex_cache) {
  dex_cache->VisitNativeRoots<kVerifyNone, /* kReadBarrierOption= */ kWithoutReadBarrier>(*this);
}

}  // namespace third_party_heap
}  // namespace gc
}  // namespace art
