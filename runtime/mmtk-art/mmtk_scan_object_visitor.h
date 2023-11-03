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

#ifndef MMTK_ART_MMTK_SCAN_OBJECT_VISITOR_H
#define MMTK_ART_MMTK_SCAN_OBJECT_VISITOR_H

#include "gc/third_party_heap.h"
#include "mirror/class.h"
#include "mirror/reference.h"
#include "mmtk.h"

#include <iostream>

namespace art {

namespace mirror {
class Object;
}

namespace gc {

namespace third_party_heap {

class MmtkScanObjectVisitor {
 public:
  MmtkScanObjectVisitor(void (*closure)(void* edge)) : closure_(closure) {}

  void operator()(ObjPtr<mirror::Object> obj, MemberOffset offset, bool /* is_static */) const ALWAYS_INLINE
      REQUIRES(Locks::mutator_lock_, Locks::heap_bitmap_lock_) {
    mirror::HeapReference<mirror::Object>* field = obj->GetFieldObjectReferenceAddr<kVerifyNone>(offset);
    void* slot = reinterpret_cast<void*>(field);
    closure_(slot);
  }

  void operator()(ObjPtr<mirror::Class> klass ATTRIBUTE_UNUSED, ObjPtr<mirror::Reference> ref) const ALWAYS_INLINE
      REQUIRES(Locks::mutator_lock_, Locks::heap_bitmap_lock_) {
    void* referent_slot = reinterpret_cast<void*>(ref->GetReferentReferenceAddr());
    // collector_->DelayReferenceReferent(klass, ref);
    closure_(referent_slot);
  }

  // TODO(kunals): Investigate why VisitRoot() is required for scanning object references
  void VisitRootIfNonNull(mirror::CompressedReference<mirror::Object>* root) const ALWAYS_INLINE
      NO_THREAD_SAFETY_ANALYSIS {
    if (!root->IsNull()) {
      VisitRoot(root);
    }
  }

  void VisitRoot(mirror::CompressedReference<mirror::Object>* root) const ALWAYS_INLINE
      NO_THREAD_SAFETY_ANALYSIS {
    closure_(reinterpret_cast<void*>(root));
  }

 private:
  void (*closure_)(void* edge);
};

}  // namespace third_party_heap
}  // namespace gc
}  // namespace art

#endif  // MMTK_ART_MMTK_SCAN_OBJECT_VISITOR_H
