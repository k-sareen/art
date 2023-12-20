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
#include "runtime.h"

namespace art {

namespace mirror {
class Object;
}

namespace gc {

namespace third_party_heap {

class MmtkScanObjectVisitor {
 public:
  MmtkScanObjectVisitor(ScanObjectClosure closure) : closure_(closure) {}

  void operator()(ObjPtr<mirror::Object> obj, MemberOffset offset, bool /* is_static */) const ALWAYS_INLINE
      NO_THREAD_SAFETY_ANALYSIS {
    mirror::HeapReference<mirror::Object>* field = obj->GetFieldObjectReferenceAddr<kVerifyNone>(offset);
    void* slot = reinterpret_cast<void*>(field);
    // Don't enqueue null references. We do this here since the object is in the
    // cache line, so this allows for better locality
    if (!field->IsNull()) {
      closure_.invoke(slot);
    }
  }

  void operator()(ObjPtr<mirror::Class> klass, ObjPtr<mirror::Reference> ref) const ALWAYS_INLINE
      NO_THREAD_SAFETY_ANALYSIS {
    // UNUSED(klass);
    // void* referent_slot = reinterpret_cast<void*>(ref->GetReferentReferenceAddr());
    // closure_.invoke(referent_slot);
    Runtime* runtime = Runtime::Current();
    if (UNLIKELY(runtime->IsActiveTransaction())) {
      // In transaction mode, keep the referent alive and avoid any reference processing to avoid the
      // issue of rolling back reference processing.
      void* referent_slot = reinterpret_cast<void*>(ref->GetReferentReferenceAddr());
      closure_.invoke(referent_slot);
    } else {
      ThirdPartyHeap* tp_heap_ = runtime->GetHeap()->GetThirdPartyHeap();
      tp_heap_->DelayReferenceReferent(klass, ref);
    }
  }

  // XXX(kunals): VisitRoot() is required while scanning object references as
  // ART finds roots such as ClassLoaders, DexCaches, etc. during object
  // scanning as opposed to registering them in the VM when they are created
  void VisitRootIfNonNull(mirror::CompressedReference<mirror::Object>* root) const ALWAYS_INLINE
      NO_THREAD_SAFETY_ANALYSIS {
    if (!root->IsNull()) {
      VisitRoot(root);
    }
  }

  void VisitRoot(mirror::CompressedReference<mirror::Object>* root) const ALWAYS_INLINE
      NO_THREAD_SAFETY_ANALYSIS {
    closure_.invoke(reinterpret_cast<void*>(root));
  }

 private:
  ScanObjectClosure closure_;
};

}  // namespace third_party_heap
}  // namespace gc
}  // namespace art

#endif  // MMTK_ART_MMTK_SCAN_OBJECT_VISITOR_H
