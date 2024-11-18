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

#if ART_USE_MMTK_EXTREME_ASSERT
#include <mutex>
#include <unordered_set>
#endif  // ART_USE_MMTK_EXTREME_ASSERT

#include "gc/verification.h"
#include "gc/verification-inl.h"
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
  MmtkScanObjectVisitor(ScanObjectClosure closure)
      : is_nursery_collection_(mmtk_is_nursery_collection()), closure_(closure) {
    tp_heap_ = Runtime::Current()->GetHeap()->GetThirdPartyHeap();
  }

  void operator()(ObjPtr<mirror::Object> obj, MemberOffset offset, bool /* is_static */) const ALWAYS_INLINE
      NO_THREAD_SAFETY_ANALYSIS {
    mirror::HeapReference<mirror::Object>* field = obj->GetFieldObjectReferenceAddr<kVerifyNone>(offset);
    void* slot = reinterpret_cast<void*>(field);
    // Don't enqueue null references. We do this here since the object is in the
    // cache line, so this allows for better locality
    if (!field->IsNull()) {
      // XXX(kunals): Disabling this check temporarily because we can fail this check if we load
      // the class word and we were trying to forward an object at the same time.
      // TODO(kunals): Update the assertion to check if it object is getting forwarded at the same time
      // if (kIsDebugBuild) {
      //   const Verification* verification = Runtime::Current()->GetHeap()->GetVerification();
      //   CHECK(verification->IsValidObject(field->AsMirrorPtr()))
      //     << "ScanObject "
      //     << obj
      //     << ": slot "
      //     << slot
      //     << " "
      //     << field->AsMirrorPtr()
      //     << " is not a valid object!\n";
      // }
#if !ART_USE_MMTK_EXTREME_ASSERT
      closure_.invoke(slot);
#else
      DCHECK(tp_heap_->slot_set_ != nullptr);
      std::pair<std::unordered_set<void*>::iterator, bool> ret;
      {
        std::unique_lock mu(tp_heap_->slot_set_mutex_);
        ret = tp_heap_->slot_set_->insert(slot);
      }
      if (ret.second) {
        closure_.invoke(slot);
      }
#endif  // !ART_USE_MMTK_EXTREME_ASSERT
    }
  }

  void operator()(ObjPtr<mirror::Class> klass, ObjPtr<mirror::Reference> ref) const ALWAYS_INLINE
      NO_THREAD_SAFETY_ANALYSIS {
    if (UNLIKELY(tp_heap_->IsActiveTransaction())) {
      // In transaction mode, keep the referent alive and avoid any reference processing to avoid the
      // issue of rolling back reference processing.
      void* referent_slot = reinterpret_cast<void*>(ref->GetReferentReferenceAddr());
      closure_.invoke(referent_slot);
    } else {
      if (LIKELY(is_nursery_collection_)) {
        // Treat java.lang.ref.Reference as a strong reference and trace the referent
        void* referent_slot = reinterpret_cast<void*>(ref->GetReferentReferenceAddr());
        closure_.invoke(referent_slot);
      } else {
        tp_heap_->DelayReferenceReferent(klass, ref);
      }
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
    DCHECK(!root->IsNull());
    // TODO(kunals): See above TODO
    // if (kIsDebugBuild) {
    //   const Verification* verification = Runtime::Current()->GetHeap()->GetVerification();
    //   CHECK(verification->IsValidObject(root->AsMirrorPtr()))
    //     << "ScanObject root "
    //     << root->AsMirrorPtr()
    //     << " is not a valid object!";
    // }
#if !ART_USE_MMTK_EXTREME_ASSERT
    closure_.invoke(reinterpret_cast<void*>(root));
#else
    DCHECK(tp_heap_->slot_set_ != nullptr);
    std::pair<std::unordered_set<void*>::iterator, bool> ret;
    {
      std::unique_lock mu(tp_heap_->slot_set_mutex_);
      ret = tp_heap_->slot_set_->insert(reinterpret_cast<void*>(root));
    }
    if (ret.second) {
      closure_.invoke(reinterpret_cast<void*>(root));
    }
#endif  // !ART_USE_MMTK_EXTREME_ASSERT
  }

 private:
  const bool is_nursery_collection_;
  ScanObjectClosure closure_;
  ThirdPartyHeap* tp_heap_;
};

}  // namespace third_party_heap
}  // namespace gc
}  // namespace art

#endif  // MMTK_ART_MMTK_SCAN_OBJECT_VISITOR_H
