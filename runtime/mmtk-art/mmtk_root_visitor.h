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

#ifndef MMTK_ART_MMTK_ROOT_VISITOR_H
#define MMTK_ART_MMTK_ROOT_VISITOR_H

#include "class_linker.h"
#include "gc/third_party_heap.h"
#include "mmtk.h"

#include <unordered_set>

namespace art {

namespace mirror {
class Object;
}

namespace gc {

namespace third_party_heap {

class MmtkRootVisitor : public ThirdPartyHeapRootVisitor, public ClassLoaderVisitor, public DexCacheVisitor {
 public:
  MmtkRootVisitor(NodesClosure closure);

  ~MmtkRootVisitor();

  void VisitRoots(mirror::Object*** roots, size_t count, const RootInfo& info) override
      REQUIRES(Locks::mutator_lock_);

  void VisitRoots(mirror::CompressedReference<mirror::Object>** roots,
                  size_t count,
                  const RootInfo& info) override
      REQUIRES(Locks::mutator_lock_);

  void Visit(ObjPtr<mirror::ClassLoader> class_loader) override
      REQUIRES_SHARED(Locks::classlinker_classes_lock_, Locks::mutator_lock_);

  void Visit(ObjPtr<mirror::DexCache> dex_cache) override
      NO_THREAD_SAFETY_ANALYSIS;

  void VisitRootIfNonNull(mirror::CompressedReference<mirror::Object>* root) const ALWAYS_INLINE
      NO_THREAD_SAFETY_ANALYSIS {
    if (!root->IsNull()) {
      VisitRoot(root);
    }
  }

  void VisitRoot(mirror::CompressedReference<mirror::Object>* root) const ALWAYS_INLINE
      NO_THREAD_SAFETY_ANALYSIS {
    const_cast<MmtkRootVisitor&>(*this).VisitRoots(&root, 1, RootInfo(kRootVMInternal));
  }

 private:
  void FlushBuffer();

  NodesClosure closure_;
  void** buffer_;
  size_t capacity_;
  size_t cursor_;
  // std::unordered_set<mirror::Object*> class_set_;
};

}  // namespace third_party_heap
}  // namespace gc
}  // namespace art

#endif  // MMTK_ART_MMTK_ROOT_VISITOR_H
