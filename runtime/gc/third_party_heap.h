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

#ifndef ART_RUNTIME_GC_THIRD_PARTY_HEAP_H_
#define ART_RUNTIME_GC_THIRD_PARTY_HEAP_H_

#include "base/locks.h"
#include "base/macros.h"
#include "gc/collector/gc_type.h"
#include "gc/collector_type.h"
#include "gc/gc_cause.h"
#include "heap.h"

namespace art {

namespace mirror {
class Object;
}

namespace gc {

namespace collector {
} // namespace collector

namespace third_party_heap {

// API for a ThirdPartyHeap
class ThirdPartyHeap {
 public:
  ThirdPartyHeap(size_t initial_size, size_t capacity);

  ~ThirdPartyHeap();

  // Allow the ThirdPartyHeap to start collecting objects. Called after heap
  // initialization has ocurred.
  void EnableCollection(Thread* tls);

  // Block and suspend mutator thread for GC
  void BlockThreadForCollection(GcCause cause, Thread* self)
    REQUIRES(!*Heap::gc_complete_lock_)
    REQUIRES_SHARED(Locks::mutator_lock_);

  // Check if a given address has been allocated by the ThirdPartyHeap. Note
  // that this check does not use the valid-object bit.
  bool IsObjectInHeapSpace(const void* addr) const REQUIRES_SHARED(Locks::mutator_lock_);

  // Try to allocate an object of size alloc_size. This function can potentially
  // suspend the mutator for a GC in case there is not enough space to fulfill
  // the allocation request.
  mirror::Object* TryToAllocate(Thread* self,
                                size_t alloc_size,
                                size_t* bytes_allocated,
                                size_t* usable_size,
                                size_t* bytes_tl_bulk_allocated)
    REQUIRES(!Locks::thread_suspend_count_lock_, !*Heap::gc_complete_lock_,
             !*Heap::pending_task_lock_)
    REQUIRES(Roles::uninterruptible_)
    REQUIRES_SHARED(Locks::mutator_lock_);

  // Visit all allocated objects. Note that this function may potentially visit
  // dead objects as well.
  template <typename Visitor>
  ALWAYS_INLINE void VisitObjects(Visitor&& visitor)
      REQUIRES_SHARED(Locks::mutator_lock_)
      REQUIRES(!Locks::heap_bitmap_lock_, !*Heap::gc_complete_lock_);

  // Collect dead objects in heap
  collector::GcType CollectGarbage(GcCause gc_cause,
                                   bool clear_soft_references,
                                   uint32_t requested_gc_num);
};

} // namespace third_party_heap
} // namespace gc
} // namespace art

#endif  // ART_RUNTIME_GC_THIRD_PARTY_HEAP_H_
