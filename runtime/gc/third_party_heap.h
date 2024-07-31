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

#include <atomic>
#include <condition_variable>
#include <mutex>

#include "base/locks.h"
#include "base/macros.h"
#include "gc/collector/gc_type.h"
#include "gc/collector_type.h"
#include "gc/gc_cause.h"
#include "gc_root.h"
#include "heap.h"
#include "mirror/object_reference.h"

namespace art {

// State of mutator threads
enum StwState {
  Resumed,
  Suspended,
};

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
  ThirdPartyHeap(size_t initial_size,
                 size_t capacity,
                 bool use_tlab,
                 bool is_zygote_process);

  ~ThirdPartyHeap();

  // Allow the ThirdPartyHeap to start collecting objects. Called after heap
  // initialization has ocurred.
  void EnableCollection(Thread* tls);

  // Return total bytes available to the runtime
  size_t GetTotalMemory();

  // Return free bytes
  size_t GetFreeMemory();

  // Return bytes allocated
  size_t GetBytesAllocated();

  // Return number of GC worker threads
  uint32_t GetNumberOfWorkers();

  // Inform the ThirdPartyHeap of the location of the boot image and its size so
  // that it can keep track of any objects it sees that may be in the boot image
  void SetBootImageSpace(uint32_t boot_image_start_address, uint32_t boot_image_size);

  // Block and suspend mutator thread for GC
  void BlockThreadForCollection(GcCause cause, Thread* self)
    REQUIRES(!*Heap::gc_complete_lock_)
    REQUIRES_SHARED(Locks::mutator_lock_);

  // Check if a given address has been allocated by the ThirdPartyHeap. Note
  // that this check does not use the valid-object bit.
  bool IsObjectInHeapSpace(const void* addr) const REQUIRES_SHARED(Locks::mutator_lock_);

  // Return if the given object obj may move during a GC
  bool IsMovableObject(ObjPtr<mirror::Object> obj) const REQUIRES_SHARED(Locks::mutator_lock_);

  // Set if the current runtime is the Zygote process or not
  void SetIsZygoteProcess(bool is_zygote_process);

  // Set if the current runtime has a Zygote space or not. Called by MMTk after
  // performing the `PreFirstZygoteForkCollection`
  void SetHasZygoteSpace(bool has_zygote_space);

  // Return if the Zygote space has been initialized or not
  bool HasZygoteSpace();

  // Try to allocate an object of size alloc_size. This function can potentially
  // suspend the mutator for a GC in case there is not enough space to fulfill
  // the allocation request.
  mirror::Object* TryToAllocate(Thread* self,
                                size_t alloc_size,
                                bool non_moving,
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
  collector::GcType CollectGarbage(Thread* self, GcCause gc_cause);

  // Delay visiting the referent of a weak reference by enqueuing it to the
  // correct weak reference discovered queue
  void DelayReferenceReferent(ObjPtr<mirror::Class> klass,
                              ObjPtr<mirror::Reference> reference)
      NO_THREAD_SAFETY_ANALYSIS;

  // Set heap state to signify the start of a GC
  void StartGC(Thread* self, GcCause cause);

  // Set heap state to signify the end of a GC. Wake up any threads that were
  // waiting on the GC to complete
  void FinishGC(Thread* self);

  // Request to transition to desired_state
  void Request(StwState desired_state);

  // Hook called before the Zygote is forked. We stop GC worker threads and
  // close file descriptors here
  void PreZygoteFork();

  // Hook called after the Zygote has been forked. We respawn GC worker threads
  // here
  void PostZygoteFork();

  // Perform a full-heap GC just before the Zygote is forked for the first time.
  // This collection should try to move as many objects as possible to compact
  // the Zygote space
  void PreFirstZygoteForkCollection(Thread* self);

 private:
  // Run the companion thread routine to suspend and resume all mutator threads
  void RunCompanionThreadRoutine(Thread* self);

  // Use the thread-local allocation buffer?
  const bool use_tlab_;

  // Is the runtime the Zygote process?
  bool is_zygote_process_;

  // Does the runtime have a Zygote space?
  bool has_zygote_space_;

  // Used to ensure only the first mutator to call `BlockThreadForCollection`
  // performs the `RunCompanionThreadRoutine`
  std::atomic<bool> first_mutator_to_block_;

  // Used by the GC worker(s) to communicate to the mutator thread
  // running `RunCompanionThreadRoutine`.
  // We use C++ stdlib mutex and condvar implementations as ART does not allow
  // waiting on a condvar while holding on to another lock (in this case we
  // would wait on `first_mutator_cond_` while holding on to the mutator_lock).
  std::mutex first_mutator_lock_;

  // Used by the GC worker(s) to communicate to the mutator thread
  // running `RunCompanionThreadRoutine`.
  // We use C++ stdlib mutex and condvar implementations as ART does not allow
  // waiting on a condvar while holding on to another lock (in this case we
  // would wait on `first_mutator_cond_` while holding on to the mutator_lock).
  std::condition_variable first_mutator_cond_;

  // Current state of mutator threads
  StwState current_state_;

  // Desired state of mutator threads
  StwState desired_state_;
};

class ThirdPartyHeapRootVisitor : public RootVisitor {
 public:
  void VisitRoots(mirror::Object*** roots, size_t count, const RootInfo& info) override
      REQUIRES(Locks::mutator_lock_);

  void VisitRoots(mirror::CompressedReference<mirror::Object>** roots,
                  size_t count,
                  const RootInfo& info) override
      REQUIRES(Locks::mutator_lock_);
};

} // namespace third_party_heap
} // namespace gc
} // namespace art

#endif  // ART_RUNTIME_GC_THIRD_PARTY_HEAP_H_
