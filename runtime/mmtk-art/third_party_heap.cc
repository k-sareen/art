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
#include "mmtk-art/mmtk_gc_thread.h"
#include "mmtk-art/mmtk_upcalls.h"
#include "mmtk.h"
#include "runtime.h"
#include "runtime_globals.h"
#include "scoped_thread_state_change-inl.h"
#include "scoped_thread_state_change.h"
#include "thread.h"

namespace art {
namespace gc {
namespace third_party_heap {

ThirdPartyHeap::ThirdPartyHeap(size_t initial_size,
                               size_t capacity,
                               bool use_tlab)
                            : use_tlab_(use_tlab) {
  mmtk_set_heap_size(initial_size, capacity);
  mmtk_init(&art_upcalls);
  // We create and start the companion thread in EnableCollection
  companion_thread_ = nullptr;
}

ThirdPartyHeap::~ThirdPartyHeap() {}

void ThirdPartyHeap::EnableCollection(Thread* tls) {
  companion_thread_ = reinterpret_cast<void*>(new MmtkVmCompanionThread("MMTk VM Companion Thread"));
  mmtk_initialize_collection(tls);
}

size_t ThirdPartyHeap::GetTotalMemory() {
  return mmtk_get_total_bytes();
}

size_t ThirdPartyHeap::GetFreeMemory() {
  return mmtk_get_free_bytes();
}

size_t ThirdPartyHeap::GetBytesAllocated() {
  return mmtk_get_used_bytes();
}

void ThirdPartyHeap::BlockThreadForCollection(GcCause cause, Thread* self) {
  art::ScopedThreadStateChange tsc(self, ThreadState::kWaitingForGcToComplete);
  Heap* heap = Runtime::Current()->GetHeap();
  {
    MutexLock mu(self, *(heap->gc_complete_lock_));
    // Set the collector_type_running_ to kCollectorTypeThirdPartyHeap so that
    // Heap::WaitForGcToComplete will wait until GC has finished
    heap->collector_type_running_ = kCollectorTypeThirdPartyHeap;
    heap->last_gc_cause_ = cause;
  }
  heap->WaitForGcToComplete(cause, self);
}

bool ThirdPartyHeap::IsObjectInHeapSpace(const void* addr) const {
  return mmtk_is_object_in_heap_space(addr);
}

bool ThirdPartyHeap::IsMovableObject(ObjPtr<mirror::Object> obj) const {
  return mmtk_is_object_movable(obj.Ptr());
}

mirror::Object* ThirdPartyHeap::TryToAllocate(Thread* self,
                                              size_t alloc_size,
                                              bool non_moving,
                                              size_t* bytes_allocated,
                                              size_t* usable_size,
                                              size_t* bytes_tl_bulk_allocated) {
  AllocationSemantics semantics = AllocatorDefault;
  if (non_moving) {
    semantics = AllocatorNonMoving;
  }
  if (alloc_size >= Heap::kMinLargeObjectThreshold) {
    // Since LOS is non-moving anyway, we don't need to check if `non_moving` is true
    semantics = AllocatorLos;
  }

  MmtkMutator mmtk_mutator = self->GetMmtkMutator();
  DCHECK(mmtk_mutator != nullptr) << "mmtk_mutator for thread " << self << " is nullptr!";

  // XXX(kunals): We don't check if the semantics are `AllocatorDefault` since
  // the NoGC plan does not use a separate non-moving space and hence always
  // uses the TLAB regardless of object allocation. This may increase the
  // overhead of this function. Evaluate how many non-moving objects exist and
  // if there is a perceivable overhead in the allocation rate
  if (use_tlab_) {
    mmtk_set_default_thread_local_cursor_limit(mmtk_mutator, self->GetMmtkBumpPointerValues());
  }

  uint8_t* ret = (uint8_t *) mmtk_alloc(
    mmtk_mutator,
    alloc_size,
    kObjectAlignment,
    /* offset= */ 0,
    semantics
  );

  if (use_tlab_) {
    self->SetMmtkBumpPointerValues(
      mmtk_get_default_thread_local_cursor_limit(mmtk_mutator)
    );
  }

  // XXX(kunals): If we actually add per-object metadata then we need to inline
  // this call everywhere
  mmtk_post_alloc(mmtk_mutator, ret, alloc_size, semantics);
  if (LIKELY(ret != nullptr)) {
    *bytes_allocated = alloc_size;
    *usable_size = alloc_size;
    *bytes_tl_bulk_allocated = alloc_size;
  }
  return reinterpret_cast<mirror::Object*>(ret);
}

collector::GcType ThirdPartyHeap::CollectGarbage(Thread* self, GcCause cause) {
  UNUSED(cause);
  mmtk_handle_user_collection_request(
    reinterpret_cast<void*>(self),
    /* force= */ true,
    /* exhaustive= */ true
  );
  return collector::kGcTypeFull;
}

void ThirdPartyHeap::FinishGC(Thread* self) {
  Heap* heap = Runtime::Current()->GetHeap();
  MutexLock mu(self, *heap->gc_complete_lock_);
  heap->collector_type_running_ = kCollectorTypeNone;
  heap->last_gc_type_ = collector::kGcTypeFull;

  heap->running_collection_is_blocking_ = false;
  heap->gcs_completed_.fetch_add(1, std::memory_order_release);
  heap->old_native_bytes_allocated_.store(heap->GetNativeBytes());

  // Wake anyone who may have been waiting for the GC to complete
  heap->gc_complete_cond_->Broadcast(self);
}

}  // namespace third_party_heap
}  // namespace gc
}  // namespace art
