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

ThirdPartyHeap::ThirdPartyHeap(size_t initial_size, size_t capacity) {
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

size_t ThirdPartyHeap::GetBytesAllocated() {
  return mmtk_get_used_bytes();
}

void ThirdPartyHeap::BlockThreadForCollection(GcCause cause, Thread* self) {
  art::ScopedThreadStateChange tsc(self, ThreadState::kWaitingForGcToComplete);
  Heap* heap = Runtime::Current()->GetHeap();
  {
    MutexLock mu(self, *(heap->gc_complete_lock_));
    // Set the collector_type_running_ to kCollectorThirdPartyHeap so that
    // Heap::WaitForGcToComplete will wait until GC has finished
    heap->collector_type_running_ = kCollectorThirdPartyHeap;
    heap->last_gc_cause_ = cause;
  }
  heap->WaitForGcToComplete(cause, self);
}

bool ThirdPartyHeap::IsObjectInHeapSpace(const void* addr) const {
  return mmtk_is_object_in_heap_space(addr);
}

mirror::Object* ThirdPartyHeap::TryToAllocate(Thread* self,
                                              size_t alloc_size,
                                              size_t* bytes_allocated,
                                              size_t* usable_size,
                                              size_t* bytes_tl_bulk_allocated) {
  // Have to round up allocation size in order to make sure that object starting
  // addresses are aligned
  alloc_size = RoundUp(alloc_size, kObjectAlignment);
  AllocationSemantics semantics = alloc_size <= 16384 ? AllocatorDefault : AllocatorLos;
  uint8_t* ret = (uint8_t *) mmtk_alloc(
    self->GetMmtkMutator(),
    alloc_size,
    kObjectAlignment,
    /* offset= */ 0,
    semantics
  );
  mmtk_post_alloc(self->GetMmtkMutator(), ret, alloc_size, semantics);
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
