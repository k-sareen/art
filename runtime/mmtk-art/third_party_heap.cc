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
#include <atomic>

#if ART_USE_MMTK_EXTREME_ASSERT
#include <unordered_set>
#endif  // ART_USE_MMTK_EXTREME_ASSERT

#include "gc/collector/gc_type.h"
#include "gc/collector_type.h"
#include "gc/gc_cause.h"
#include "gc/reference_processor.h"
#include "handle_scope-inl.h"
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
                               size_t growth_limit,
                               double target_utilization,
                               size_t min_free,
                               size_t max_free,
                               double foreground_heap_growth_multiplier,
                               bool use_tlab,
                               bool is_zygote_process)
                            : use_tlab_(use_tlab),
                              is_zygote_process_(is_zygote_process),
                              has_zygote_space_(false),
                              first_mutator_to_block_(false),
                              current_state_(StwState::Resumed),
                              desired_state_(StwState::Resumed) {
#if ART_USE_WRITE_BARRIER
  MmtkPlanSelector plan = MmtkPlanSelector::StickyImmix;
#else
  MmtkPlanSelector plan = MmtkPlanSelector::Immix;
#endif  // ART_USE_WRITE_BARRIER
  mmtk_set_heap_size(initial_size,
                     capacity,
                     growth_limit,
                     target_utilization,
                     min_free,
                     max_free,
                     foreground_heap_growth_multiplier);
  mmtk_init(&art_upcalls, plan, is_zygote_process_);
}

ThirdPartyHeap::~ThirdPartyHeap() {}

void ThirdPartyHeap::EnableCollection(Thread* tls) {
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

uint32_t ThirdPartyHeap::GetNumberOfWorkers() {
  return mmtk_get_number_of_workers();
}

void ThirdPartyHeap::ClampGrowthLimit() {
  mmtk_clamp_growth_limit();
}

void ThirdPartyHeap::ClearGrowthLimit() {
  mmtk_clear_growth_limit();
}

bool ThirdPartyHeap::ClampMaxHeapSize(size_t max) {
  return mmtk_clamp_max_heap_size(max);
}

void ThirdPartyHeap::SetIsJankPerceptible(bool is_jank_perceptible) {
  mmtk_set_is_jank_perceptible(is_jank_perceptible);
}

void ThirdPartyHeap::GrowHeapOnJankPerceptibleSwitch() {
  mmtk_grow_heap_on_jank_perceptible_switch();
}

void ThirdPartyHeap::SetBootImageSpace(uint32_t boot_image_start_address, uint32_t boot_image_size) {
  mmtk_set_image_space(boot_image_start_address, boot_image_size);
}

void ThirdPartyHeap::SetPointerSize(size_t pointer_size) {
  mmtk_set_runtime_pointer_size(pointer_size);
}

bool ThirdPartyHeap::IsObjectInHeapSpace(const void* addr) const {
  return mmtk_is_object_in_heap_space(addr);
}

bool ThirdPartyHeap::IsMovableObject(ObjPtr<mirror::Object> obj) const {
  mirror::Object* object = obj.Ptr();
  if (mmtk_is_object_pinned(object)) {
    return false;
  }
  return mmtk_is_object_movable(object);
}

void ThirdPartyHeap::SetIsZygoteProcess(bool is_zygote_process) {
  is_zygote_process_ = is_zygote_process;
  mmtk_set_is_zygote_process(is_zygote_process_);
}

void ThirdPartyHeap::SetHasZygoteSpace(bool has_zygote_space) {
  has_zygote_space_ = has_zygote_space;
}

bool ThirdPartyHeap::HasZygoteSpace() {
  // This is set by MMTk after performing the pre-first Zygote fork GC
  // Do not set yourself!
  return has_zygote_space_;
}

void ThirdPartyHeap::Request(StwState desired_state) {
  std::unique_lock mu(first_mutator_lock_);
  desired_state_ = desired_state;
  first_mutator_cond_.notify_all();
  first_mutator_cond_.wait(mu, [&]{ return current_state_ == desired_state; });
}

// Suspend all mutator threads. Acquires exclusive lock on mutator_lock_
EXCLUSIVE_LOCK_FUNCTION(Locks::mutator_lock_)
static void SuspendAll() {
  Runtime::Current()->GetThreadList()->SuspendAll(__FUNCTION__, /* long_suspend= */ false);
}

// Resume all mutator threads. Releases exclusive lock on mutator_lock_
UNLOCK_FUNCTION(Locks::mutator_lock_)
static void ResumeAll() {
  Runtime::Current()->GetThreadList()->ResumeAll();
}

void ThirdPartyHeap::RunCompanionThreadRoutine(Thread* self) {
  art::ScopedThreadStateChange tsc(self, ThreadState::kWaitingForGcToComplete);
#if ART_USE_MMTK_SANITY
  const Verification* verification = Runtime::Current()->GetHeap()->GetVerification();
#endif  // ART_USE_MMTK_SANITY

  {
    std::unique_lock mu(first_mutator_lock_);
    first_mutator_cond_.wait(mu, [&]{ return desired_state_ == StwState::Suspended; });
  }

  SuspendAll();


#if ART_USE_MMTK_SANITY
  // We run SanityPostGC after the GC has completed the transitive closure so
  // that we still have all forwarding pointers etc.
  verification->SanityPreGC();
#endif  // ART_USE_MMTK_SANITY

  {
    std::unique_lock mu(first_mutator_lock_);
    current_state_ = StwState::Suspended;
    first_mutator_cond_.notify_all();
    first_mutator_cond_.wait(mu, [&]{ return desired_state_ == StwState::Resumed; });
  }

  ResumeAll();

  {
    std::unique_lock mu(first_mutator_lock_);
    current_state_ = StwState::Resumed;
    first_mutator_cond_.notify_all();
  }
}

void ThirdPartyHeap::BlockThreadForCollection(Thread* self) {
  DCHECK(self->GetMmtkMutator() != nullptr);

  Heap* heap = Runtime::Current()->GetHeap();
  VLOG(threads) << "Blocking GC requested by thread: " << *self;

  if (!first_mutator_to_block_.exchange(true)) {
    VLOG(threads) << "First thread to block: " << *self;
    RunCompanionThreadRoutine(self);
    VLOG(threads) << "First thread to block is waking up: " << *self;
    first_mutator_to_block_.store(false);
  } else {
    // XXX(kunals): There is a subtle race condition here if two threads are about to both block
    // but the current thread gets suspended when it acquires the gc_complete_lock_ below. In
    // such a case the current thread would get stuck in a loop waiting for a non-existent GC
    // to complete if the next_gc_num was calculated after the mutex was acquired.
    // Hence, calculate the next_gc_num before acquiring the lock to avoid this rare infinite loop.
    uint32_t next_gc_num = heap->GetCurrentGcNum() + 1;
    CHECK(first_mutator_to_block_.load()) << "First mutator to block is not set! " << *self;
    art::ScopedThreadStateChange tsc(self, ThreadState::kWaitingForGcToComplete);
    MutexLock mu(self, *heap->gc_complete_lock_);
    heap->gc_complete_cond_->CheckSafeToWait(self);
    // Waiting on the GC number to go up is fine as the number does not go up for fake GCs
    while (heap->GetCurrentGcNum() < next_gc_num) {
      heap->gc_complete_cond_->Wait(self);
    }
  }
}

mirror::Object* ThirdPartyHeap::TryToAllocate(Thread* self,
                                              size_t alloc_size,
                                              bool non_moving,
                                              size_t* bytes_allocated,
                                              size_t* usable_size,
                                              size_t* bytes_tl_bulk_allocated,
                                              ObjPtr<mirror::Class>* klass) {
  // Make sure there is no pending exception since we may need to throw an OOME.
  self->AssertNoPendingException();

  // Preserve the klass as a root so that it gets updated properly after GC
  StackHandleScope<1> hs(self);
  HandleWrapperObjPtr<mirror::Class> h_klass(hs.NewHandleWrapper(klass));

  AllocationSemantics semantics = AllocatorDefault;
  // TODO(kunals): Pin objects for Immix-based plans instead of using the non-moving allocator
  if (non_moving
      && (!is_zygote_process_
          || (is_zygote_process_ && has_zygote_space_))) {
    semantics = AllocatorNonMoving;
  }
  if (alloc_size >= Heap::kMinLargeObjectThreshold) {
    // Since LOS is non-moving anyway, we don't need to check if `non_moving` is true
    if ((*klass)->IsPrimitiveArray()) {
      // We can handle large primitive arrays specially since we don't need to scan them
      semantics = AllocatorLos;
    } else {
      semantics = AllocatorLos;
    }
  }

  MmtkMutator mmtk_mutator = self->GetMmtkMutator();
  DCHECK(mmtk_mutator != nullptr) << "mmtk_mutator for thread " << *self << " is nullptr!";

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

  if (LIKELY(ret != nullptr)) {
    // XXX(kunals): If we actually add per-object metadata then we need to inline
    // this call everywhere
    mmtk_post_alloc(mmtk_mutator, ret, alloc_size, semantics);
    *bytes_allocated = alloc_size;
    *usable_size = alloc_size;
    *bytes_tl_bulk_allocated = alloc_size;

    if (non_moving && semantics == AllocatorDefault) {
      bool pinned = mmtk_pin_object(ret);
      CHECK(pinned) << "Allocated non-moving object at "
                    << reinterpret_cast<mirror::Object*>(ret)
                    << " in default space but it could not be pinned";
    }
  }
  return reinterpret_cast<mirror::Object*>(ret);
}

collector::GcType ThirdPartyHeap::CollectGarbage(Thread* self, GcCause cause) {
  Heap* heap = Runtime::Current()->GetHeap();
  {
    MutexLock mu(self, *heap->gc_complete_lock_);
    if (heap->collector_type_running_ != kCollectorTypeNone) {
      // Someone else has scheduled a GC for us. Wait until the GC has finished
      art::ScopedThreadStateChange tsc(self, ThreadState::kWaitingForGcToComplete);
      VLOG(heap) << "Someone else has scheduled a GC for us. Us= "
                 << *self
                 << ", running GC "
                 << heap->collector_type_running_
                 << " "
                 << heap->last_gc_cause_;
      uint32_t next_gc_num = heap->GetCurrentGcNum() + 1;
      heap->gc_complete_cond_->CheckSafeToWait(self);
      while (heap->GetCurrentGcNum() < next_gc_num) {
        heap->gc_complete_cond_->Wait(self);
      }
      // Since someone else has scheduled a GC for us, we just return instead of
      // scheduling another GC
      return collector::kGcTypeFull;
    } else {
      heap->collector_type_running_ = kCollectorTypeThirdPartyHeap;
      heap->last_gc_cause_ = cause;
    }
  }
  bool ran_gc = mmtk_handle_user_collection_request(
    reinterpret_cast<void*>(self),
    /* force= */ true,
    /* exhaustive= */ true
  );
  if (ran_gc) {
    return collector::kGcTypeFull;
  } else {
    // If we didn't run a GC (for example NoGC) then we reset the running collector and return
    // that no GC was ran
    {
      MutexLock mu(self, *heap->gc_complete_lock_);
      DCHECK_EQ(heap->collector_type_running_, kCollectorTypeThirdPartyHeap);
      heap->collector_type_running_ = kCollectorTypeNone;
    }
    return collector::kGcTypeNone;
  }
}

void ThirdPartyHeap::DelayReferenceReferent(ObjPtr<mirror::Class> klass,
                                            ObjPtr<mirror::Reference> reference) {
  Heap* heap = Runtime::Current()->GetHeap();
  heap->GetReferenceProcessor()->DelayReferenceReferentTPH(klass, reference);
}

void ThirdPartyHeap::StartGC(Thread* self, GcCause cause) {
  Heap* heap = Runtime::Current()->GetHeap();
  MutexLock mu(self, *heap->gc_complete_lock_);

  // Wait until there is no other (fake) GC running before attempting to start a GC
  heap->gc_complete_cond_->CheckSafeToWait(self);
  while ((heap->collector_type_running_ != kCollectorTypeNone
          && heap->collector_type_running_ != kCollectorTypeThirdPartyHeap)
          || heap->disable_moving_gc_count_ != 0) {
    heap->gc_complete_cond_->Wait(self);
  }

  // Only set collector_type_running_ if it was previously None
  if (heap->collector_type_running_ == kCollectorTypeNone) {
    heap->collector_type_running_ = kCollectorTypeThirdPartyHeap;
    heap->last_gc_cause_ = cause;
  } else {
    // XXX(kunals): The collector type can be something other than None only if
    // `CollectGarbage` has been called since we have waited for other fake GCs
    // to finish above. Assert that is the case
    CHECK_EQ(heap->collector_type_running_, kCollectorTypeThirdPartyHeap)
      << "Cannot run StartGC when some other GC is running. Running GC "
      << heap->collector_type_running_
      << ", cause "
      << PrettyCause(heap->last_gc_cause_);
  }
#if ART_USE_MMTK_EXTREME_ASSERT
  slot_set_.reset(new std::unordered_set<void*>());
#endif  // ART_USE_MMTK_EXTREME_ASSERT
  is_transaction_active_ = Runtime::Current()->IsActiveTransaction();
}

void ThirdPartyHeap::FinishGC(Thread* self) {
  Heap* heap = Runtime::Current()->GetHeap();
  MutexLock mu(self, *heap->gc_complete_lock_);
  heap->collector_type_running_ = kCollectorTypeNone;
  heap->last_gc_type_ = collector::kGcTypeFull;

  heap->running_collection_is_blocking_ = false;
  heap->gcs_completed_.fetch_add(1, std::memory_order_release);
  heap->old_native_bytes_allocated_.store(heap->GetNativeBytes());

  is_transaction_active_ = false;

  // Wake anyone who may have been waiting for the GC to complete
  heap->gc_complete_cond_->Broadcast(self);
}

void ThirdPartyHeap::PreZygoteFork() {
  mmtk_pre_zygote_fork();
}

void ThirdPartyHeap::PostZygoteFork() {
  // XXX(kunals): tls is unused so passing a nullptr is fine
  mmtk_post_zygote_fork(/* tls= */ nullptr);
}

void ThirdPartyHeap::PreFirstZygoteForkCollection(Thread* self) {
  mmtk_handle_pre_first_zygote_fork_collection_request(reinterpret_cast<void*>(self));
}

void ThirdPartyHeap::Shutdown() {}

}  // namespace third_party_heap
}  // namespace gc
}  // namespace art
