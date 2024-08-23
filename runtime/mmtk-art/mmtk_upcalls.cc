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

#include "gc/gc_cause.h"
#include "gc/reference_processor.h"
#include "gc/task_processor.h"
#include "gc/third_party_heap.h"
#include "jni/java_vm_ext.h"
#include "mmtk_gc_thread.h"
#include "mmtk_is_marked_visitor.h"
#include "mmtk_mark_object_visitor.h"
#include "mmtk_root_visitor.h"
#include "mmtk_scan_object_visitor.h"
#include "mmtk_upcalls.h"
#include "mirror/object-inl.h"
#include "mirror/object-refvisitor-inl.h"
#include "thread.h"
#include "thread_list.h"

namespace art {
class Thread;
}  // namespace art

REQUIRES_SHARED(art::Locks::mutator_lock_)
static size_t size_of(void* object) {
  DCHECK(object != nullptr);
  art::mirror::Object* obj = reinterpret_cast<art::mirror::Object*>(object);
  return obj->SizeOf();
}

static void scan_object(void* object, ScanObjectClosure closure) {
  DCHECK(object != nullptr);
  art::gc::third_party_heap::MmtkScanObjectVisitor visitor(closure);
  art::mirror::Object* obj = reinterpret_cast<art::mirror::Object*>(object);
  obj->VisitReferences</* kVisitNativeRoots= */ true, art::kVerifyNone, art::kWithoutReadBarrier>(visitor, visitor);
}

REQUIRES(art::Roles::uninterruptible_)
REQUIRES_SHARED(art::Locks::mutator_lock_)
static void block_for_gc(void* tls) {
#define PERFORM_SUSPENDING_OPERATION(self, op)                                              \
  [&]() REQUIRES(art::Roles::uninterruptible_) REQUIRES_SHARED(art::Locks::mutator_lock_) { \
    if (!self->IsThreadSuspensionAllowable()) {                                             \
      art::ScopedAllowThreadSuspension ats;                                                 \
      return (op);                                                                          \
    } else {                                                                                \
      return (op);                                                                          \
    }                                                                                       \
  }()
  DCHECK(tls != nullptr);
  art::Thread* self = reinterpret_cast<art::Thread*>(tls);
  VLOG(threads) << "Block for GC requested: " << *self;
  art::gc::third_party_heap::ThirdPartyHeap* tp_heap =
    art::Runtime::Current()->GetHeap()->GetThirdPartyHeap();
  PERFORM_SUSPENDING_OPERATION(self, tp_heap->BlockThreadForCollection(self));
#undef PERFORM_SUSPENDING_OPERATION
  VLOG(threads) << "Block for GC finished: " << *self;
}

static void spawn_gc_thread(void* tls, GcThreadKind kind, void* ctx) {
  UNUSED(tls);
  switch (kind) {
    case MmtkGcWorker: {
      new art::MmtkCollectorThread("MMTk Collector Thread", ctx);
      break;
    }
    default: {
      LOG(FATAL) << "Unexpected GC thread kind: " << kind;
      UNREACHABLE();
    }
  }
}

static void suspend_mutators(void* tls) {
  DCHECK(tls != nullptr);
  VLOG(threads) << "Suspend all mutators. Sending request to first mutator thread.";
  art::Thread* self = reinterpret_cast<art::Thread*>(tls);
  art::gc::third_party_heap::ThirdPartyHeap* tp_heap =
    art::Runtime::Current()->GetHeap()->GetThirdPartyHeap();

  tp_heap->StartGC(self, art::gc::kGcCauseForAlloc);

  tp_heap->Request(art::StwState::Suspended);
  VLOG(threads) << "Suspend request sent to first mutator thread.";
}

REQUIRES(!art::Locks::thread_list_lock_)
static void resume_mutators(void* tls) {
  DCHECK(tls != nullptr);
  VLOG(threads) << "Resume all mutators. Sending request to first mutator thread.";
  art::Thread* self = reinterpret_cast<art::Thread*>(tls);
  art::Runtime* runtime = art::Runtime::Current();
  {
    art::MutexLock mu(self, *art::Locks::thread_list_lock_);

    // XXX(kunals): After a GC we need to reset the TLAB cursor and limit to 0
    // to reflect the allocator reset in release_mutator. Fix after
    // https://github.com/mmtk/mmtk-core/issues/1017
    runtime->GetThreadList()->ForEach([](art::Thread* thread) {
      thread->SetMmtkBumpPointerValues(MmtkBumpPointer {});
    });
  }

  art::gc::Heap* heap = runtime->GetHeap();
  art::gc::third_party_heap::ThirdPartyHeap* tp_heap = heap->GetThirdPartyHeap();

  tp_heap->Request(art::StwState::Resumed);
  VLOG(threads) << "Resume request sent to first mutator thread.";

  tp_heap->FinishGC(self);

  // Collect cleared references and enqueue cleared references
  // Do this after the GC has officially finished since otherwise
  // we can deadlock.
  // XXX(kunals): We don't need to use the return value of CollectClearedReferences
  // as we directly enqueue the task into the HeapTaskDaemon thread.
  // For more context read the comment for `kAsyncReferenceQueueAdd` inside reference_processor.cc
  heap->GetReferenceProcessor()->CollectClearedReferences(self);
  // Unload native libraries for class unloading. We do this after calling FinishGC to prevent
  // deadlocks in case the JNI_OnUnload function does allocations.
  {
    art::ScopedObjectAccess soa(self);
    soa.Vm()->UnloadNativeLibraries();
  }
}

REQUIRES(!art::Locks::thread_list_lock_)
REQUIRES_SHARED(art::Locks::mutator_lock_)
static size_t number_of_mutators() {
  size_t num = 0;
  {
    art::Runtime* runtime = art::Runtime::Current();
    art::MutexLock mu(art::Thread::Current(), *art::Locks::thread_list_lock_);

    runtime->GetThreadList()->ForEach([&num](art::Thread* thread) {
      UNUSED(thread);
      num++;
    });
  }

  return num;
}

static bool is_mutator(void* tls) {
  DCHECK(tls != nullptr);
  art::Thread* self = reinterpret_cast<art::Thread*>(tls);
  return self->GetMmtkMutator() != nullptr;
}

static MmtkMutator get_mmtk_mutator(void* tls) {
  DCHECK(tls != nullptr);
  DCHECK(is_mutator(tls));
  art::Thread* self = reinterpret_cast<art::Thread*>(tls);
  return self->GetMmtkMutator();
}

REQUIRES(!art::Locks::thread_list_lock_)
REQUIRES_SHARED(art::Locks::mutator_lock_)
static void for_all_mutators(MutatorClosure closure) {
  {
    art::Runtime* runtime = art::Runtime::Current();
    art::MutexLock mu(art::Thread::Current(), *art::Locks::thread_list_lock_);

    runtime->GetThreadList()->ForEach([&closure](art::Thread* thread) {
      closure.invoke(thread->GetMmtkMutator());
    });
  }
}

REQUIRES_SHARED(art::Locks::mutator_lock_)
static void scan_all_roots(SlotsClosure closure) {
  art::Runtime* runtime = art::Runtime::Current();
  art::gc::third_party_heap::MmtkRootVisitor visitor(closure);
  runtime->VisitRoots(&visitor, art::kVisitRootFlagAllRoots);
  // {
  //   art::WriterMutexLock mu(art::Thread::Current(), *art::Locks::classlinker_classes_lock_);
  //   runtime->GetClassLinker()->VisitClassLoaders(&visitor);
  // }
  // {
  //   art::WriterMutexLock mu(art::Thread::Current(), *art::Locks::dex_lock_);
  //   runtime->GetClassLinker()->VisitDexCaches(&visitor);
  // }
}

REQUIRES_SHARED(art::Locks::mutator_lock_)
static void process_references(void* tls,
                               TraceObjectClosure closure,
                               RefProcessingPhase phase,
                               bool clear_soft_references) {
  DCHECK(tls != nullptr);
  art::Thread* self = reinterpret_cast<art::Thread*>(tls);
  art::Runtime* runtime = art::Runtime::Current();

  // Process references
  art::gc::third_party_heap::MmtkIsMarkedVisitor is_marked_visitor;
  art::gc::third_party_heap::MmtkMarkObjectVisitor mark_object_visitor(closure);
  art::gc::ReferenceProcessor* rp = runtime->GetHeap()->GetReferenceProcessor();
  rp->ProcessReferencesTPH(self,
                           phase,
                           &mark_object_visitor,
                           &is_marked_visitor,
                           clear_soft_references);

  // Sweep system weaks after clearing the soft, weak, and phantom references
  if (phase == Phase3) {
    runtime->SweepSystemWeaks(&is_marked_visitor);
    runtime->GetThreadList()->SweepInterpreterCaches(&is_marked_visitor);
    runtime->BroadcastForNewSystemWeaks();
    runtime->GetClassLinker()->CleanupClassLoaders();
  }
}

REQUIRES_SHARED(art::Locks::mutator_lock_)
static void sweep_system_weaks() {
  art::Runtime* runtime = art::Runtime::Current();
  art::gc::third_party_heap::MmtkIsMarkedVisitor is_marked_visitor;

  runtime->SweepSystemWeaks(&is_marked_visitor);
  runtime->GetThreadList()->SweepInterpreterCaches(&is_marked_visitor);
  runtime->BroadcastForNewSystemWeaks();
  runtime->GetClassLinker()->CleanupClassLoaders();
}

static void set_has_zygote_space_in_art(bool has_zygote_space) {
  art::gc::third_party_heap::ThirdPartyHeap* tp_heap =
    art::Runtime::Current()->GetHeap()->GetThirdPartyHeap();
  tp_heap->SetHasZygoteSpace(has_zygote_space);
}

REQUIRES_SHARED(art::Locks::mutator_lock_)
static void throw_out_of_memory(void* tls, MmtkAllocationError err_kind) {
  DCHECK(tls != nullptr);
  switch (err_kind) {
    case MmapOOM:
      LOG(FATAL) << "Failed to allocate pages for space";
      break;
    case HeapOOM:
      art::Thread* self = reinterpret_cast<art::Thread*>(tls);
      art::Runtime* runtime = art::Runtime::Current();
      // If we're in a stack overflow, do not create a new exception. It would require running the
      // constructor, which will of course still be in a stack overflow.
      if (self->IsHandlingStackOverflow()) {
        self->SetException(
            runtime->GetPreAllocatedOutOfMemoryErrorWhenHandlingStackOverflow());
        return;
      }
      // Allow plugins to intercept out of memory errors.
      runtime->OutOfMemoryErrorHook();

      std::ostringstream oss;
      oss << "Failed to allocate object. Java heap space exhausted.";
      self->ThrowOutOfMemoryError(oss.str().c_str());
      break;
  }
}

ArtUpcalls art_upcalls = {
  size_of,
  scan_object,
  block_for_gc,
  spawn_gc_thread,
  suspend_mutators,
  resume_mutators,
  number_of_mutators,
  is_mutator,
  get_mmtk_mutator,
  for_all_mutators,
  scan_all_roots,
  process_references,
  sweep_system_weaks,
  set_has_zygote_space_in_art,
  throw_out_of_memory,
};
