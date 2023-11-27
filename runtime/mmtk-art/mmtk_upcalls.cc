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
#include "gc/third_party_heap.h"
#include "mmtk_gc_thread.h"
#include "mmtk_is_marked_visitor.h"
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
  // XXX(kunals): Temporarily mask lowest order bits to avoid reading MMTk GC
  // state and causing segfaults
  // art::mirror::Object* obj = reinterpret_cast<art::mirror::Object*>((size_t)object & ~0b11);
  art::mirror::Object* obj = reinterpret_cast<art::mirror::Object*>(object);
  return obj->SizeOf();
}

static void scan_object(void* object, ScanObjectClosure closure) {
  art::gc::third_party_heap::MmtkScanObjectVisitor visitor(closure);
  // XXX(kunals): Temporarily mask lowest order bits to avoid reading MMTk GC
  // state and causing segfaults
  // art::mirror::Object* obj = reinterpret_cast<art::mirror::Object*>((size_t)object & ~0b11);
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
  art::Thread* self = reinterpret_cast<art::Thread*>(tls);
  art::gc::third_party_heap::ThirdPartyHeap* tp_heap =
    art::Runtime::Current()->GetHeap()->GetThirdPartyHeap();
  // ThirdPartyHeap::BlockThreadForCollection calls Heap::WaitForGcToComplete internally
  PERFORM_SUSPENDING_OPERATION(self, tp_heap->BlockThreadForCollection(art::gc::kGcCauseForAlloc, self));
#undef PERFORM_SUSPENDING_OPERATION
}

static void spawn_gc_thread(void* tls, GcThreadKind kind, void* ctx) {
  UNUSED(tls);
  switch (kind) {
    case MmtkGcController: {
      new art::MmtkControllerThread("MMTk Controller Context Thread", ctx);
      break;
    }
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

static void stop_all_mutators() {
  art::gc::third_party_heap::ThirdPartyHeap* tp_heap =
    art::Runtime::Current()->GetHeap()->GetThirdPartyHeap();

  art::MmtkVmCompanionThread* companion =
    reinterpret_cast<art::MmtkVmCompanionThread*>(tp_heap->GetCompanionThread());
  companion->Request(art::StwState::Suspended);
}

REQUIRES(!art::Locks::thread_list_lock_)
static void resume_mutators(void* tls) {
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

  art::gc::third_party_heap::ThirdPartyHeap* tp_heap =
    runtime->GetHeap()->GetThirdPartyHeap();

  art::MmtkVmCompanionThread* companion =
    reinterpret_cast<art::MmtkVmCompanionThread*>(tp_heap->GetCompanionThread());
  companion->Request(art::StwState::Resumed);
  tp_heap->FinishGC(self);
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
  art::Thread* self = reinterpret_cast<art::Thread*>(tls);
  return self->GetMmtkMutator() != nullptr;
}

static MmtkMutator get_mmtk_mutator(void* tls) {
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
static void scan_all_roots(NodesClosure closure) {
  art::Runtime* runtime = art::Runtime::Current();
  art::gc::third_party_heap::MmtkRootVisitor visitor(closure);
  runtime->VisitRoots(&visitor);
}

static void sweep_system_weaks(void* tls ATTRIBUTE_UNUSED) {
  art::Runtime* runtime = art::Runtime::Current();
  art::gc::third_party_heap::MmtkIsMarkedVisitor visitor;
  runtime->SweepSystemWeaks(&visitor);
  runtime->GetThreadList()->SweepInterpreterCaches(&visitor);
}

ArtUpcalls art_upcalls = {
  size_of,
  scan_object,
  block_for_gc,
  spawn_gc_thread,
  stop_all_mutators,
  resume_mutators,
  number_of_mutators,
  is_mutator,
  get_mmtk_mutator,
  for_all_mutators,
  scan_all_roots,
  sweep_system_weaks,
};
