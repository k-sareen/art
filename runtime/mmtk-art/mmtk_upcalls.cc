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
#include "mmtk_upcalls.h"
#include "mirror/object-inl.h"
#include "thread.h"
#include "thread_list.h"

namespace art {
class Thread;
}  // namespace art

REQUIRES_SHARED(art::Locks::mutator_lock_)
static size_t size_of(void* object) {
  art::mirror::Object* obj = (art::mirror::Object*) object;
  return obj->SizeOf();
}

REQUIRES(art::Roles::uninterruptible_)
REQUIRES_SHARED(art::Locks::mutator_lock_)
static void block_for_gc(void* tls) {
#define PERFORM_SUSPENDING_OPERATION(op)                                                    \
  [&]() REQUIRES(art::Roles::uninterruptible_) REQUIRES_SHARED(art::Locks::mutator_lock_) { \
    art::ScopedAllowThreadSuspension ats;                                                   \
    return (op);                                                                            \
  }()
  art::Thread* self = reinterpret_cast<art::Thread*>(tls);
  art::gc::third_party_heap::ThirdPartyHeap* tp_heap =
    art::Runtime::Current()->GetHeap()->GetThirdPartyHeap();
  // tp_heap->BlockThreadForCollection calls Heap::WaitForGcToComplete internally
  PERFORM_SUSPENDING_OPERATION(tp_heap->BlockThreadForCollection(art::gc::kGcCauseForAlloc, self));
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
      new art::MmtkWorkerThread("MMTk Collector Thread", ctx);
      break;
    }
    default: {
      LOG(FATAL) << "Unexpected GC thread kind: " << kind;
      UNREACHABLE();
    }
  }
}

EXCLUSIVE_LOCK_FUNCTION(art::Locks::mutator_lock_)
static void stop_all_mutators() {
  art::Runtime* runtime = art::Runtime::Current();
  runtime->GetThreadList()->SuspendAll(__FUNCTION__);
}

UNLOCK_FUNCTION(art::Locks::mutator_lock_)
static void resume_mutators() {
  art::Runtime* runtime = art::Runtime::Current();
  runtime->GetThreadList()->ResumeAll();
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
#if ART_USE_MMTK
  art::Thread* self = reinterpret_cast<art::Thread*>(tls);
  return self->GetMmtkMutator() != nullptr;
#else
  return true;
#endif  // ART_USE_MMTK
}

static MmtkMutator get_mmtk_mutator(void* tls) {
#if ART_USE_MMTK
  art::Thread* self = reinterpret_cast<art::Thread*>(tls);
  return self->GetMmtkMutator();
#else
  return nullptr;
#endif  // ART_USE_MMTK
}

REQUIRES(!art::Locks::thread_list_lock_)
REQUIRES_SHARED(art::Locks::mutator_lock_)
static void for_all_mutators(MutatorClosure closure) {
#if ART_USE_MMTK
  {
    art::Runtime* runtime = art::Runtime::Current();
    art::MutexLock mu(art::Thread::Current(), *art::Locks::thread_list_lock_);

    runtime->GetThreadList()->ForEach([&closure](art::Thread* thread) {
      closure.invoke(thread->GetMmtkMutator());
    });
  }
#endif  // ART_USE_MMTK
}

ArtUpcalls art_upcalls = {
  size_of,
  block_for_gc,
  spawn_gc_thread,
  stop_all_mutators,
  resume_mutators,
  number_of_mutators,
  is_mutator,
  get_mmtk_mutator,
  for_all_mutators,
};
