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

ArtUpcalls art_upcalls = {
  size_of,
  block_for_gc,
  spawn_gc_thread,
};
