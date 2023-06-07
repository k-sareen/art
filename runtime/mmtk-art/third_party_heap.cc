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
#include "mmtk-art/mmtk_upcalls.h"
#include "mmtk.h"
#include "runtime_globals.h"
#include "thread.h"

namespace art {
namespace gc {
namespace third_party_heap {

ThirdPartyHeap::ThirdPartyHeap(size_t initial_size, size_t capacity) {
  mmtk_set_heap_size(initial_size, capacity);
  mmtk_init(&art_upcalls);
}

ThirdPartyHeap::~ThirdPartyHeap() {}

void ThirdPartyHeap::EnableCollection(Thread* tls) {
  mmtk_initialize_collection(tls);
}

bool ThirdPartyHeap::IsObjectInHeapSpace(const void* addr) const {
  return mmtk_is_object_in_heap_space(addr);
}

mirror::Object* ThirdPartyHeap::TryToAllocate(Thread* self,
                                              size_t alloc_size,
                                              size_t* bytes_allocated,
                                              size_t* usable_size,
                                              size_t* bytes_tl_bulk_allocated) {
#if ART_USE_MMTK
  // Have to round up allocation size in order to make sure that object starting
  // addresses are aligned
  alloc_size = RoundUp(alloc_size, kObjectAlignment);
  uint8_t* ret = (uint8_t *) mmtk_alloc(
    self->GetMmtkMutator(),
    alloc_size,
    kObjectAlignment,
    0 /* offset */,
    0 /* default allocation semantics */
  );
  mmtk_post_alloc(self->GetMmtkMutator(), ret, alloc_size, 0 /* default allocation semantics */);
  if (LIKELY(ret != nullptr)) {
    *bytes_allocated = alloc_size;
    *usable_size = alloc_size;
    *bytes_tl_bulk_allocated = alloc_size;
  }
  return reinterpret_cast<mirror::Object*>(ret);
#else
  UNUSED(self);
  UNUSED(alloc_size);
  UNUSED(bytes_allocated);
  UNUSED(usable_size);
  UNUSED(bytes_tl_bulk_allocated);
  return nullptr;
#endif  // ART_USE_MMTK
}

collector::GcType ThirdPartyHeap::CollectGarbage(GcCause cause,
                                                 bool clear_soft_references,
                                                 uint32_t requested_gc_num) {
  UNUSED(clear_soft_references);
  UNUSED(cause);
  UNUSED(requested_gc_num);
  LOG(WARNING) << "Called Heap::CollectGarbage() for MMTk!";
  return collector::kGcTypeNone;
}

}  // namespace third_party_heap
}  // namespace gc {}
}  // namespace art
