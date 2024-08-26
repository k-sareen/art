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

#ifndef MMTK_ART_THIRD_PARTY_HEAP_VISIT_OBJECTS_INL_H_
#define MMTK_ART_THIRD_PARTY_HEAP_VISIT_OBJECTS_INL_H_

#include <sstream>

#include "android-base/unique_fd.h"
#include "gc/third_party_heap.h"
#include "mirror/class.h"
#include "mmtk.h"
#include "read_barrier_option.h"

namespace art {
namespace gc {
namespace third_party_heap {

[[maybe_unused]] static inline std::string DumpRAMAroundAddress(uintptr_t addr, uintptr_t bytes) {
  uintptr_t* dump_start = reinterpret_cast<uintptr_t*>(addr - bytes);
  uintptr_t* dump_end = reinterpret_cast<uintptr_t*>(addr + bytes);
  std::ostringstream oss;
  oss << " adjacent_ram=";

  {
    // Check if the RAM is accessible.
    android::base::unique_fd read_fd, write_fd;
    if (!android::base::Pipe(&read_fd, &write_fd)) {
      LOG(WARNING) << "Could not create pipe, RAM being dumped may be unaccessible";
    } else {
      size_t count = 2 * bytes;
      if (write(write_fd.get(), dump_start, count) != static_cast<ssize_t>(count)) {
        oss << "unaccessible";
        dump_start = dump_end;
      }
    }
  }

  for (const uintptr_t* p = dump_start; p < dump_end; ++p) {
    if (p == reinterpret_cast<uintptr_t*>(addr)) {
      // Marker of where the address is.
      oss << "| ";
    }
    oss << std::hex << std::setfill('0') << std::setw(sizeof(uintptr_t) * 2) << *p << " ";
  }

  return oss.str();
}

REQUIRES_SHARED(Locks::mutator_lock_)
static inline bool IsValidHeapObjectAddress(ThirdPartyHeap* tp_heap, const void* addr) {
  return IsAligned<kObjectAlignment>(addr) && tp_heap->IsObjectInHeapSpace(addr);
}

template <ReadBarrierOption kReadBarrierOption>
static bool IsValidClassUnchecked(ThirdPartyHeap* tp_heap, mirror::Class* klass) REQUIRES_SHARED(Locks::mutator_lock_) {
  mirror::Class* k1 = klass->GetClass<kVerifyNone, kReadBarrierOption>();
  if (!IsValidHeapObjectAddress(tp_heap, k1)) {
    return false;
  }
  // `k1` should be class class, take the class again to verify.
  // Note that this check may not be valid for the no image space
  // since the class class might move around from moving GC.
  mirror::Class* k2 = k1->GetClass<kVerifyNone, kReadBarrierOption>();
  if (!IsValidHeapObjectAddress(tp_heap, k2)) {
    return false;
  }
  return k1 == k2;
}

template <ReadBarrierOption kReadBarrierOption>
static bool IsValidClass(ThirdPartyHeap* tp_heap, mirror::Class* klass) REQUIRES_SHARED(Locks::mutator_lock_) {
  if (!IsValidHeapObjectAddress(tp_heap, klass)) {
    return false;
  }
  return IsValidClassUnchecked<kReadBarrierOption>(tp_heap, klass);
}

template <typename Visitor>
inline void ThirdPartyHeap::VisitObjects(Visitor&& visitor) {
  // TODO(kunals): Investigate performance of visiting objects like this
  // Visit objects by doing a linear scan through allocated regions
  RustAllocatedRegionBuffer regions = mmtk_iterate_allocated_regions();
  for (size_t i = 0; i < regions.len; i++) {
    AllocatedRegion region = regions.buf[i];
    uint8_t* cursor = reinterpret_cast<uint8_t*>(region.start);
    uint8_t* region_end = reinterpret_cast<uint8_t*>(((size_t)region.start) + region.size);
    while (cursor < region_end) {
      if (IsAligned<kObjectAlignment>(cursor) &&
          mmtk_is_object_marked(cursor) &&
          !mmtk_is_object_forwarded(cursor)) {
        mirror::Object* object = reinterpret_cast<mirror::Object*>(cursor);
        mirror::Class* klass = object->GetClass();
        if (klass == nullptr || !IsValidClass<kWithoutReadBarrier>(this, klass)) {
          cursor += kObjectAlignment;
          continue;
        }
        visitor(object);
        cursor += RoundUp(object->SizeOf(), kObjectAlignment);
      } else {
        cursor += kObjectAlignment;
      }
    }
  }

  // Visit large objects
  RustObjectReferenceBuffer large_objects = mmtk_enumerate_large_objects();
  for (size_t i = 0; i < large_objects.len; i++) {
    mirror::Object* object = reinterpret_cast<mirror::Object*>(large_objects.buf[i]);
    visitor(object);
  }

  mmtk_release_rust_allocated_region_buffer(regions.buf, regions.len, regions.capacity);
  mmtk_release_rust_object_reference_buffer(large_objects.buf, large_objects.len, large_objects.capacity);

  {
    // Visit objects inside image space
    ReaderMutexLock mu(Thread::Current(), *Locks::heap_bitmap_lock_);
    Runtime::Current()->GetHeap()->GetLiveBitmap()->Visit<Visitor>(visitor);
  }
}

}  // namespace third_party_heap
}  // namespace gc
}  // namespace art

#endif  // MMTK_ART_THIRD_PARTY_HEAP_VISIT_OBJECTS_INL_H_
