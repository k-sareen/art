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

#include "gc/third_party_heap.h"
#include "mmtk.h"

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

template <typename Visitor>
inline void ThirdPartyHeap::VisitObjects(Visitor&& visitor) {
  // TODO(kunals): Investigate performance of visiting objects like this
  void* heap_start = mmtk_get_heap_start();
  void* heap_end = mmtk_get_heap_end();

  // Linear scan through all live objects and call the visitor for each one
  uint8_t* cursor = reinterpret_cast<uint8_t*>(heap_start);
  while (cursor < heap_end) {
    // Skip forwarded objects. We'll find the actual object later in the linear
    // scan. MMTk will return the correct value for `mmtk_is_object_marked` even
    // for freshly moved objects
    if (IsAligned<kObjectAlignment>(cursor) &&
        mmtk_is_object_marked(cursor) &&
        !mmtk_is_object_forwarded(cursor)) {
      mirror::Object* object = reinterpret_cast<mirror::Object*>(cursor);
      visitor(object);
      cursor += RoundUp(object->SizeOf(), kObjectAlignment);
    } else {
      cursor += kObjectAlignment;
    }
  }

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
