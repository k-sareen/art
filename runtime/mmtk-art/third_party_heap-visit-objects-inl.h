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

#include "gc/third_party_heap.h"
#include "mmtk.h"

namespace art {
namespace gc {
namespace third_party_heap {

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
}

}  // namespace third_party_heap
}  // namespace gc
}  // namespace art

#endif  // MMTK_ART_THIRD_PARTY_HEAP_VISIT_OBJECTS_INL_H_
