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

#ifndef MMTK_ART_MMTK_IS_MARKED_VISITOR_H
#define MMTK_ART_MMTK_IS_MARKED_VISITOR_H

#include "gc/third_party_heap.h"
#include "mmtk.h"
#include "object_callbacks.h"

namespace art {

namespace mirror {
class Object;
}

namespace gc {

namespace third_party_heap {

class MmtkIsMarkedVisitor : public IsMarkedVisitor {
  mirror::Object* IsMarked(mirror::Object* obj) {
    if (mmtk_is_object_marked(reinterpret_cast<void*>(obj))) {
      mirror::Object* forwarded_object = reinterpret_cast<mirror::Object*>(
        mmtk_get_forwarded_object(reinterpret_cast<void*>(obj))
      );

      if (forwarded_object) {
        return forwarded_object;
      } else {
        return obj;
      }
    } else {
      return nullptr;
    }
  }
};

}  // namespace third_party_heap
}  // namespace gc
}  // namespace art

#endif  // MMTK_ART_MMTK_IS_MARKED_VISITOR_H
