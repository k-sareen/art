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

#ifndef MMTK_ART_MMTK_MARK_OBJECT_VISITOR_H
#define MMTK_ART_MMTK_MARK_OBJECT_VISITOR_H

#include "gc/third_party_heap.h"
#include "mmtk.h"
#include "object_callbacks.h"

namespace art {

namespace mirror {
class Object;
}

namespace gc {

namespace third_party_heap {

class MmtkMarkObjectVisitor : public MarkObjectVisitor {
 public:
  MmtkMarkObjectVisitor(TraceObjectClosure closure) : closure_(closure) {}

  mirror::Object* MarkObject(mirror::Object* obj) NO_THREAD_SAFETY_ANALYSIS {
    void* ret = closure_.invoke(obj);
    return reinterpret_cast<mirror::Object*>(ret);
  }

  void MarkHeapReference(mirror::HeapReference<mirror::Object>* obj_ptr,
                         bool do_atomic_update ATTRIBUTE_UNUSED) NO_THREAD_SAFETY_ANALYSIS {
    void* ret = closure_.invoke(obj_ptr->AsMirrorPtr());
    mirror::Object* forwarded_address = reinterpret_cast<mirror::Object*>(ret);
    obj_ptr->Assign(forwarded_address);
  }

 private:
  TraceObjectClosure closure_;
};

}  // namespace third_party_heap
}  // namespace gc
}  // namespace art

#endif  // MMTK_ART_MMTK_MARK_OBJECT_VISITOR_H
