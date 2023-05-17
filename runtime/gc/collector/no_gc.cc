/*
 * Copyright (C) 2022 The Android Open Source Project
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

#include "no_gc.h"
#include "gc/heap.h"

namespace art {
namespace gc {
namespace collector {

NoGC::NoGC(Heap* heap, const std::string& name_prefix)
    : GarbageCollector(heap, name_prefix + "no gc") {
}

void NoGC::RunPhases() {
  // Thread* self = Thread::Current();
  // LOG(ERROR) << "GC triggered in NoGC!";
  // self->ThrowOutOfMemoryError("GC triggered in NoGC. No space remaining for further allocation.");
  LOG(ERROR) << "GC triggered in NoGC!";
  return;
}

mirror::Object* NoGC::MarkObject(mirror::Object* root) {
  return root;
}

void NoGC::MarkHeapReference(mirror::HeapReference<mirror::Object>* obj ATTRIBUTE_UNUSED,
                               bool do_atomic_update ATTRIBUTE_UNUSED) {
  return;
}

void NoGC::DelayReferenceReferent(ObjPtr<mirror::Class> klass ATTRIBUTE_UNUSED,
                                    ObjPtr<mirror::Reference> reference ATTRIBUTE_UNUSED) {
  return;
}

void NoGC::VisitRoots(mirror::Object*** roots ATTRIBUTE_UNUSED,
                        size_t count ATTRIBUTE_UNUSED,
                        const RootInfo& info ATTRIBUTE_UNUSED) {
  return;
}

void NoGC::VisitRoots(mirror::CompressedReference<mirror::Object>** roots ATTRIBUTE_UNUSED,
                size_t count ATTRIBUTE_UNUSED,
                const RootInfo& info ATTRIBUTE_UNUSED) {
  return;
}

mirror::Object* NoGC::IsMarked(mirror::Object* obj) {
  // All objects are considered marked
  return obj;
}

bool NoGC::IsNullOrMarkedHeapReference(mirror::HeapReference<mirror::Object>* obj ATTRIBUTE_UNUSED,
                                           bool do_atomic_update ATTRIBUTE_UNUSED) {
  return true;
}

void NoGC::ProcessMarkStack() {
  return;
}

void NoGC::RevokeAllThreadLocalBuffers() {
  TimingLogger::ScopedTiming t(__FUNCTION__, GetTimings());
  GetHeap()->RevokeAllThreadLocalBuffers();
}

}  // namespace collector
}  // namespace gc
}  // namespace art
