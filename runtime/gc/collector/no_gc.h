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

#ifndef ART_RUNTIME_GC_COLLECTOR_NO_GC_H_
#define ART_RUNTIME_GC_COLLECTOR_NO_GC_H_

#include <memory>

#include "garbage_collector.h"
#include "gc_root.h"

namespace art {

namespace mirror {
class Class;
class Object;
class Reference;
}  // namespace mirror

class Thread;

namespace gc {

class Heap;

namespace collector {

class NoGC : public GarbageCollector {
 public:
  explicit NoGC(Heap* heap, const std::string& name_prefix = "");

  ~NoGC() {}

  GcType GetGcType() const override {
    return kGcTypeNoGC;
  }

  CollectorType GetCollectorType() const override {
    return kCollectorTypeNoGC;
  }

  void RunPhases() override NO_THREAD_SAFETY_ANALYSIS;
  // void RunPhases() override REQUIRES_SHARED(Locks::mutator_lock_);

  mirror::Object* MarkObject(mirror::Object* root) override NO_THREAD_SAFETY_ANALYSIS;

  void MarkHeapReference(mirror::HeapReference<mirror::Object>* obj,
                         bool do_atomic_update)
    override NO_THREAD_SAFETY_ANALYSIS;

  void DelayReferenceReferent(ObjPtr<mirror::Class> klass,
                              ObjPtr<mirror::Reference> reference)
    override NO_THREAD_SAFETY_ANALYSIS;

  void VisitRoots(mirror::Object*** roots, size_t count, const RootInfo& info)
    override NO_THREAD_SAFETY_ANALYSIS;

  void VisitRoots(mirror::CompressedReference<mirror::Object>** roots,
                  size_t count,
                  const RootInfo& info)
    override NO_THREAD_SAFETY_ANALYSIS;

 protected:
  mirror::Object* IsMarked(mirror::Object* obj)
    override NO_THREAD_SAFETY_ANALYSIS;

  bool IsNullOrMarkedHeapReference(mirror::HeapReference<mirror::Object>* obj,
                                   bool do_atomic_update)
    override NO_THREAD_SAFETY_ANALYSIS;

  void ProcessMarkStack() override NO_THREAD_SAFETY_ANALYSIS;

  // Revoke all the thread-local buffers.
  void RevokeAllThreadLocalBuffers() override;

 private:
  DISALLOW_IMPLICIT_CONSTRUCTORS(NoGC);
};

}  // namespace collector
}  // namespace gc
}  // namespace art

#endif 	// ART_RUNTIME_GC_COLLECTOR_NO_GC_H_
