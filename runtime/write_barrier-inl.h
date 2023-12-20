/*
 * Copyright (C) 2018 The Android Open Source Project
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

#ifndef ART_RUNTIME_WRITE_BARRIER_INL_H_
#define ART_RUNTIME_WRITE_BARRIER_INL_H_

#include "write_barrier.h"

#include "gc/accounting/card_table-inl.h"
#include "gc/heap.h"
#if ART_USE_MMTK
#include "mmtk.h"
#endif  // ART_USE_MMTK
#include "obj_ptr-inl.h"
#include "runtime.h"
#include "write_barrier_config.h"

#include <iostream>

namespace art {

template <WriteBarrier::NullCheck kNullCheck>
inline void WriteBarrier::ForFieldWrite(ObjPtr<mirror::Object> dst,
                                        MemberOffset offset,
                                        ObjPtr<mirror::Object> new_value) {
  if (gUseWriteBarrier) {
#if !ART_USE_MMTK
    UNUSED(offset);
    if (kNullCheck == kWithNullCheck && new_value == nullptr) {
      return;
    }
    DCHECK(new_value != nullptr);
    GetCardTable()->MarkCard(dst.Ptr());
#else
    // XXX(kunals): Check that the GC worker doesn't execute the write barrier
    MmtkMutator mmtk_mutator = Thread::Current()->GetMmtkMutator();
    if (mmtk_mutator != nullptr) {
      // XXX(kunals): In MMTk's terminology, the `dst` here is the `src`
      void* src = reinterpret_cast<void*>(dst.Ptr());
      uint8_t* slot = reinterpret_cast<uint8_t*>(src) + offset.Int32Value();
      void* target = reinterpret_cast<void*>(new_value.Ptr());
      // std::cout << "ObjectWrite mutator " << mmtk_mutator << " src " << src
      //   << " slot " << reinterpret_cast<void*>(slot)
      //   << " target " << target
      //   << "\n";
      mmtk_object_reference_write_post(mmtk_mutator, src, slot, target);
    } else {
      // XXX(kunals): In MMTk's terminology, the `dst` here is the `src`
      // void* src = reinterpret_cast<void*>(dst.Ptr());
      // uint8_t* slot = reinterpret_cast<uint8_t*>(src) + offset.Int32Value();
      // void* target = reinterpret_cast<void*>(new_value.Ptr());
      // std::cout << "ObjectWrite GC thread src " << src
      //   << " slot " << reinterpret_cast<void*>(slot)
      //   << " target " << target
      //   << "\n";
    }
#endif  // !ART_USE_MMTK
  }
}

inline void WriteBarrier::ForArrayWrite(void* src,
                                        void* dst,
                                        int start_offset,
                                        size_t length) {
  if (gUseWriteBarrier) {
#if !ART_USE_MMTK
    UNUSED(src);
    UNUSED(start_offset);
    UNUSED(length);
    GetCardTable()->MarkCard(dst);
#else
    UNUSED(start_offset);
    // XXX(kunals): Check that the GC worker doesn't execute the write barrier
    MmtkMutator mmtk_mutator = Thread::Current()->GetMmtkMutator();
    if (mmtk_mutator != nullptr) {
      // std::cout << "ArrayWrite mutator " << mmtk_mutator << " src " << src
      //   << " dst " << dst
      //   << " length " << length
      //   << "\n";
      mmtk_array_copy_post(mmtk_mutator, src, dst, length);
    } else {
      // std::cout << "ArrayWrite GC thread src " << src
      //   << " dst " << dst
      //   << " length " << length
      //   << "\n";
    }
#endif  // !ART_USE_MMTK
  }
}

inline void WriteBarrier::ForEveryFieldWrite(ObjPtr<mirror::Object> obj) {
  if (gUseWriteBarrier) {
#if !ART_USE_MMTK
    GetCardTable()->MarkCard(obj.Ptr());
#else
    // XXX(kunals): Check that the GC worker doesn't execute the write barrier
    MmtkMutator mmtk_mutator = Thread::Current()->GetMmtkMutator();
    if (mmtk_mutator != nullptr) {
      // TODO(kunals): What are the semantics of `ForEveryFieldWrite`?
      void* src = reinterpret_cast<void*>(obj.Ptr());
      // std::cout << "ObjectWrite mutator " << mmtk_mutator << " src " << src
      //   << "\n";
      // XXX(kunals): Since this is a post-barrier and I'm not sure what the
      // semantics are, we pass `nullptr`s as the slot and target
      mmtk_object_reference_write_post(mmtk_mutator, src, nullptr, nullptr);
    } else {
      // TODO(kunals): What are the semantics of `ForEveryFieldWrite`?
      // void* src = reinterpret_cast<void*>(obj.Ptr());
      // std::cout << "ObjectWrite GC thread src " << src
      //   << "\n";
    }
#endif  // !ART_USE_MMTK
  }
}

inline gc::accounting::CardTable* WriteBarrier::GetCardTable() {
  if (gUseWriteBarrier) {
#if !ART_USE_MMTK
    return Runtime::Current()->GetHeap()->GetCardTable();
#else
    return nullptr;
#endif  // !ART_USE_MMTK
  } else {
    return nullptr;
  }
}

}  // namespace art

#endif  // ART_RUNTIME_WRITE_BARRIER_INL_H_
