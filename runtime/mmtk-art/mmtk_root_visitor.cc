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
#include "mmtk_root_visitor.h"
#include "mmtk.h"

namespace art {
namespace gc {
namespace third_party_heap {

MmtkRootVisitor::MmtkRootVisitor(EdgesClosure closure) : closure_(closure), cursor_(0) {
  RustBuffer buf = closure_.invoke(NULL, 0, 0);
  buffer_ = buf.buf;
  capacity_ = buf.capacity;
}

MmtkRootVisitor::~MmtkRootVisitor() {
  if (cursor_ > 0) {
    FlushBuffer();
  }

  if (buffer_ != NULL) {
    mmtk_release_rust_buffer(buffer_, cursor_, capacity_);
  }
}

void MmtkRootVisitor::VisitRoots(mirror::Object*** roots,
                size_t count,
                const RootInfo& info ATTRIBUTE_UNUSED) {
  for (size_t i = 0; i < count; ++i) {
    auto* root = roots[i];
    // auto ref = StackReference<mirror::Object>::FromMirrorPtr(*root);

    buffer_[cursor_++] = (void*) root;
    if (cursor_ >= capacity_) {
      FlushBuffer();
    }

    // if (*root != ref.AsMirrorPtr()) {
    //   *root = ref.AsMirrorPtr();
    // }
  }
}

void MmtkRootVisitor::VisitRoots(mirror::CompressedReference<mirror::Object>** roots,
                size_t count,
                const RootInfo& info ATTRIBUTE_UNUSED) {
  for (size_t i = 0; i < count; ++i) {
    buffer_[cursor_++] = (void*) roots[i];
    if (cursor_ >= capacity_) {
      FlushBuffer();
    }
  }
}

void MmtkRootVisitor::FlushBuffer() {
  if (cursor_ > 0) {
    RustBuffer buf = closure_.invoke(buffer_, cursor_, capacity_);
    buffer_ = buf.buf;
    capacity_ = buf.capacity;
    cursor_ = 0;
  }
}

}  // namespace third_party_heap
}  // namespace gc
}  // namespace art
