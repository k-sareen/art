/*
 * Copyright (C) 2017 The Android Open Source Project
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

#ifndef ART_COMPILER_OPTIMIZING_CODE_SINKING_H_
#define ART_COMPILER_OPTIMIZING_CODE_SINKING_H_

#include "base/macros.h"
#include "nodes.h"
#include "optimization.h"

namespace art HIDDEN {

/**
 * Optimization pass to move instructions into uncommon branches,
 * when it is safe to do so.
 */
class CodeSinking : public HOptimization {
 public:
  CodeSinking(HGraph* graph,
              OptimizingCompilerStats* stats,
              const char* name = kCodeSinkingPassName)
      : HOptimization(graph, name, stats) {}

  bool Run() override;

  static constexpr const char* kCodeSinkingPassName = "code_sinking";

 private:
  // Tries to sink code to uncommon branches.
  void UncommonBranchSinking();
  // Tries to move code only used by `end_block` and all its post-dominated / dominated
  // blocks, to these blocks.
  void SinkCodeToUncommonBranch(HBasicBlock* end_block);

  // Coalesces the Return/ReturnVoid instructions into one, if we have two or more. We do this to
  // avoid generating the exit frame code several times.
  void ReturnSinking();

  DISALLOW_COPY_AND_ASSIGN(CodeSinking);
};

}  // namespace art

#endif  // ART_COMPILER_OPTIMIZING_CODE_SINKING_H_
