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

#include "profiling_info_builder.h"

#include "art_method-inl.h"
#include "code_generator.h"
#include "driver/compiler_options.h"
#include "dex/code_item_accessors-inl.h"
#include "jit/profiling_info.h"
#include "optimizing_compiler_stats.h"
#include "scoped_thread_state_change-inl.h"

namespace art HIDDEN {

void ProfilingInfoBuilder::Run() {
  DCHECK_EQ(GetGraph()->GetProfilingInfo(), nullptr);
  // Order does not matter.
  for (HBasicBlock* block : GetGraph()->GetReversePostOrder()) {
    // No need to visit the phis.
    for (HInstructionIteratorHandleChanges inst_it(block->GetInstructions()); !inst_it.Done();
         inst_it.Advance()) {
      inst_it.Current()->Accept(this);
    }
  }

  ScopedObjectAccess soa(Thread::Current());
  GetGraph()->SetProfilingInfo(
      ProfilingInfo::Create(soa.Self(), GetGraph()->GetArtMethod(), inline_caches_));
}

void ProfilingInfoBuilder::HandleInvoke(HInvoke* invoke) {
  DCHECK(!invoke->GetEnvironment()->IsFromInlinedInvoke());
  if (IsInlineCacheUseful(invoke, codegen_)) {
    inline_caches_.push_back(invoke->GetDexPc());
  }
}

void ProfilingInfoBuilder::VisitInvokeInterface(HInvokeInterface* invoke) {
  HandleInvoke(invoke);
}

void ProfilingInfoBuilder::VisitInvokeVirtual(HInvokeVirtual* invoke) {
  HandleInvoke(invoke);
}

bool ProfilingInfoBuilder::IsInlineCacheUseful(HInvoke* invoke, CodeGenerator* codegen) {
  DCHECK(invoke->IsInvokeVirtual() || invoke->IsInvokeInterface());
  if (codegen->IsImplementedIntrinsic(invoke)) {
    return false;
  }
  if (!invoke->GetBlock()->GetGraph()->IsCompilingBaseline()) {
    return false;
  }
  if (Runtime::Current()->IsAotCompiler()) {
    return false;
  }
  if (invoke->InputAt(0)->GetReferenceTypeInfo().IsExact()) {
    return false;
  }
  if (invoke->GetResolvedMethod() != nullptr) {
    ScopedObjectAccess soa(Thread::Current());
    if (invoke->GetResolvedMethod()->IsFinal() ||
        invoke->GetResolvedMethod()->GetDeclaringClass()->IsFinal()) {
      return false;
    }
  }
  return true;
}

InlineCache* ProfilingInfoBuilder::GetInlineCache(ProfilingInfo* info, HInvoke* instruction) {
  DCHECK(!instruction->GetEnvironment()->IsFromInlinedInvoke());
  ScopedObjectAccess soa(Thread::Current());
  return info->GetInlineCache(instruction->GetDexPc());
}

}  // namespace art
