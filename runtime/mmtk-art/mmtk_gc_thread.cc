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

#include "base/logging.h"
#include "base/utils.h"
#include "mmtk.h"
#include "mmtk_gc_thread.h"
#include "runtime.h"
#include "thread-inl.h"
#include "thread.h"

namespace art {

#if defined(__BIONIC__)
static constexpr bool kUseCustomThreadPoolStack = false;
#else
static constexpr bool kUseCustomThreadPoolStack = true;
#endif

MmtkGcThread::MmtkGcThread(const std::string& name)
    : name_(name) {
  std::string error_msg;
  size_t stack_size = kDefaultStackSize;
  // On Bionic, we know pthreads will give us a big-enough stack with
  // a guard page, so don't do anything special on Bionic libc.
  if (kUseCustomThreadPoolStack) {
    // Add an inaccessible page to catch stack overflow.
    stack_size += kPageSize;
    stack_ = MemMap::MapAnonymous(name.c_str(),
                                  stack_size,
                                  PROT_READ | PROT_WRITE,
                                  /*low_4gb=*/ false,
                                  &error_msg);
    CHECK(stack_.IsValid()) << error_msg;
    CHECK_ALIGNED(stack_.Begin(), kPageSize);
    CheckedCall(mprotect,
                "mprotect bottom page of MmtkGcThread stack",
                stack_.Begin(),
                kPageSize,
                PROT_NONE);
  }
  const char* reason = "new MmtkGcThread";
  pthread_attr_t attr;
  CHECK_PTHREAD_CALL(pthread_attr_init, (&attr), reason);
  if (kUseCustomThreadPoolStack) {
    CHECK_PTHREAD_CALL(pthread_attr_setstack, (&attr, stack_.Begin(), stack_.Size()), reason);
  } else {
    CHECK_PTHREAD_CALL(pthread_attr_setstacksize, (&attr, stack_size), reason);
  }
  CHECK_PTHREAD_CALL(pthread_create, (&pthread_, &attr, &Callback, this), reason);
  CHECK_PTHREAD_CALL(pthread_attr_destroy, (&attr), reason);
}

void* MmtkGcThread::Callback(void* arg) {
  MmtkGcThread* worker = reinterpret_cast<MmtkGcThread*>(arg);
  Runtime* runtime = Runtime::Current();
  // Don't run callbacks for MmtkGcThreads. See comment in
  // ThreadPoolWorker::Callback() for more information
  CHECK(runtime->AttachCurrentThread(
      worker->name_.c_str(),
      /* as_daemon= */ true,
      /* thread_group= */ nullptr,
      /* create_peer= */ false,
      /* should_run_callbacks= */ false));
  worker->thread_ = Thread::Current();
  // Mark MMTk GC threads as runtime-threads.
  worker->thread_->SetIsRuntimeThread(true);
  // Do work until its time to shut down.
  worker->Run();
  runtime->DetachCurrentThread(/* should_run_callbacks= */ false);
  return nullptr;
}

void MmtkGcThread::Run() {}

MmtkControllerThread::MmtkControllerThread(const std::string& name,
                                               void* context)
                                        : MmtkGcThread(name),
                                          context_(context) {}

void MmtkControllerThread::Run() {
  mmtk_start_gc_controller_thread((void*) thread_, (void*) context_);
}

MmtkWorkerThread::MmtkWorkerThread(const std::string& name,
                                       void* context)
                                : MmtkGcThread(name),
                                  context_(context) {}

void MmtkWorkerThread::Run() {
  mmtk_start_gc_worker_thread((void*) thread_, (void*) context_);
}

}  // namespace art
