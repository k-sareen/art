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
#include "thread_list.h"

#include <condition_variable>
#include <mutex>

namespace art {

#if defined(__BIONIC__)
static constexpr bool kUseCustomThreadPoolStack = false;
#else
static constexpr bool kUseCustomThreadPoolStack = true;
#endif

MmtkWorkerThread::MmtkWorkerThread(const std::string& name, void* context)
        : name_(name), context_(context) {}

void MmtkWorkerThread::CreateWorkerThread(MmtkWorkerThread* worker) {
  std::string error_msg;
  size_t stack_size = kDefaultStackSize;
  // On Bionic, we know pthreads will give us a big-enough stack with
  // a guard page, so don't do anything special on Bionic libc.
  if (kUseCustomThreadPoolStack) {
    // Add an inaccessible page to catch stack overflow.
    stack_size += kPageSize;
    worker->stack_ = MemMap::MapAnonymous(worker->name_.c_str(),
                                  stack_size,
                                  PROT_READ | PROT_WRITE,
                                  /*low_4gb=*/ false,
                                  &error_msg);
    CHECK(worker->stack_.IsValid()) << error_msg;
    CHECK_ALIGNED(worker->stack_.Begin(), kPageSize);
    CheckedCall(mprotect,
                "mprotect bottom page of MmtkWorkerThread stack",
                worker->stack_.Begin(),
                kPageSize,
                PROT_NONE);
  }
  const char* reason = "MmtkWorkerThread";
  pthread_attr_t attr;
  CHECK_PTHREAD_CALL(pthread_attr_init, (&attr), reason);
  if (kUseCustomThreadPoolStack) {
    CHECK_PTHREAD_CALL(pthread_attr_setstack, (&attr, worker->stack_.Begin(), worker->stack_.Size()), reason);
  } else {
    CHECK_PTHREAD_CALL(pthread_attr_setstacksize, (&attr, stack_size), reason);
  }
  CHECK_PTHREAD_CALL(pthread_create, (&worker->pthread_, &attr, &Callback, worker), reason);
  CHECK_PTHREAD_CALL(pthread_attr_destroy, (&attr), reason);
}

void* MmtkWorkerThread::Callback(void* arg) {
  MmtkWorkerThread* worker = reinterpret_cast<MmtkWorkerThread*>(arg);
  Runtime* runtime = Runtime::Current();
  // Don't run callbacks for MmtkGcThreads. See comment in
  // ThreadPoolWorker::Callback() for more information
  CHECK(runtime->AttachCurrentThread(
      worker->name_.c_str(),
      /* as_daemon= */ true,
      /* thread_group= */ nullptr,
      /* create_peer= */ false,
      /* should_run_callbacks= */ false,
      /* add_to_thread_list= */ false));
  worker->thread_ = Thread::Current();
  // Mark MMTk GC threads as runtime-threads.
  worker->thread_->SetIsRuntimeThread(true);
  // Do work until its time to shut down.
  worker->Run();
  runtime->DetachCurrentThread(/* should_run_callbacks= */ false);
  return nullptr;
}

MmtkCollectorThread::MmtkCollectorThread(const std::string& name,
                                         void* context)
                                    : MmtkWorkerThread(name, context) {
  CreateWorkerThread(this);
}

void MmtkCollectorThread::Run() {
  LOG(INFO) << "Starting MmtkCollectorThread " << thread_ << " with context " << context_;
  mmtk_start_gc_worker_thread((void*) thread_, (void*) context_);
}

MmtkControllerThread::MmtkControllerThread(const std::string& name,
                                           void* context)
                                    : MmtkWorkerThread(name, context) {
  CreateWorkerThread(this);
}

void MmtkControllerThread::Run() {
  LOG(INFO) << "Starting MmtkControllerThread " << thread_ << " with context " << context_;
  mmtk_start_gc_controller_thread((void*) thread_, (void*) context_);
}

MmtkVmCompanionThread::MmtkVmCompanionThread(const std::string& name)
                                    : MmtkWorkerThread(name, nullptr),
                                      current_state_(StwState::Resumed),
                                      desired_state_(StwState::Resumed) {
  lock_ = new std::mutex();
  cond_ = new std::condition_variable();
  CreateWorkerThread(this);
}

void MmtkVmCompanionThread::Run() {
  LOG(INFO) << "Starting MmtkVmCompanionThread " << thread_;
  while (true) {
    VLOG(threads) << "Waiting for suspend request";
    {
      std::unique_lock mu(*lock_);
      cond_->wait(mu, [&]{ return desired_state_ == StwState::Suspended; });
    }

    // Suspend threads
    VLOG(threads) << "Received suspend request";
    SuspendAll();

    // All threads have been suspended at this point. Notify GC worker which
    // requested STW
    {
      std::unique_lock mu(*lock_);
      current_state_ = StwState::Suspended;
      cond_->notify_all();
    }

    VLOG(threads) << "Waiting for resume request";
    {
      std::unique_lock mu(*lock_);
      cond_->wait(mu, [&]{ return desired_state_ == StwState::Resumed; });
    }

    // Resume threads
    VLOG(threads) << "Received resume request";
    ResumeAll();

    // All threads have been resumed at this point. Notify GC worker which
    // requested resumption
    {
      std::unique_lock mu(*lock_);
      current_state_ = StwState::Resumed;
      cond_->notify_all();
    }
  }
}

void MmtkVmCompanionThread::Request(StwState desired_state) {
  std::unique_lock mu(*lock_);
  desired_state_ = desired_state;
  cond_->notify_all();
  cond_->wait(mu, [&]{ return current_state_ == desired_state; });
}

void MmtkVmCompanionThread::SuspendAll() {
  Runtime* runtime = Runtime::Current();
  runtime->GetThreadList()->SuspendAll(__FUNCTION__, /* long_suspend= */ false,
        /* is_self_registered= */ false);
}

void MmtkVmCompanionThread::ResumeAll() {
  Runtime::Current()->GetThreadList()->ResumeAll();
}

}  // namespace art