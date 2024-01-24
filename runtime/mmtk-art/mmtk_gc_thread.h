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

#ifndef MMTK_ART_MMTK_GC_THREAD_H_
#define MMTK_ART_MMTK_GC_THREAD_H_

#include <pthread.h>

#include <condition_variable>
#include <mutex>

#include "base/locks.h"
#include "base/mem_map.h"
#include "thread.h"

namespace art {

class MmtkWorkerThread {
 public:
  MmtkWorkerThread(const std::string& name, void* context);

  [[noreturn]]
  virtual ~MmtkWorkerThread() {
    LOG(FATAL) << "MmtkWorkerThread's destructor called";
    UNREACHABLE();
  }

 protected:
  static const size_t kDefaultStackSize = 1 * MB;

  // Callback to `pthread_create` that registers worker thread with the runtime
  // (i.e. creating an associated art::Thread object) and then executes the
  // `Run` function for the worker.
  static void* Callback(void* arg) REQUIRES(!Locks::mutator_lock_);

  // Create a new worker thread using `pthread_create`. Note there is a race
  // condition if one tries to create a new pthread inside the MmtkWorkerThread
  // constructor (See https://github.com/rr-debugger/rr/issues/3641). Hence,
  // each derived class needs to call this function inside their own
  // constructor.
  static void CreateWorkerThread(MmtkWorkerThread* worker);

  // Function that specialized classes override for worker type-specific work
  virtual void Run() {}

  const std::string name_;
  pthread_t pthread_;
  Thread* thread_;
  MemMap stack_;
  void* context_;
};

// MMTk GC thread. These threads are responsible for performing garbage collection.
class MmtkCollectorThread : MmtkWorkerThread {
 public:
  MmtkCollectorThread(const std::string& name, void* context);

 protected:
  void Run() override;
};

// MMTk GC Controller thread. There is only a single such thread. It is
// responsible for coordinating garbage collection.
class MmtkControllerThread : MmtkWorkerThread {
 public:
  MmtkControllerThread(const std::string& name, void* context);

 protected:
  void Run() override;
};

enum StwState {
  Resumed,
  Suspended,
};

// A separate companion thread whose sole job is to suspend/resume all mutator
// threads. We require a separate thread as ART expects the thread that suspends
// the mutators to be the thread that resumes the mutators (due to
// ThreadList::SuspendAll acquiring an exclusive lock on mutator_lock_).
class MmtkVmCompanionThread : MmtkWorkerThread {
 public:
  MmtkVmCompanionThread(const std::string& name);

  // Request the companion thread to transition to desired_state
  void Request(StwState desired_state);

 protected:
  void Run() override;

  // Suspend all mutator threads. Acquires exclusive lock on mutator_lock_
  void SuspendAll() EXCLUSIVE_LOCK_FUNCTION(Locks::mutator_lock_);

  // Resume all mutator threads. Releases exclusive lock on mutator_lock_
  void ResumeAll() UNLOCK_FUNCTION(Locks::mutator_lock_);

 private:
  // Current state of mutator threads
  StwState current_state_;
  // Desired state of mutator threads
  StwState desired_state_;

  // We use C++ stdlib mutex and condvar implementations as ART does not allow
  // waiting on a condvar while holding on to another lock (in this case we
  // would wait on `cond_` while holding on to the mutator_lock).

  // Lock associated with requesting suspend/resume all mutator threads
  std::mutex* lock_;
  // Associated condition variable. Used by GC worker(s) to communicate to the
  // companion thread
  std::condition_variable* cond_;
};

}  // namespace art

#endif  // MMTK_ART_MMTK_GC_THREAD_H_
