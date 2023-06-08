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

#include "base/locks.h"
#include "base/mem_map.h"
#include "thread.h"

namespace art {

class MmtkWorkerThread {
  static const size_t kDefaultStackSize = 1 * MB;

 public:
  MmtkWorkerThread(const std::string& name, void* context);

  [[noreturn]]
  ~MmtkWorkerThread() {
    LOG(FATAL) << "MmtkWorkerThread's destructor called";
    UNREACHABLE();
  }

 protected:
  static void* Callback(void* arg) REQUIRES(!Locks::mutator_lock_);
  virtual void Run();

  const std::string name_;
  pthread_t pthread_;
  Thread* thread_;
  MemMap stack_;
  void* context_;
};

class MmtkControllerThread : MmtkWorkerThread {
 public:
  MmtkControllerThread(const std::string& name, void* context);

  ~MmtkControllerThread();

 protected:
  void Run() override;
};

}  // namespace art

#endif  // MMTK_ART_MMTK_GC_THREAD_H_
