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

#ifndef ART_RUNTIME_KUTRACE_H_
#define ART_RUNTIME_KUTRACE_H_

#include <string.h>
#include <stdint.h>
#include <unistd.h>

// XXX: CLU is dependant on the architecture
#define CLU(x)                x##LLU
#define KUTRACE_CMD_INSERT1   7
#define KUTRACE_CMD_VERSION   11
#define KUTRACE_MARKA         0x20A
#define KUTRACE_MARKB         0x20B
#define KUTRACE_MARKC         0x20C
#define KUTRACE_MARKD         0x20D
#define __NR_kutrace_control  1023

namespace art {
namespace kutrace {

inline static uint64_t DoControl(uint64_t command, uint64_t arg) {
  return syscall(__NR_kutrace_control, command, arg);
}

inline static void DoMark(uint64_t n, uint64_t arg) {
  //         T             N                       ARG
  uint64_t temp = (CLU(0) << 44) | (n << 32) | (arg & CLU(0x00000000FFFFFFFF));
  DoControl(KUTRACE_CMD_INSERT1, temp);
}


inline void mark_a(uint64_t label_base_40) {
  DoMark(KUTRACE_MARKA, label_base_40);
}

inline void mark_b(uint64_t label_base_40) {
  DoMark(KUTRACE_MARKB, label_base_40);
}

inline void mark_c(uint64_t label_base_40) {
  DoMark(KUTRACE_MARKC, label_base_40);
}

inline void mark_d(uint64_t n) {
  DoMark(KUTRACE_MARKD, n);
}

}  // namespace kutrace
}  // namespace art

#endif  // ART_RUNTIME_KUTRACE_H_
