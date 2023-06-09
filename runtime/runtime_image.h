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

#ifndef ART_RUNTIME_RUNTIME_IMAGE_H_
#define ART_RUNTIME_RUNTIME_IMAGE_H_

#include <string>

namespace art {

class RuntimeImage {
 public:
    // Writes an app image for the currently running process.
  static bool WriteImageToDisk(std::string* error_msg);

  // Gets the path where a runtime-generated app image is stored.
  static std::string GetRuntimeImagePath(const std::string& dex_location);
};

}  // namespace art

#endif  // ART_RUNTIME_RUNTIME_IMAGE_H_
