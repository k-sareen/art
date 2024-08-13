/*
 * Copyright (C) 2024 The Android Open Source Project
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

#ifndef MMTK_ART_MMTK_UTILS_H
#define MMTK_ART_MMTK_UTILS_H

#include "mirror/object.h"
#include "mirror/object_reference.h"
#include "object_callbacks.h"

#if ART_USE_MMTK

bool MmtkIsNullOrMarkedHeapReference(art::mirror::HeapReference<art::mirror::Object>* object,
                                     art::IsMarkedVisitor* visitor) NO_THREAD_SAFETY_ANALYSIS;

#endif  // ART_USE_MMTK
#endif  // MMTK_ART_MMTK_UTILS_H
