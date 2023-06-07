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

#include "mmtk_upcalls.h"
#include "mirror/object-inl.h"

REQUIRES_SHARED(art::Locks::mutator_lock_)
static size_t size_of(void *object) {
  art::mirror::Object *obj = (art::mirror::Object *) object;
  return obj->SizeOf();
}

ArtUpcalls art_upcalls = {
  size_of,
};
