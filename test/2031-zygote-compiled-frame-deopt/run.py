#!/bin/bash
#
# Copyright 2020 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


def run(ctx, args):
  # The -Xopaque-jni-ids makes sure we can do structural redefinition. The --add-libdir-argument tells
  # default-run to pass the directory where the jvmti-agent is so we can load it later. The others
  # set the process to zygote mode and setup the jit cache size. We use a larger than normal jit-size
  # to avoid having to deal with jit-gc, a complication that's not relevant to this test.
  ctx.default_run(
      args,
      runtime_option=[
          "-Xopaque-jni-ids:true", "-Xzygote", "-Xjitinitialsize:64M"
      ],
      add_libdir_argument=True)
