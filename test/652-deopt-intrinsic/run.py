#!/bin/bash
#
# Copyright (C) 2017 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


def run(ctx, args):
  # Ensure this test is not subject to code collection.
  # We also need at least a few invocations of the method Main.$noinline$doCall
  # to ensure the inline cache sees the two types being passed to the method. Pass
  # a large number in case there's some weights on some invocation kinds (eg
  # compiler to interpreter transitions).
  ctx.default_run(
      args, runtime_option=["-Xjitinitialsize:32M", "-Xjitthreshold:1000"])
