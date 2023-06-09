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
  # Set threshold to 1000 to match the iterations done in the test.
  # Pass --verbose-methods to only generate the CFG of these methods.
  # The test is for JIT, but we run in "optimizing" (AOT) mode, so that the Checker
  # stanzas in test/638-checker-inline-cache-intrinsic/src/Main.java will be checked.
  # Also pass a large JIT code cache size to avoid getting the inline caches GCed.
  ctx.default_run(
      args,
      jit=True,
      runtime_option=["-Xjitinitialsize:32M", "-Xjitthreshold:1000"],
      Xcompiler_option=[
          "--verbose-methods=inlineMonomorphic,inlinePolymorphic,knownReceiverType,stringEquals"
      ])
