#!/bin/bash
#
# Copyright 2016 The Android Open Source Project
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
  ctx.default_run(args, jvmti=True)

  # The number of paused background threads (and therefore InterruptedExceptions)
  # can change so we will just delete their lines from the log.
  ctx.run(
      fr"""sed -i -E "/Object allocated of type 'java\.lang\.(InterruptedException|Long)'/d" '{args.stdout_file}'"""
  )
