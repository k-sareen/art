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

#include <dirent.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "android-base/file.h"
#include "android-base/unique_fd.h"
#include "base/file_utils.h"
#include "base/os.h"
#include "base/stl_util.h"
#include "base/unix_file/fd_file.h"
#include "gtest/gtest.h"

namespace art {
namespace {

void ClearDirectory(const char* dirpath, bool recursive = true) {
  ASSERT_TRUE(dirpath != nullptr);
  DIR* dir = opendir(dirpath);
  ASSERT_TRUE(dir != nullptr);
  dirent* e;
  struct stat s;
  while ((e = readdir(dir)) != nullptr) {
    if ((strcmp(e->d_name, ".") == 0) || (strcmp(e->d_name, "..") == 0)) {
      continue;
    }
    std::string filename(dirpath);
    filename.push_back('/');
    filename.append(e->d_name);
    int stat_result = lstat(filename.c_str(), &s);
    ASSERT_EQ(0, stat_result) << "unable to stat " << filename;
    if (S_ISDIR(s.st_mode)) {
      if (recursive) {
        ClearDirectory(filename.c_str());
        int rmdir_result = rmdir(filename.c_str());
        ASSERT_EQ(0, rmdir_result) << filename;
      }
    } else {
      int unlink_result = unlink(filename.c_str());
      ASSERT_EQ(0, unlink_result) << filename;
    }
  }
  closedir(dir);
}

class Dex2oatScratchDirs {
 public:
  void SetUp(const std::string& android_data) {
    // Create a scratch directory to work from.

    // Get the realpath of the android data. The oat dir should always point to real location
    // when generating oat files in dalvik-cache. This avoids complicating the unit tests
    // when matching the expected paths.
    UniqueCPtr<const char[]> android_data_real(realpath(android_data.c_str(), nullptr));
    ASSERT_TRUE(android_data_real != nullptr)
        << "Could not get the realpath of the android data" << android_data << strerror(errno);

    scratch_dir_.assign(android_data_real.get());
    scratch_dir_ += "/Dex2oatEnvironmentTest";
    ASSERT_EQ(0, mkdir(scratch_dir_.c_str(), 0700));

    // Create a subdirectory in scratch for odex files.
    odex_oat_dir_ = scratch_dir_ + "/oat";
    ASSERT_EQ(0, mkdir(odex_oat_dir_.c_str(), 0700));

    odex_dir_ = odex_oat_dir_ + "/" + std::string(GetInstructionSetString(kRuntimeISA));
    ASSERT_EQ(0, mkdir(odex_dir_.c_str(), 0700));
  }

  void TearDown() {
    ClearDirectory(odex_dir_.c_str());
    ASSERT_EQ(0, rmdir(odex_dir_.c_str()));

    ClearDirectory(odex_oat_dir_.c_str());
    ASSERT_EQ(0, rmdir(odex_oat_dir_.c_str()));

    ClearDirectory(scratch_dir_.c_str());
    ASSERT_EQ(0, rmdir(scratch_dir_.c_str()));
  }

  // Scratch directory, for dex and odex files (oat files will go in the
  // dalvik cache).
  const std::string& GetScratchDir() const { return scratch_dir_; }

  // Odex directory is the subdirectory in the scratch directory where odex
  // files should be located.
  const std::string& GetOdexDir() const { return odex_dir_; }

 private:
  std::string scratch_dir_;
  std::string odex_oat_dir_;
  std::string odex_dir_;
};

class Dex2oatCtsTest : public Dex2oatScratchDirs, public testing::Test {
 public:
  void SetUpAndroidDataDir(std::string& android_data) {
    android_data = "/data/local/tmp";
    android_data += "/art-data-XXXXXX";
    if (mkdtemp(&android_data[0]) == nullptr) {
      PLOG(FATAL) << "mkdtemp(\"" << &android_data[0] << "\") failed";
    }
    setenv("ANDROID_DATA", android_data.c_str(), 1);
  }

  void TearDownAndroidDataDir(const std::string& android_data,
                              bool fail_on_error) {
    if (fail_on_error) {
      ASSERT_EQ(rmdir(android_data.c_str()), 0);
    } else {
      rmdir(android_data.c_str());
    }
  }

  std::string GetTestDexFileName(const char* name) const {
    CHECK(name != nullptr);
    // The needed jar files for gtest are located next to the gtest binary itself.
    std::string executable_dir = android::base::GetExecutableDirectory();
    for (auto ext : {".jar", ".dex"}) {
      std::string path = executable_dir + "/art-gtest-jars-" + name + ext;
      if (OS::FileExists(path.c_str())) {
        return path;
      }
    }
    LOG(FATAL) << "Test file " << name << " not found";
    UNREACHABLE();
  }

  void SetUp() override {
    SetUpAndroidDataDir(android_data_);
    Dex2oatScratchDirs::SetUp(android_data_);
  }

  void TearDown() override {
    Dex2oatScratchDirs::TearDown();
    TearDownAndroidDataDir(android_data_, true);
  }

  struct ForkAndExecResult {
    enum Stage {
      kLink,
      kFork,
      kWaitpid,
      kFinished,
    };
    Stage stage;
    int status_code;

    bool StandardSuccess() {
      return stage == kFinished && WIFEXITED(status_code) && WEXITSTATUS(status_code) == 0;
    }
  };
  using OutputHandlerFn = std::function<void(char*, size_t)>;
  using PostForkFn = std::function<bool()>;

  ForkAndExecResult ForkAndExec(
      const std::vector<std::string>& argv,
      const PostForkFn& post_fork,
      const OutputHandlerFn& handler) {
    ForkAndExecResult result;
    result.status_code = 0;
    result.stage = ForkAndExecResult::kLink;

    std::vector<const char*> c_args;
    c_args.reserve(argv.size() + 1);
    for (const std::string& str : argv) {
      c_args.push_back(str.c_str());
    }
    c_args.push_back(nullptr);

    android::base::unique_fd link[2];
    {
      int link_fd[2];

      if (pipe(link_fd) == -1) {
        return result;
      }
      link[0].reset(link_fd[0]);
      link[1].reset(link_fd[1]);
    }

    result.stage = ForkAndExecResult::kFork;

    pid_t pid = fork();
    if (pid == -1) {
      return result;
    }

    if (pid == 0) {
      if (!post_fork()) {
        LOG(ERROR) << "Failed post-fork function";
        exit(1);
        UNREACHABLE();
      }

      // Redirect stdout and stderr.
      dup2(link[1].get(), STDOUT_FILENO);
      dup2(link[1].get(), STDERR_FILENO);

      link[0].reset();
      link[1].reset();

      execv(c_args[0], const_cast<char* const*>(c_args.data()));
      exit(1);
      UNREACHABLE();
    }

    result.stage = ForkAndExecResult::kWaitpid;
    link[1].reset();

    char buffer[128] = { 0 };
    ssize_t bytes_read = 0;
    while (TEMP_FAILURE_RETRY(bytes_read = read(link[0].get(), buffer, 128)) > 0) {
      handler(buffer, bytes_read);
    }
    handler(buffer, 0u);  // End with a virtual write of zero length to simplify clients.

    link[0].reset();

    if (waitpid(pid, &result.status_code, 0) == -1) {
      return result;
    }

    result.stage = ForkAndExecResult::kFinished;
    return result;
  }

  ForkAndExecResult ForkAndExec(
      const std::vector<std::string>& argv, const PostForkFn& post_fork, std::string* output) {
    auto string_collect_fn = [output](char* buf, size_t len) {
      *output += std::string(buf, len);
    };
    return ForkAndExec(argv, post_fork, string_collect_fn);
  }

 protected:
  std::string android_data_;

  // Stripped down counterpart to Dex2oatEnvironmentTest::Dex2Oat that only adds
  // enough arguments for our purposes.
  int Dex2Oat(const std::vector<std::string>& dex2oat_args,
              std::string* output,
              std::string* error_msg) {
    // This command line should work regardless of bitness, ISA, etc.
    std::vector<std::string> argv = {std::string(kAndroidArtApexDefaultPath) + "/bin/dex2oat"};
    argv.insert(argv.end(), dex2oat_args.begin(), dex2oat_args.end());

    // We must set --android-root.
    const char* android_root = getenv("ANDROID_ROOT");
    CHECK(android_root != nullptr);
    argv.push_back("--android-root=" + std::string(android_root));

    // We need dex2oat to actually log things.
    auto post_fork_fn = []() { return setenv("ANDROID_LOG_TAGS", "*:d", 1) == 0; };
    ForkAndExecResult res = ForkAndExec(argv, post_fork_fn, output);
    if (res.stage != ForkAndExecResult::kFinished) {
      *error_msg = strerror(errno);
      ::testing::AssertionFailure() << "Failed to finish dex2oat invocation: " << *error_msg;
    }

    if (!res.StandardSuccess()) {
      // We cannot use ASSERT_TRUE since the method returns an int and not void.
      ::testing::AssertionFailure() << "dex2oat fork/exec failed: " << *error_msg;
    }

    return res.status_code;
  }
};

}  // namespace

// Run dex2oat with --enable-palette-compilation-hooks to force calls to
// PaletteNotify{Start,End}Dex2oatCompilation.
TEST_F(Dex2oatCtsTest, CompilationHooks) {
  const std::string dex_location = GetTestDexFileName("Main");
  const std::string oat_location = GetScratchDir() + "/base.oat";
  const std::string vdex_location = GetScratchDir() + "/base.vdex";

  std::vector<std::string> args;
  args.emplace_back("--dex-file=" + dex_location);

  std::unique_ptr<File> oat_file(OS::CreateEmptyFile(oat_location.c_str()));
  ASSERT_NE(oat_file, nullptr) << oat_location;
  args.emplace_back("--oat-fd=" + std::to_string(oat_file->Fd()));
  args.emplace_back("--oat-location=" + oat_location);

  std::unique_ptr<File> vdex_file(OS::CreateEmptyFile(vdex_location.c_str()));
  ASSERT_NE(vdex_file, nullptr) << vdex_location;
  args.emplace_back("--output-vdex-fd=" + std::to_string(vdex_file->Fd()));

  args.emplace_back("--force-palette-compilation-hooks");

  std::string output = "";
  std::string error_msg;
  int res = Dex2Oat(args, &output, &error_msg);
  EXPECT_EQ(res, 0) << error_msg;
  EXPECT_EQ(oat_file->FlushCloseOrErase(), 0);
  EXPECT_EQ(vdex_file->FlushCloseOrErase(), 0);
}

}  // namespace art
