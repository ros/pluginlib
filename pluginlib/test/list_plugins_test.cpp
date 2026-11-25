// Copyright 2026 Open Source Robotics Foundation, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

#ifdef _WIN32
#define popen _popen
#define pclose _pclose
#else
#include <sys/wait.h>
#endif

namespace
{

struct Result
{
  int exit_code;
  std::string output;  // stdout and stderr
};

// Runs list_plugins with 'args' and returns its exit code and output.
Result runListPlugins(const std::string & args)
{
  std::string command = "\"" LIST_PLUGINS_EXECUTABLE "\" " + args + " 2>&1";
#ifdef _WIN32
  // cmd.exe strips the outer quotes of the command line.
  command = "\"" + command + "\"";
#endif
  Result result{-1, ""};
  FILE * pipe = popen(command.c_str(), "r");
  if (pipe == nullptr) {
    ADD_FAILURE() << "Failed to run: " << command;
    return result;
  }
  std::array<char, 256> buffer;
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
    result.output += buffer.data();
  }
  const int status = pclose(pipe);
#ifdef _WIN32
  result.exit_code = status;
#else
  result.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
#endif
  return result;
}

std::vector<std::string> sortedLines(const std::string & text)
{
  std::vector<std::string> lines;
  std::istringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (!line.empty()) {
      lines.push_back(line);
    }
  }
  std::sort(lines.begin(), lines.end());
  return lines;
}

TEST(ListPluginsTest, wrongNumberOfArguments) {
  const Result result = runListPlugins("test_pluginlib");
  EXPECT_EQ(1, result.exit_code) << result.output;
  EXPECT_NE(std::string::npos, result.output.find("Usage:")) << result.output;
}

TEST(ListPluginsTest, listsDeclaredClasses) {
  // 'test_pluginlib/foo_with_ctor' has a different base class, so it is not listed.
  const Result result = runListPlugins("test_pluginlib test_base::Fubar");
  EXPECT_EQ(0, result.exit_code) << result.output;
  EXPECT_EQ(
    (std::vector<std::string>{
      "test_pluginlib/bar", "test_pluginlib/foo", "test_pluginlib/none"}),
    sortedLines(result.output));
}

TEST(ListPluginsTest, unknownBaseClass) {
  const Result result = runListPlugins("test_pluginlib test_base::Unknown");
  EXPECT_EQ(0, result.exit_code) << result.output;
  EXPECT_TRUE(sortedLines(result.output).empty()) << result.output;
}

TEST(ListPluginsTest, unknownPackage) {
  const Result result = runListPlugins("pluginlib_unknown_package test_base::Fubar");
  EXPECT_EQ(1, result.exit_code) << result.output;
  EXPECT_NE(std::string::npos, result.output.find("Error:")) << result.output;
  EXPECT_NE(std::string::npos, result.output.find("pluginlib_unknown_package")) << result.output;
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
