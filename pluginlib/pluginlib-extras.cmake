# Copyright 2017 Open Source Robotics Foundation, Inc.
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

# copied from pluginlib/pluginlib-extras.cmake

find_package(ament_cmake_core QUIET REQUIRED)
ament_register_extension("ament_package" "pluginlib"
  "pluginlib_package_hook.cmake")

include("${pluginlib_DIR}/pluginlib_export_plugin_description_file.cmake")
include("${pluginlib_DIR}/pluginlib_enable_plugin_testing.cmake")

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_CXX_FLAGS MATCHES "-stdlib=libc\\+\\+")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.0)
    # Before LLVM 7.0, filesystem is part of experimental
    set(FILESYSTEM_LIB c++experimental)
  elseif(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
    # Before LLVM 9.0 you have to manually link the fs library
    set(FILESYSTEM_LIB c++fs)
  else()
    # Starting at LLVM 9.0 filesystem is built in
    set(FILESYSTEM_LIB)
  endif()
else()
  set(FILESYSTEM_LIB stdc++fs)
endif()

if(UNIX AND NOT APPLE)
  # this is needed to use the experimental/filesystem on Linux, but cannot be passed with
  # ament_export_libraries() because it is not absolute and cannot be found with find_library
  list(APPEND pluginlib_LIBRARIES ${FILESYSTEM_LIB})
endif()
