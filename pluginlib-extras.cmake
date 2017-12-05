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

if(CMAKE_COMPILER_IS_GNUCXX)
  # this is needed to use the experimental/filesystem on Linux, but cannot be passed with
  # ament_export_libraries() because it is not absolute and cannot be found with find_library
  list(APPEND pluginlib_LIBRARIES stdc++fs)
endif()

# tinyxml2 is being exported as a dependency of pluginlib, but
# with newer versions of tinyxml2 (>=5.0.1), the tinyxml2_LIBRARIES variable is empty
# and instead the tinyxml2 exported CMake target exists.
# However, ament_export_dependencies will not propagate this, so here we will
# check for the tinyxml2 exported target and if found we will add it to the
# pluginlib_LIBRARIES.
if(TARGET tinyxml2)
  list(APPEND pluginlib_LIBRARIES tinyxml2)
endif()
