# Copyright 2026 Open Source Robotics Foundation, Inc.
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

import os
import shutil
import subprocess

import pytest


def run_ros2_plugin(*args):
    ros2 = shutil.which('ros2')
    if ros2 is None:
        pytest.skip("'ros2' executable not found")
    return subprocess.run(
        [ros2, 'plugin', *args], env=os.environ.copy(),
        capture_output=True, text=True, timeout=60)


def test_cli_list_packages(plugin_prefix):
    result = run_ros2_plugin('list', '--packages')
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        'pkg_a', 'pkg_b', 'pkg_c', 'pkg_d', 'pkg_ghost']


def test_cli_without_verb(plugin_prefix):
    result = run_ros2_plugin()
    assert result.returncode == 0, result.stderr
    assert 'list' in result.stdout
