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

import pytest

from ros2plugin.api import get_package_names_with_plugin_resource_types
from ros2plugin.api import get_package_plugin_resource
from ros2plugin.api import get_registered_plugin_resource_list
from ros2plugin.api import get_registered_plugin_resources
from ros2plugin.api import is_plugin_resource_type


def test_is_plugin_resource_type():
    assert is_plugin_resource_type('base_pkg__pluginlib__plugin')
    assert not is_plugin_resource_type('packages')
    with pytest.raises(AssertionError):
        is_plugin_resource_type('')


def test_get_registered_plugin_resource_list(plugin_prefix):
    assert sorted(get_registered_plugin_resource_list()) == [
        'base_pkg__pluginlib__plugin', 'other_pkg__pluginlib__plugin']


def test_get_package_names_with_plugin_resource_types(plugin_prefix):
    assert sorted(get_package_names_with_plugin_resource_types()) == [
        'pkg_a', 'pkg_b', 'pkg_c', 'pkg_d', 'pkg_ghost']


def test_get_package_plugin_resource(plugin_prefix):
    assert get_package_plugin_resource(package_name='pkg_a') == ['share/pkg_a/plugins.xml']
    # Only the part before ';' is the path to the plugin XML.
    assert get_package_plugin_resource(package_name='pkg_d') == ['share/pkg_d/plugins.xml']
    assert get_package_plugin_resource(package_name='unknown_pkg') == []


def test_get_registered_plugin_resources(plugin_prefix):
    assert sorted(get_registered_plugin_resources()) == [
        ('pkg_a', ['share/pkg_a/plugins.xml']),
        ('pkg_b', ['share/pkg_b/missing.xml']),
        ('pkg_c', ['share/pkg_c/broken.xml']),
        ('pkg_d', ['share/pkg_d/plugins.xml']),
        ('pkg_ghost', ['share/pkg_ghost/plugins.xml']),
    ]


def test_no_plugins(tmp_path, monkeypatch):
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(tmp_path))
    assert get_registered_plugin_resource_list() == []
    assert get_registered_plugin_resources() == []
