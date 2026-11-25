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

import argparse

import pytest

from ros2plugin.api import get_package_names_with_plugin_resource_types
from ros2plugin.verb.list import ListVerb


def _write(path, content=''):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)


@pytest.fixture
def prefix(tmp_path, monkeypatch):
    """Create an empty install prefix and make it the only ament prefix."""
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(tmp_path))
    return tmp_path


def _add_plugins(prefix, package, base_package, xml_name, xml_content):
    index = prefix / 'share' / 'ament_index' / 'resource_index'
    _write(index / 'packages' / package)
    _write(
        index / f'{base_package}__pluginlib__plugin' / package,
        f'share/{package}/{xml_name}\n')
    _write(prefix / 'share' / package / xml_name, xml_content)


def _list(capsys, *, packages=False):
    ListVerb().main(args=argparse.Namespace(packages=packages, package=None))
    return capsys.readouterr().out.splitlines()


def test_package_with_several_base_classes_is_listed_once(prefix, capsys):
    _add_plugins(
        prefix, 'pkg', 'base_a', 'a.xml',
        '<library path="pkg_plugins">'
        '<class name="pkg/a" type="pkg::A" base_class_type="base_a::A"/>'
        '</library>')
    _add_plugins(
        prefix, 'pkg', 'base_b', 'b.xml',
        '<library path="pkg_plugins">'
        '<class name="pkg/b" type="pkg::B" base_class_type="base_b::B"/>'
        '</library>')

    assert get_package_names_with_plugin_resource_types() == ['pkg']
    assert _list(capsys, packages=True) == ['pkg']

    output = _list(capsys)
    assert output[0] == 'pkg:'
    assert sorted(output[1:]) == [
        '\tpkg/a [pkg::A] (base: base_a::A)',
        '\tpkg/b [pkg::B] (base: base_b::B)',
    ]


def test_class_with_missing_attributes_is_skipped(prefix, capsys):
    _add_plugins(
        prefix, 'pkg', 'base', 'plugins.xml',
        '<library path="pkg_plugins">'
        '<class name="pkg/no_type" base_class_type="base::Base"/>'
        '<class name="pkg/no_base" type="pkg::NoBase"/>'
        '<class name="pkg/foo" type="pkg::Foo" base_class_type="base::Base"/>'
        '</library>')

    xml = prefix / 'share' / 'pkg' / 'plugins.xml'
    skipped = f"Skipping class without 'type' or 'base_class_type' in {xml}."
    assert _list(capsys) == [
        'pkg:',
        skipped,
        skipped,
        '\tpkg/foo [pkg::Foo] (base: base::Base)',
    ]
