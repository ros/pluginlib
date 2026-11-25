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

from ros2plugin.verb.list import ListVerb


def run_list(capsys, *, packages=False, package=None):
    ListVerb().main(args=argparse.Namespace(packages=packages, package=package))
    return capsys.readouterr().out.splitlines()


def test_add_arguments():
    parser = argparse.ArgumentParser()
    ListVerb().add_arguments(parser, 'ros2 plugin list')
    args = parser.parse_args(['--package', 'pkg_a'])
    assert not args.packages
    assert args.package == 'pkg_a'
    assert parser.parse_args(['--packages']).packages


def test_list_packages(plugin_prefix, capsys):
    assert run_list(capsys, packages=True) == [
        'pkg_a', 'pkg_b', 'pkg_c', 'pkg_d', 'pkg_ghost']


def test_list_single_package(plugin_prefix, capsys):
    # Plugins without a name are listed by their type.
    assert run_list(capsys, package='pkg_a') == [
        'pkg_a:',
        '\tpkg_a/foo [pkg_a::Foo] (base: base_pkg::Base)',
        '\tpkg_a::Bar [pkg_a::Bar] (base: base_pkg::Base)',
    ]


def test_list_unknown_package(plugin_prefix, capsys):
    assert run_list(capsys, package='unknown_pkg') == []


def test_list_all(plugin_prefix, capsys):
    output = run_list(capsys)

    assert output[:3] == [
        'pkg_a:',
        '\tpkg_a/foo [pkg_a::Foo] (base: base_pkg::Base)',
        '\tpkg_a::Bar [pkg_a::Bar] (base: base_pkg::Base)',
    ]
    # Problems with one package are reported and the others are still listed.
    assert output[3:5] == ['pkg_b:', 'XML manifest missing.xml not found.']
    assert output[5] == 'pkg_c:'
    assert output[6] == (
        'Failed to parse plugin XML file: ' +
        str(plugin_prefix / 'share' / 'pkg_c' / 'broken.xml'))
    assert output[7].startswith('XML error: ')
    assert output[8:] == [
        'pkg_d:',
        '\tpkg_d/baz [pkg_d::Baz] (base: other_pkg::Other)',
        'pkg_ghost:',
        'Package pkg_ghost not found.',
    ]
