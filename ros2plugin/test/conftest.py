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


PLUGINS_XML = """\
<library path="pkg_a_plugins">
  <class name="pkg_a/foo" type="pkg_a::Foo" base_class_type="base_pkg::Base">
    <description>Foo plugin</description>
  </class>
  <class type="pkg_a::Bar" base_class_type="base_pkg::Base"/>
</library>
"""

OTHER_PLUGINS_XML = """\
<library path="pkg_d_plugins">
  <class name="pkg_d/baz" type="pkg_d::Baz" base_class_type="other_pkg::Other"/>
</library>
"""


def _write(path, content=''):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)


@pytest.fixture
def plugin_prefix(tmp_path, monkeypatch):
    """
    Create an install prefix with plugin resources and make it the only ament prefix.

    - pkg_a exports two plugins for base_pkg (one of them without a name).
    - pkg_b registers a plugin XML that does not exist.
    - pkg_c registers a plugin XML that is not valid XML.
    - pkg_d exports a plugin for other_pkg, with an extra ';' field in the resource.
    - pkg_ghost registers plugins but is not a known package.
    """
    prefix = tmp_path / 'install'
    index = prefix / 'share' / 'ament_index' / 'resource_index'
    for package in ('pkg_a', 'pkg_b', 'pkg_c', 'pkg_d'):
        _write(index / 'packages' / package)

    base = index / 'base_pkg__pluginlib__plugin'
    _write(base / 'pkg_a', 'share/pkg_a/plugins.xml\n')
    _write(base / 'pkg_b', 'share/pkg_b/missing.xml\n')
    _write(base / 'pkg_c', 'share/pkg_c/broken.xml\n')
    _write(base / 'pkg_ghost', 'share/pkg_ghost/plugins.xml\n')
    _write(
        index / 'other_pkg__pluginlib__plugin' / 'pkg_d',
        'share/pkg_d/plugins.xml;ignored\n')

    _write(prefix / 'share' / 'pkg_a' / 'plugins.xml', PLUGINS_XML)
    _write(prefix / 'share' / 'pkg_c' / 'broken.xml', '<library')
    _write(prefix / 'share' / 'pkg_d' / 'plugins.xml', OTHER_PLUGINS_XML)

    monkeypatch.setenv('AMENT_PREFIX_PATH', str(prefix))
    return prefix
