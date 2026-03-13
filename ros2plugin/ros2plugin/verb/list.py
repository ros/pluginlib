# Copyright 2025 Canonical Ltd.
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

from collections import namedtuple

import os
import xml.etree.ElementTree as ET

from ament_index_python import get_package_prefix
from ament_index_python import PackageNotFoundError
from ros2cli.node.strategy import add_arguments

from ros2plugin.api import get_registered_plugin_resources
from ros2plugin.verb import VerbExtension


PluginInfo = namedtuple('Plugin', ('name', 'type', 'base'))


class ListVerb(VerbExtension):
    """Output a list of plugins."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            '--packages', action='store_true',
            help='List the packages that register plugins')
        parser.add_argument(
            '--package', type=str,
            help='Name of the package to list plugins from')

    @staticmethod
    def _print_package_resources(plugin_resources):
        """Print package names and their associated plugin resource paths."""
        for package_name, package_plugin_resources in sorted(plugin_resources):
            print(f'{package_name}:')
            if any(package_plugin_resources):
                print(*[f'\t{r}' for r in package_plugin_resources], sep='\n')

    def main(self, *, args):
        plugin_resources = get_registered_plugin_resources()

        if args.package:
            plugin_resources = [
                (pkg, res) for pkg, res in plugin_resources
                if pkg == args.package
            ]

        if args.packages or args.package:
            self._print_package_resources(plugin_resources)
            return

        for package_name, package_plugin_resources in sorted(plugin_resources):
            plugins = []
            print(f'{package_name}:')
            for package_plugin_resource in package_plugin_resources:
                try:
                    package_prefix = get_package_prefix(package_name)
                except PackageNotFoundError:
                    print(f'Package {package_name} not found.')
                    continue

                plugin_xml = os.path.join(package_prefix, package_plugin_resource)
                if not os.path.isfile(plugin_xml):
                    print(f'XML manifest {os.path.basename(plugin_xml)} not found.')
                    continue

                try:
                    tree = ET.parse(plugin_xml)
                except ET.ParseError as e:
                    print(
                        f'Failed to parse plugin XML file: {plugin_xml}\n'
                        f'XML error: {e}'
                    )
                    continue

                for e in tree.iter():
                    if e.tag == 'class':
                        try:
                            plugin_name = e.attrib['name']
                        except KeyError:
                            plugin_name = e.attrib['type']
                        plugins.append(PluginInfo(
                            plugin_name, e.attrib['type'], e.attrib['base_class_type'])
                        )

            if any(plugins):
                print(*[f'\t{p}' for p in plugins], sep='\n')
