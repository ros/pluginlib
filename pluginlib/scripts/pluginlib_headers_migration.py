#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holders nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Run this script to update legacy pluginlib include statements
# to utilize the new pluginlib headers.

from __future__ import print_function

import subprocess

cmd = "find . -type f ! -name '*.svn-base' -a ! -name '*.hg' -a ! -name '*.git' -a " \
      "\( -name '*.c*' -o -name '*.h*' \)"

header_mappings = {
    'pluginlib/class_desc.h': 'pluginlib/class_desc.hpp',
    'pluginlib/class_list_macros.h': 'pluginlib/class_list_macros.hpp',
    'pluginlib/class_loader_base.h': 'pluginlib/class_loader_base.hpp',
    'pluginlib/class_loader.h': 'pluginlib/class_loader.hpp',
    'pluginlib/class_loader_imp.h': 'pluginlib/class_loader_imp.hpp',
    'pluginlib/pluginlib_exceptions.h': 'pluginlib/exceptions.hpp',
}
include_tokens = {
    '"': '"',
    '<': '>',
}
include_prefix = '#include '
sed_cmd_prefix = ' -exec sed -i \' s'
sed_cmd_suffix = ' {} \; '
sed_separator = '@'
full_cmd = cmd
for old_header, new_header in header_mappings.iteritems():
    for leading_token, ending_token in include_tokens.iteritems():
        full_cmd += \
            sed_cmd_prefix + sed_separator + \
            include_prefix + leading_token + old_header + ending_token + sed_separator + \
            include_prefix + leading_token + new_header + ending_token + sed_separator + \
            "g'" + sed_cmd_suffix
    print("Looking for '%s' to replace with '%s'" % (old_header, new_header))

print('Running %s' % full_cmd)
ret_code = subprocess.call(full_cmd, shell=True)
if ret_code == 0:
    print('success')
else:
    print('failure')
