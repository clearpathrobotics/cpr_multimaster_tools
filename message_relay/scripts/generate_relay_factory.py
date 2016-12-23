#!/usr/bin/env python
# Software License Agreement (proprietary)
#
# @author    Paul Bovbel <pbovbel@clearpathrobotics.com>
# @copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, is not permitted without the
# express permission of Clearpath Robotics.

from Cheetah.Template import Template
import argparse
import pprint

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Generate topic relay factory source.')
    parser.add_argument('--msg-srv-names', metavar='msg_srv_names', nargs='*', help='Message/Service Names')
    parser.add_argument('--cpp-tmpl', metavar='*.cpp.tmpl', help='Input template', required=True)
    parser.add_argument('--cpp-out', metavar='*.cpp', help='Output source file', required=True)

    args = parser.parse_args()

    template_namespace = {}

    template_namespace['msg_srv_names'] = args.msg_srv_names
    template_namespace['pkg_names'] = set(s.rsplit('/', 1)[0] for s in args.msg_srv_names)

    # For debug, print definitions to console
    # pp = pprint.PrettyPrinter(indent=1)
    # pp.pprint(template_namespace)

    with open(args.cpp_tmpl, 'r') as f:
        source_template = Template(f.read(), searchList=[template_namespace])

    # print(template_namespace)
    with open(args.cpp_out, 'w') as f:
        f.write(str(source_template))
