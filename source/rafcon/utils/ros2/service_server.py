# Copyright (C) 2016-2026 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Johannes Ernst
# Ryo Sakagami
#
# NOTE: To use the scripts, ROS2 needs to be installed (tested with 'Humble')

'''
This files provides the implementation of a ServiceServerHandle for ROS2 service servers in RAFCON.
'''


class ServiceServerHandle():
    """
    A class for the ROS2 service server handle for usage in RAFCON state machines.

    Variables:
    rafcon: The RAFCON handle to access, e.g. the logger
    node: The ROS node that is used for instantiation
    srv_def: Stores the service definition
    srv_name: Stores the service server name
    srv_callback: Stores the function handle to the service callback
    callback_group: Optional callback group for multi threaded executtion

    Functions:
    destroy_service: Destroys the service saved in the handle
    replace_chars: Replace non-ascii chars with 'char(#)' for printing
    """

    def __init__(self, rafcon, node, srv_def, srv_name, srv_callback, callback_group=None):
        self.rafcon = rafcon
        self.node = node
        self.srv_def = srv_def
        self.srv_name = srv_name
        self.srv_callback = srv_callback
        self.callback_group = callback_group

        # Check if node is already initilized
        if self.node is None:
            raise RuntimeError("ROS2 node is not available in gvm. "\
                               "Have you called init_ros2_node?")

        # Create service server
        self.ser = self.node.create_service(self.srv_def,
                                            self.srv_name,
                                            self.srv_callback,
                                            callback_group=self.callback_group)

    def destroy_service(self):
        self.node.destroy_service(self.ser)

    def replace_chars(self, message):
        # Replace non-ascii chars with 'char(#)' for printing
        message_str = ''.join([i if 32 <= ord(i) < 127 else 'char(' + str(ord(i)) + ')' \
                               for i in str(message)])
        return message_str
