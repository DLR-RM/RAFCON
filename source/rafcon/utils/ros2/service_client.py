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
This files provides the implementation of a ServiceClientHandle for ROS2 service clients in RAFCON.
'''

import threading


class ServiceClientHandle():
    """
    A class for the ROS2 service client handle for usage in RAFCON state machines.

    Variables:
    rafcon: The RAFCON handle to access, e.g. the logger
    gvm: The global variable manager to lookup node and dictionaries
    node: The ROS node that is used for instantiation
    srv_def: Stores the service definition
    srv_req: Stores the service request
    srv_name: Stores the service name
    srv_future: Stores a future object
    srv_resp: Stores the received response
    callback_group: Optional callback group for multi threaded executtion

    Functions:
    wait_for_srv: Wait for the service to be available
    set_srv_req: Set the service request manually
    set_srv_req_field_from_inputs: Sets the service request from given inputs
    set_srv_req_field: Sets the request field for a specific value
    call_srv: Calls the service
    wait_until_srv_completion: Waits for service call completion
    destroy_client: Destroys the client saved in the handle
    get_srv_resp: Get the full service response manually
    get_srv_resp_field: Gets the response fields from the service
    replace_chars: Replace non-ascii chars with 'char(#)' for printing
    """

    def __init__(self, rafcon, gvm, srv_def, srv_name, callback_group=None):
        self.rafcon = rafcon
        self.gvm = gvm
        self.srv_def = srv_def
        self.srv_req = srv_def.Request()
        self.srv_name = srv_name
        self.callback_group = callback_group
        self.srv_future = None
        self.srv_resp = None

        # Get node and check if it is already initilized
        self.node = self.gvm.get_variable('rafcon_ros_node', per_reference=True)
        if self.node is None:
            raise RuntimeError("ROS2 node is not available in gvm. "\
                               "Have you called init_ros2_node?")

        # Get service client from gvm dictionary or create anew
        dict_name = "ros_clients"
        access_key = self.gvm.lock_variable(dict_name, block=True)
        try:
            ros_clients = self.gvm.get_variable(dict_name,
                                                per_reference=True,
                                                access_key=access_key)
            if self.srv_name in ros_clients:
                self.cli, self.lock = ros_clients[self.srv_name]
                self.rafcon.logger.info(f"Using existing service client '{self.srv_name}'")
            else:
                self.cli = self.node.create_client(self.srv_def,
                                                   self.srv_name,
                                                   callback_group=self.callback_group)
                self.lock = threading.Lock()
                ros_clients[self.srv_name] = (self.cli, self.lock)
                self.gvm.set_variable(dict_name,
                                      ros_clients,
                                      per_reference=True,
                                      access_key=access_key)
                self.rafcon.logger.info(
                    f"Created new service client '{self.srv_name}' and saved to GVM"
                )
        finally:
            self.gvm.unlock_variable(dict_name, access_key)


    def wait_for_srv(self):
        # Wait for a short moment in ros level
        while not self.cli.wait_for_service(timeout_sec=0.1):
            # Wait relatively long in rafcon level to allow for user interaction to preempt
            if self.rafcon.wait_for_interruption(1.0):
                if self.rafcon.preempted:
                    return "preempted"
                if self.rafcon.paused:
                    self.rafcon.wait_for_unpause(0.1)
                    if self.rafcon.preempted:
                        return "preempted"
            self.rafcon.logger.info(f"Service {self.srv_name} is not available, waiting again...")
        return "success"

    def set_srv_req(self, srv_req):
        # This is a generic setter for service request
        self.srv_req = srv_req

    def set_srv_req_field_from_inputs(self, key, inputs):
        # This can be used for setting req from rafcon inputs
        # if the key of the req and rafcon inputs matches
        if hasattr(self.srv_req, key) and key in inputs:
            self.set_srv_req_field(key, inputs[key])
        else:
            error_msg = f"Request field does not have attribute {key} or rafcon inputs "\
                        f"does not have key {key}"
            self.rafcon.logger.error(error_msg)
            raise AttributeError(error_msg)

    def set_srv_req_field(self, key, value):
        # This is a generic setter for service request fields
        return setattr(self.srv_req, key, value)

    def call_srv(self):
        self.srv_future = self.cli.call_async(self.srv_req)
        self.rafcon.logger.info(f"ROS: called '{self.srv_name}' service")

    def wait_until_srv_completion(self):
        try:
            while self.srv_future.result() is None:
                # Use rafcon wait in case service provider crashes and won't return anything
                if self.rafcon.wait_for_interruption(0.00001):
                    if self.rafcon.preempted:
                        return "preempted"
                    if self.rafcon.paused:
                        self.rafcon.wait_for_unpause(0.1)
                        if self.rafcon.preempted:
                            return "preempted"
                pass
            self.srv_resp = self.srv_future.result()
            self.rafcon.logger.info(f"ROS: received response {self.replace_chars(self.srv_resp)}")
            if hasattr(self.srv_resp, "error_message") and self.srv_resp.error_message != "":
                self.rafcon.logger.error(self.srv_resp.error_message)
                return "aborted"
            return "success"
        except Exception as error:
            self.rafcon.logger.error(str(error))
            return "aborted"

    def destroy_client(self):
        self.node.destroy_client(self.cli)

    def get_srv_resp(self):
        # This is a generic getter for service response
        return self.srv_resp

    def get_srv_resp_field(self, key):
        # This is a generic getter for response message fields
        return getattr(self.srv_resp, key)

    def replace_chars(self, message):
        # Replace non-ascii chars with 'char(#)' for printing
        message_str = ''.join([i if 32 <= ord(i) < 127 else 'char(' + str(ord(i)) + ')' \
                               for i in str(message)])
        return message_str
