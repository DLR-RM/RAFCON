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
This files provides the implementation of a PublisherHandle for ROS2 publishing in RAFCON.
'''

import threading


class PublisherHandle():
    """
    A class for the ROS2 publisher handle for usage in RAFCON state machines.

    Variables:
    rafcon: The RAFCON handle to access, e.g. the logger
    gvm: The global variable manager to lookup node and dictionaries
    node: The ROS node that is used for instantiation
    pub_topic: Stores the publisher topic
    pub_msg_def: Stores the publishing message definition
    pub_msg: Stores the actual publishing message
    qos_profile: Custom quality of service profile
    callback_group: Optional callback group for multi threaded executtion

    Functions:
    set_pub_msg: Sets the pub_msg manually
    set_pub_msg_field_from_inputs: Sets the message fields from given inputs
    wait_for_sub: Wait for a subscriber to subscribe to the publisher topic
    publish_msg: Publishes the pub_msg with the given publisher handle
    replace_chars: Replace non-ascii chars with 'char(#)' for printing
    """

    def __init__(self, rafcon, gvm, pub_msg_def, pub_topic, qos_profile=10, callback_group=None):
        self.rafcon = rafcon
        self.gvm = gvm
        self.pub_msg_def = pub_msg_def
        self.pub_msg = self.pub_msg_def()
        self.pub_topic = pub_topic
        self.qos_profile = qos_profile
        self.callback_group = callback_group

        # Get node and check if it is already initilized
        self.node = self.gvm.get_variable('rafcon_ros_node', per_reference=True)
        if self.node is None:
            raise RuntimeError("ROS2 node is not available in gvm. "\
                               "Have you called init_ros2_node?")

        # Get publisher from gvm dictionary or create anew
        dict_name = "ros_publishers"
        access_key = self.gvm.lock_variable(dict_name, block=True)
        try:
            ros_publishers = self.gvm.get_variable(dict_name,
                                                   per_reference=True,
                                                   access_key=access_key)
            if self.pub_topic in ros_publishers:
                self.pub, self.lock = ros_publishers[self.pub_topic]
                self.rafcon.logger.info(f"Using existing publisher '{self.pub_topic}'")
            else:
                self.pub = self.node.create_publisher(self.pub_msg_def,
                                                      self.pub_topic,
                                                      self.qos_profile,
                                                      callback_group=self.callback_group)
                self.lock = threading.Lock()
                ros_publishers[self.pub_topic] = (self.pub, self.lock)
                self.gvm.set_variable(dict_name,
                                      ros_publishers,
                                      per_reference=True,
                                      access_key=access_key)
                self.rafcon.logger.info(
                    f"Created new publisher '{self.pub_topic}' and saved to GVM"
                )
        finally:
            self.gvm.unlock_variable(dict_name, access_key)


    def set_pub_msg(self, pub_msg):
        # This is a generic setter for the publishing message
        self.pub_msg = pub_msg

    def set_pub_msg_field(self, key, value):
        # This is a generic setter for the publishing message field
        return setattr(self.pub_msg, key, value)

    def set_pub_msg_field_from_inputs(self, key, inputs):
        # This can be used for setting message fields from rafcon inputs
        # if the key of the message and rafcon inputs matches
        if hasattr(self.pub_msg, key) and key in inputs:
            self.set_pub_msg_field(key, inputs[key])
        else:
            error_msg = f"Request field does not have attribute {key} or rafcon inputs "\
                        f"does not have key {key}"
            self.rafcon.logger.error(error_msg)
            raise AttributeError(error_msg)

    def wait_for_sub(self, timeout=1):
        timeout = timeout
        # Wait with RAFCON until a subscriber is available or timeout
        for i in range(round(timeout*1000)):
            if not self.pub.get_subscription_count():
                if self.rafcon.wait_for_interruption(0.001):
                    if self.rafcon.preempted:
                        return "preempted"
                    if self.rafcon.paused:
                        self.rafcon.wait_for_unpause(0.1)
                        if self.rafcon.preempted:
                            return "preempted"
            else:
                return
        self.rafcon.logger.error(f"No subscription node available after {timeout}s, "\
                                 f"publishing still...")

    def publish_msg(self):
        try:
            self.pub.publish(self.pub_msg)
            self.rafcon.logger.info(f"ROS: published on '{self.pub_topic}': "\
                                    f"{self.replace_chars(self.pub_msg)}")
            return "success"
        except Exception as error:
            self.rafcon.logger.error(str(error))
            return "aborted"

    def destroy_publisher(self):
        self.node.destroy_publisher(self.pub)

    def replace_chars(self, message):
        # Replace non-ascii chars with 'char(#)' for printing
        message_str = ''.join([i if 32 <= ord(i) < 127 else 'char(' + str(ord(i)) + ')' \
                               for i in str(message)])
        return message_str
