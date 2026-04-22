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
This files provides the implementation of a SubscriberHandle for ROS2 subscriptions in RAFCON.
'''

import threading
import time

# NOTE: Import to catch InvalidHandle bug (see below)
import rclpy
from rclpy._rclpy_pybind11 import InvalidHandle


class SubscriberHandle():
    """
    A class for the ROS2 subscriber handle for usage in RAFCON state machines.

    Variables:
    rafcon: The RAFCON handle to access, e.g. the logger
    node: The ROS node that is used for instantiation
    sub_topic: Stores the subscription topic
    sub_msg_def: Stores the subscription message definition
    sub_msg: Stores the actual subscription message
    sub_msg_lock: A threading lock to prevent race conditions when receiving multiple
                  subscriptions
    qos_profile: Custom quality of service profile
    callback_group: Optional callback group for multi threaded execution

    Functions:
    sub_callback: Function that handles a received subscription
    get_sub_msg: Returns the entire subscription message
    get_sub_msg_field: Gets the received subscription message field
    wait_for_sub_msg: Wait until the desired subscription arrives
    destroy_subscriber: Destroys the subscriber saved in the handle
    replace_chars: Replace non-ascii chars with 'char(#)' for printing
    """

    def __init__(self, rafcon, node, sub_msg_def, sub_topic, qos_profile=10, callback_group=None):
        self.rafcon = rafcon
        self.node = node
        self.sub_msg_def = sub_msg_def
        self.sub_msg = None
        self.sub_topic = sub_topic
        self.sub_msg_lock = threading.Lock()
        self.qos_profile = qos_profile
        self.callback_group = callback_group

        # Check if node is already initilized
        if self.node is None:
            raise RuntimeError("ROS2 node is not available in gvm. "\
                               "Have you called init_ros2_node?")

        # Create publisher
        self.sub = self.node.create_subscription(self.sub_msg_def,
                                                 self.sub_topic,
                                                 self.sub_callback,
                                                 self.qos_profile,
                                                 callback_group=self.callback_group)

    def sub_callback(self, msg):
        # Use a threading lock to prevent race condition with execute_subscribing
        with self.sub_msg_lock:
            self.sub_msg = msg
        self.rafcon.logger.info(f"ROS: subscribed from '{self.sub_topic}': "\
                                f"{self.replace_chars(self.sub_msg)}")
        # Wait a minimal time, so the while loop in execute_subscribing
        # catches the first received message
        time.sleep(0.002)

    def get_sub_msg(self):
        # This is a generic getter for the subscription message
        return self.sub_msg

    def get_sub_msg_field(self, key):
        # This is a generic getter of received message
        return getattr(self.sub_msg, key)

    def wait_for_sub_msg(self, timeout=None):
        try:
            # Set preempted as output if no subscription happens
            result = "preempted"

            # Constantly check for subscription over the period of timeout
            start_time = time.monotonic()
            if timeout == None:
                self.rafcon.logger.warn("You should specify a 'timeout' for the subscriber. "\
                                        "Now using standard timeout of 5s.")
                timeout_limit = 5
            else:
                timeout_limit = timeout
                if timeout_limit < 0:
                    timeout_limit = 50000000
            while time.monotonic() - start_time < timeout_limit:
                # Use a threading lock to prevent race condition with sub_callback()
                with self.sub_msg_lock:
                    if self.sub_msg is not None:
                        result = "success"
                        break
                # Use RAFCON wait in case of manual interruption
                if self.rafcon.wait_for_interruption(0.001):
                    if self.rafcon.preempted:
                        result = "preempted"
                        break
                    if self.rafcon.paused:
                        self.rafcon.wait_for_unpause(0.1)
                        if self.rafcon.preempted:
                            result = "preempted"
                            break
        except Exception as error:
            self.rafcon.logger.error(str(error))
            result = "aborted"
        finally:
            if result == "preempted":
                self.rafcon.logger.warn(f"No message was received on {self.sub_topic}")
            # NOTE: Hack to prevent error because of sporadic ROS2 bug:
            # "InvalidHandle: cannot use Destroyable because destruction was requested"
            # See https://github.com/ros2/rclpy/issues/1142
            try:
                self.node.destroy_subscription(self.sub)
            except InvalidHandle as destruction_error:
                self.rafcon.logger.warn(
                    f"Caught InvalidHandle ROS2 error: {destruction_error}"
                )
                return result
            return result

    def replace_chars(self, message):
        # Replace non-ascii chars with 'char(#)' for printing
        message_str = ''.join([i if 32 <= ord(i) < 127 else 'char(' + str(ord(i)) + ')' \
                               for i in str(message)])
        return message_str
