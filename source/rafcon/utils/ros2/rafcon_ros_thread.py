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
This files provides the implementation of a ROSThread to handle node spinning in RAFCON.
'''

import rclpy
import threading


class ROSThread():
    """
    A class holding the RAFCON ros thread to handle the spinning of the node
    without causing a blocking behavior of this state machine.
    The thread will use a ros executor to spin the rafcon_ros_node.

    Functions:
    start_thread: Starts the thread
    """
    def __init__(self, executor, node):
        self.executor = executor
        self.node = node
        self.rafcon_ros_thread = threading.Thread(target=self.executor.spin)

    def start_thread(self):
        self.rafcon_ros_thread.start()

    def __del__(self):
        self.node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()
