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
This files provides the implementation of a RafconRosNode to create a ROS2 node for RAFCON.
'''

import rclpy
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class RafconRosNode(Node):
    """
    A class holding the RAFCON ros node that will perform publishing,
    subscribing and service calls. The instance of this class will be 
    stored in the GVM.
    Additionally, a tf listener and buffer are instantiated.

    Variables:
    tf_buffer: Uses output of tf_listerner for storage
    tf_listener: Listens to tf topic to store in buffer
    """

    def __init__(self, node_name, namespace=""):
        super().__init__(node_name, namespace=namespace)
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=60))
        self.tf_listener = TransformListener(self.tf_buffer, self)
