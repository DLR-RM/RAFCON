# RAFCON ROS2 Initialization State Machine
# Author: Johannes Ernst, Ryo Sakagami
#
# NOTE: To use the state machine, ROS2 needs to be installed (tested with 'Humble')

import rclpy
import traceback
import string
import random

from rafcon.utils.ros2.rafcon_ros_node import RafconRosNode
from rafcon.utils.ros2.rafcon_ros_thread import ROSThread


def execute(self, inputs, outputs, gvm):

    # Initialize rclpy and the node
    try:
        rclpy.init(args=None)
    except RuntimeError:
        self.logger.warn("rclpy.init() was already called once before. Just continues.")
        pass

    try:
        # Initialize the node
        if not gvm.variable_exist("ros_initialized") or not gvm.get_variable("ros_initialized"):
            # Create random identifier for ros node to avoid interference
            random_identifier = ''.join(random.choices(string.ascii_letters,k=7))
            node_name = "rafcon_ros_node" + str("_") + str(random_identifier)
            self.logger.info("Creating node: " + node_name)

            # Initialize node with random identifier
            gvm.set_variable("ros_initialized", True)
            self.logger.info("creating tf listener node")
            rafcon_ros_node = RafconRosNode(node_name, namespace=inputs["robot_namespace"])
            gvm.set_variable("rafcon_ros_node", rafcon_ros_node, per_reference=True)

            # Initialize the ros executor
            executor = rclpy.executors.MultiThreadedExecutor(num_threads=5)
            executor.add_node(rafcon_ros_node)
            gvm.set_variable("ros_executor", executor, per_reference=True)

            # Create a thread to start the ROS2 executor and add thread to the GVM
            ros_thread = ROSThread(executor, rafcon_ros_node)
            gvm.set_variable("ros_thread", ros_thread, per_reference=True)
            ros_thread.start_thread()
            self.logger.info("Started ros2 node thread")

            # Create dicts to hold the handles for clients and publishers
            gvm.set_variable("ros_clients",  {}, per_reference=True)
            gvm.set_variable("ros_publishers",  {}, per_reference=True)

    except Exception as e:
        self.logger.error("Unexpected error:" + str(e) + str(traceback.format_exc()))
        return "aborted"

    return "success"
