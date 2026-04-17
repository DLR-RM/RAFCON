import rclpy

def execute(self, inputs, outputs, gvm):
    if not (gvm.variable_exist("rafcon_ros_node") and gvm.variable_exist("ros_thread") and gvm.variable_exist("ros_executor")): 
        self.logger.warn("ROS2 not initialized! Nothing to shutdown...")
    else:
        if gvm.variable_exist("rafcon_ros_node"):  
            rafcon_ros_node = gvm.get_variable("rafcon_ros_node")
            rafcon_ros_node.destroy_node()
            gvm.delete_variable("rafcon_ros_node")
            gvm.delete_variable("ros_initialized")
        if gvm.variable_exist("ros_thread"):     
            gvm.delete_variable("ros_thread")
        if gvm.variable_exist("ros_executor"):
            gvm.delete_variable("ros_executor")
        # Note: Clients and publishers are already destroyed with the node
        if gvm.variable_exist("ros_clients"):
            gvm.delete_variable("ros_clients")
        if gvm.variable_exist("ros_publishers"):
            gvm.delete_variable("ros_publishers")
        self.logger.info("Shutdown ROS2 and deleted related variables!")

    return "success"