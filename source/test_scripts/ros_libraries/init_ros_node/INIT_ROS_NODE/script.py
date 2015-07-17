import rospy
import time
import traceback


def execute(self, inputs, outputs, gvm):

        # check if the roscore is already running
        try:
            rospy.wait_for_service("/rosout/get_loggers", 3.0)
        except Exception, e:
            self.logger.error("Exception: " + str(e) + str(traceback.format_exc()))
            return -1

        node_name = inputs["node_name"]
        self.logger.info("Creating node: " + node_name)
        try:
            if gvm.variable_exist("ros_node_initialized"):
                ros_node_initialized = gvm.get_variable("ros_node_initialized")
                if not ros_node_initialized:
                    gvm.set_variable("ros_node_initialized", True)
                    rospy.init_node(node_name, anonymous=False, disable_signals=True)
            else:
                gvm.set_variable("ros_node_initialized", True)
                rospy.init_node(node_name, anonymous=False, disable_signals=True)
        except Exception, e:
            self.logger.error("Unexpected error:" + str(e) + str(traceback.format_exc()))

        return 0
