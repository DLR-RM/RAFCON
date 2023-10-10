import rospy
import tf
import time
import traceback


def execute(self, inputs, outputs, gvm):

        # check if the roscore is already running
        try:
            rospy.wait_for_service("/rosout/get_loggers", 5.0)
        except Exception as e:
            self.logger.error("Exception: " + str(e) + str(traceback.format_exc()))
            return -1

        node_name = inputs["node_name"]
        if type(node_name).__name__ == "unicode":
            node_name = node_name.encode('ascii', 'ignore')
            # print node_name
                    
        self.logger.info("Creating node: " + node_name)
        try:
            if not gvm.variable_exist("ros_node_initialized") or not gvm.get_variable("ros_node_initialized"):
                gvm.set_variable("ros_node_initialized", True)
                rospy.init_node(node_name, anonymous=True, disable_signals=True)
                listener = tf.TransformListener()
                self.logger.info("Creating node: " + str(listener))
                gvm.set_variable("tf_listener", listener,per_reference=True)
            if not gvm.variable_exist("robot_id"):
                try: 
                    gvm.set_variable("robot_id",rospy.get_param("robot_id"))
                except KeyError as e:
                    self.logger.warn("robot_id does not exist in the parameter server")

        except Exception as e:
            self.logger.error("Unexpected error:" + str(e) + str(traceback.format_exc()))

        return 0
