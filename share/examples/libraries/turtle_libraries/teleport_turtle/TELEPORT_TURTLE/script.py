import rospy
from turtlesim.srv import *


def execute(self, inputs, outputs, gvm):

        turtle_name = inputs["turtle_name"]
        x = inputs["x_pos"]
        y = inputs["y_pos"]
        phi = inputs["phi"]

        service = "/" + turtle_name + "/teleport_absolute"
        rospy.wait_for_service(service)
        try:
            move_turtle = rospy.ServiceProxy(service, TeleportAbsolute)
            resp1 = move_turtle(x, y, phi)
            print "ROS external module: executed the ", service, " service"
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        return 0
