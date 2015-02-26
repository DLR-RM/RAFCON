import rospy
from turtlesim.srv import *


def execute(self, inputs, outputs, gvm):

        turtle_name = inputs["turtle_name"]
        x = inputs["x_pos"]
        y = inputs["y_pos"]
        phi = inputs["phi"]

        service = "/spawn"
        rospy.wait_for_service(service)
        try:
            spawn_turtle_service = rospy.ServiceProxy(service, Spawn)
            resp1 = spawn_turtle_service(x, y, phi, turtle_name)
            print "ROS external module: executed the ", service, " service"
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        return 0
