import rospy
from turtlesim.srv import Kill
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

def execute(self, inputs, outputs, gvm):

        turtle_name = inputs["turtle_name"]

        service = "/kill"
        rospy.wait_for_service(service)
        try:
            kill_turtle_service = rospy.ServiceProxy(service, Kill)
            resp1 = kill_turtle_service(turtle_name)
            print "ROS external module: executed the ", service, " service"
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        return 0