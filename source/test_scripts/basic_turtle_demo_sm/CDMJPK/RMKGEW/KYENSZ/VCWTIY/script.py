import rospy
from turtlesim.srv import *
from std_srvs.srv import Empty

def execute(self, inputs, outputs, gvm):
        service = "/clear"
        rospy.wait_for_service(service)
        try:
            clear_turtle_area_service = rospy.ServiceProxy(service, Empty)
            resp1 = clear_turtle_area_service()
            print "ROS external module: executed the ", service, " service"
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        return 0