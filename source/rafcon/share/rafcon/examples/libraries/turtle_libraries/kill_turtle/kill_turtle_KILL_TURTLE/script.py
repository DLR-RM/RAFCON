import rospy
from turtlesim.srv import Kill
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

def execute(self, inputs, outputs, gvm):
    turtle_name = inputs["turtle_name"]
    service = "/kill"
    rospy.wait_for_service(service)
    kill_turtle_service = rospy.ServiceProxy(service, Kill)
    resp1 = kill_turtle_service(turtle_name)
    self.logger.info("ROS external module: executed the {} service".format(service))
    return 0