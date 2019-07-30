from builtins import str
import rospy
from turtlesim.srv import *
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

def execute(self, inputs, outputs, gvm):
    turtle_name = inputs["turtle_name"]
    x = inputs["x_vel"]
    phi = inputs["phi_vel"]
    rate = rospy.Rate(10)
    position_vector = Vector3(x, 0, 0)
    rotation_vector = Vector3(0, 0, phi)
    twist_msg = Twist(position_vector, rotation_vector)
    self.logger.info("moving turtle {} {}".format(str(x), str(phi)))
    self.logger.info("publish twist to turtle {}".format(turtle_name))
    turtle_vel_publisher = rospy.Publisher("/" + turtle_name + "/cmd_vel", Twist, queue_size=10, latch=True)
    turtle_vel_publisher.publish(twist_msg)
    rate.sleep()

    return 0