from __future__ import print_function
from builtins import str
import rospy
from turtlesim.msg import *


x = None
y = None
theta = None
turtle_name = ""


def save_turtle_pose(pose):
    global x
    global y
    global theta
    x = pose.x
    y = pose.y
    theta = pose.theta


def execute(self, inputs, outputs, gvm):

    turtle_name = inputs["turtle_name"]
    global_storage_id = inputs["global_storage_id_of_turtle_pos"]

    self.turtle_pos_subscriber = rospy.Subscriber("/" + turtle_name + "/pose", Pose, save_turtle_pose)

    global x
    global y
    global theta

    r = rospy.Rate(3)

    while x is None:
        print("turtle_position_subscriber: Wait for the subscriber to get a position message from turtle ", turtle_name)
        # actually ros.spin_once should be called but under python each subscriber gets his own thread
        # and cares for the subscriber to get called
        r.sleep()


    self.logger.info("turtle_position_subscriber: position of user turtle {} {} {}".format(str(x), str(y), str(theta)))
    gvm.set_variable(global_storage_id + "/" + "x", x)
    gvm.set_variable(global_storage_id + "/" + "y", y)
    gvm.set_variable(global_storage_id + "/" + "phi", theta)
    outputs["x_pos"] = x
    outputs["y_pos"] = y
    outputs["phi"] = theta
    
    if self.preempted:
        return "preempted"

    return 0
