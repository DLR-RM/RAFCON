import rospy
from turtlesim.msg import *
import time


x = None
y = None
theta = None
turtle_name = ""


def save_turtle1_pose(pose):
    global x
    global y
    global theta
    x = pose.x
    y = pose.y
    theta = pose.theta
    # print "positions of received turtle pose", x, y, theta


def execute(self, inputs, outputs, gvm):

    turtle_name = inputs["turtle_name"]
    global_storage_id = inputs["global_storage_id_of_turtle_pos"]

    self.turtle1_pos_subscriber = rospy.Subscriber("/" + turtle_name + "/pose", Pose, save_turtle1_pose)

    global x
    global y
    global theta

    counter = 0
    while x is None:
        print "Wait for the subscriber to get a position message from turtle ", turtle_name
        # actually ros.spin_once should be called but under pyton each subscriber gets his own thread
        # and cares for the subscriber to get called
        time.sleep(0.1)
        counter += 1
        if counter == 10:
            break

    print "position of user turtle ", x, y, theta
    gvm.set_variable(global_storage_id + "/" + "x", x)
    gvm.set_variable(global_storage_id + "/" + "y", y)
    gvm.set_variable(global_storage_id + "/" + "phi", theta)
    outputs["x_pos"] = x
    outputs["y_pos"] = y
    outputs["phi"] = theta

    return 0
