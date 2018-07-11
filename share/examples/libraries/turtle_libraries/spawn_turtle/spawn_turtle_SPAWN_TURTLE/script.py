import rospy
from turtlesim.srv import *
from turtlesim.msg import *


pose_received = False


def check_turtle_pose(pose):
    global pose_received
    pose_received = True


def execute(self, inputs, outputs, gvm):
    turtle_name = inputs["turtle_name"]
    x = inputs["x_pos"]
    y = inputs["y_pos"]
    phi = inputs["phi"]

    service = "/spawn"
    rospy.wait_for_service(service)
    spawn_turtle_service = rospy.ServiceProxy(service, Spawn)
    resp1 = spawn_turtle_service(x, y, phi, turtle_name)
    self.logger.info("ROS external module: executed the {} service".format(service))

    turtle_pos_subscriber = rospy.Subscriber("/" + turtle_name + "/pose", Pose, check_turtle_pose)

    r = rospy.Rate(10)
    global pose_received
    # wait until the first pose message was received
    while not pose_received:
        r.sleep()

    return 0
