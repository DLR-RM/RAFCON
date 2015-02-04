import roslib
#roslib.load_manifest('beginner_tutorials')
import sys
import rospy
import time
#from beginner_tutorials.srv import *
from turtlesim.srv import *
from turtlesim.msg import *
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from random import randint, uniform
from std_srvs.srv import Empty


class RosModule:

    def __init__(self):
        self.my_first_var = 10
        self.my_second_var = 30
        self.turtle1_vel_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10, latch=True)
        self.turtle2_vel_publisher = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10, latch=True)

        self.turtle1_pos_subscriber = rospy.Subscriber("/turtle1/pose", Pose, self.save_turtle_pose)

        rospy.init_node('awesome_tool_turtle_commander', anonymous=True)
        self.rate = rospy.Rate(10)
        self.turtle1_pose = None
        #rospy.spin()

    def start(self):
        print "ROS module: started"
        print "ROS module: my_first_var is " + str(self.my_first_var)

    def stop(self):
        print "ROS module: stopped"

    def pause(self):
        print "ROS module: paused"

    def usage(self):
        return "%s [x y]"%sys.argv[0]

    def save_turtle_pose(self, Pose):
        #print "The pose of turtle1 was saved"
        self.turtle1_pose = Pose

    # def add_two_ints_client(self):
    #     x = self.my_first_var
    #     y = self.my_second_var
    #     rospy.wait_for_service('add_two_ints')
    #     try:
    #         add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
    #         resp1 = add_two_ints(x, y)
    #         print "ROS module: the sum of the ros service call: "
    #         print resp1.sum
    #         #return resp1.sum
    #     except rospy.ServiceException, e:
    #         print "Service call failed: %s"%e

    def teleport_turtle(self, turtle_name, x, y, phi):
        service = "/" + turtle_name + "/teleport_absolute"
        rospy.wait_for_service(service)
        try:
            move_turtle = rospy.ServiceProxy(service, TeleportAbsolute)
            resp1 = move_turtle(x, y, phi)
            print "ROS external module: executed the ", service, " service"
            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def spawn_turtle(self, turtle_name, x, y, phi):
        service = "/spawn"
        rospy.wait_for_service(service)
        try:
            spawn_turtle_service = rospy.ServiceProxy(service, Spawn)
            resp1 = spawn_turtle_service(x, y, phi, turtle_name)
            print "ROS external module: executed the ", service, " service"
            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def clear_field(self):
        service = "/clear"
        rospy.wait_for_service(service)
        try:
            clear_turtle_area_service = rospy.ServiceProxy(service, Empty)
            resp1 = clear_turtle_area_service()
            print "ROS external module: executed the ", service, " service"
            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def move_turtle(self, turtle_name, x, y, phi):
        position_vector = Vector3(x, y, 0)
        rotation_vector = Vector3(0, 0, phi)
        twist_msg = Twist(position_vector, rotation_vector)
        print twist_msg
        try:
            if turtle_name == "turtle1":
                print "publish twist to turtle1"
                self. turtle1_vel_publisher.publish(twist_msg)
                self.rate.sleep()
            elif turtle_name == "turtle2":
                print "publish twist to turtle2"
                self. turtle2_vel_publisher.publish(twist_msg)
                self.rate.sleep()
        except rospy.ROSInterruptException:
            print "Failed to send a velocity command to turtle1"

    def kill_turtle(self, turtle_name):
        service = "/kill"
        rospy.wait_for_service(service)
        try:
            kill_turtle_service = rospy.ServiceProxy(service, Kill)
            resp1 = kill_turtle_service(turtle_name)
            print "ROS external module: executed the ", service, " service"
            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def get_position_of_turtle(self, turtle_name):
        x = self.turtle1_pose.x
        y = self.turtle1_pose.y
        theta = self.turtle1_pose.theta
        linear_velocity = self.turtle1_pose.linear_velocity
        angular_velocity = self.turtle1_pose.angular_velocity
        print "The pose of turtle 1 is: ", [x, y, theta]

if __name__ == '__main__':
    my_ros_module = RosModule()
    my_ros_module.spawn_turtle("turtle2", 4, 4, 1)
    my_ros_module.teleport_turtle("turtle1", randint(2, 9), randint(2, 9), uniform(0, 3.1415))
    my_ros_module.move_turtle("turtle1", 2, 1, 1)
    time.sleep(2)
    my_ros_module.get_position_of_turtle("turtle1")
    my_ros_module.clear_field()
    my_ros_module.kill_turtle("turtle2")

