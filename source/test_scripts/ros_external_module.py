import roslib; roslib.load_manifest('beginner_tutorials')
import sys
import rospy
from beginner_tutorials.srv import *

class RosModule:

    def __init__(self):
        self.my_first_var = 10
        self.my_second_var = 30

    def start(self):
        print "ROS module: started"
        print "ROS module: my_first_var is " + str(self.my_first_var)

    def stop(self):
        print "ROS module: stopped"

    def pause(self):
        print "ROS module: paused"

    def usage(self):
        return "%s [x y]"%sys.argv[0]

    def add_two_ints_client(self):
        x = self.my_first_var
        y = self.my_second_var
        rospy.wait_for_service('add_two_ints')
        try:
            add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
            resp1 = add_two_ints(x, y)
            print "ROS module: the sum of the ros service call: "
            print resp1.sum
            #return resp1.sum
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
