import rospy
import time

def execute(self, inputs, outputs, gvm):

        node_name = inputs["node_name"]
        print "Creating node: ", node_name
        #rospy.init_node(node_name, anonymous=True)
        rospy.init_node(node_name, anonymous=False, disable_signals=True)
        return 0


if __name__ == '__main__':
        counter = 0
        execute(counter, {"node_name": "my_node"}, [], counter)
        rate = rospy.Rate(1.0)
        while True:
                rate.sleep()
                print "alive"