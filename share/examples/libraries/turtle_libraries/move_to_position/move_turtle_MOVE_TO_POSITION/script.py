from __future__ import print_function
from builtins import str
import math
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

max_x_move = 1.0
min_x_move = 0.3
max_rotation = 0.75


def normalize_angle_to_positive_360(angle):
    return (angle + 2.0 * math.pi) % (2 * math.pi)


def normalize_angle_to_pos_neg_180(angle):
    result_angle = angle
    while result_angle > math.pi:
        result_angle -= 2 * math.pi
    while result_angle < - math.pi:
        result_angle += 2 * math.pi
    return result_angle


def sign(number):
    if number < 0:
        return -1
    else:
        return 1


def set_velocity(x, phi, turtle_name, logger):
    position_vector = Vector3(x, 0, 0)
    rotation_vector = Vector3(0, 0, phi)
    twist_msg = Twist(position_vector, rotation_vector)
    try:
        logger.info("move_to_position: publish twist to turtle".format(turtle_name))
        turtle_vel_publisher = rospy.Publisher("/" + turtle_name + "/cmd_vel", Twist, queue_size=10, latch=True)
        turtle_vel_publisher.publish(twist_msg)
        rate = rospy.Rate(10)
        rate.sleep()
    except rospy.ROSInterruptException as e:
        logger.error("Failed to send a velocity command to turtle {}: {}".format(turtle_name, str(e)))


def execute(self, inputs, outputs, gvm):
    self.logger.info("move_to_position: inputs of move_difference: {}".format(str(inputs)))

    global_storage_id_of_turtle_pose = inputs["global_storage_id_of_turtle_pos"]
    turtle_name = inputs["turtle_name"]

    my_x = gvm.get_variable(global_storage_id_of_turtle_pose + "/" + "x")
    my_y = gvm.get_variable(global_storage_id_of_turtle_pose + "/" + "y")
    my_phi = gvm.get_variable(global_storage_id_of_turtle_pose + "/" + "phi")

    r = rospy.Rate(2)

    if (my_x or my_y or my_phi) is None:
        print("move_to_position: global variable is None, return from execute function")
        print("move_to_position: my_x")
        print("move_to_position: my_y")
        print("move_to_position: my_phi")
        r.sleep()
        return 1

    print("my_x: ", my_x)
    print("my_y: ", my_y)
    print("my_phi: ", my_phi)

    x_diff = inputs["x_pos"] - my_x
    y_diff = inputs["y_pos"] - my_y
    theta = normalize_angle_to_pos_neg_180(my_phi)

    ######################################################
    # calc the target orientation to drive
    ######################################################

    target_direction = math.atan2(y_diff, x_diff)
    print("move_to_position: target_direction: ", target_direction)

    orientation_diff = normalize_angle_to_pos_neg_180(theta - target_direction)
    print("move_to_position: orientation_diff: ", orientation_diff)

    # negative sign as we want to countersteer
    target_orientation_sign = -sign(orientation_diff)
    # normalize absolute value to max_rotation
    tmp = math.fabs(orientation_diff) / max_rotation
    if tmp > 1.0:
        tmp = 1.0
    theta_move = tmp * max_rotation * target_orientation_sign

    ######################################################
    # calc distance to drive
    ######################################################
    # normalize to max_x_dist
    distance = math.sqrt(x_diff**(2.0) + y_diff**(2.0))
    tmp = distance / max_x_move
    if tmp > 1.0:
        tmp = 1.0
    x_move = tmp * max_x_move
    print("move_to_position: x_move before taking direction into account: ", x_move)

    if math.fabs(theta_move) > math.pi / 3:
        x_move = min_x_move
    print("move_to_position: x_move before checking minimal distance: ", x_move)

    if distance < 1.5:
        x_move = 0
        theta_move = 0
    self.logger.info("move_to_position: final theta_move: {}".format(str(theta_move)))
    self.logger.info("move_to_position: final x_move: {}".format(str(x_move)))

    set_velocity(x_move, theta_move, turtle_name, self.logger)
    if x_move == 0:
        return 0
 
    r.sleep()
    return 1