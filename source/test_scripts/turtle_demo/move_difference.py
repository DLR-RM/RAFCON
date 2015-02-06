import time
import math
import numpy

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


def execute(self, inputs, outputs, external_modules, gvm):
    print "inputs of move_difference: ", inputs
    x_diff = inputs["distance to user turtle"]["x-diff"]
    y_diff = inputs["distance to user turtle"]["y-diff"]
    theta = normalize_angle_to_pos_neg_180(inputs["distance to user turtle"]["theta"])

    ######################################################
    # calc the target orientation to drive
    ######################################################

    target_direction = math.atan2(y_diff, x_diff)
    print "target_direction: ", target_direction

    orientation_diff = normalize_angle_to_pos_neg_180(theta - target_direction)
    print "orientation_diff: ", orientation_diff

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
    print "x_move before taking direction into account: ", x_move

    if math.fabs(theta_move) > math.pi / 3:
        x_move = min_x_move
    print "x_move before checking minimal distance: ", x_move

    if distance < 1.5:
        x_move = 0
        theta_move = 0
    print "final theta_move: ", theta_move
    print "final x_move: ", x_move

    external_modules["ros"].instance.move_turtle("bot_turtle", x_move, 0, theta_move)
    if x_move == 0:
        # sleep if the user turtle does not move
        # sleep after move, else the turtle will overshoot
        time.sleep(3.0)

    time.sleep(0.5)
    return 0