"""The module uses the module pymouse of pypi package PyMouse or the equivalent element of the package PyUserInput."""

import os
import time
from math import *
import numpy as np

import pymouse
# print os.environ['PYTHONPATH'].replace(':', '\n')
# import pyuserinput
mouse = pymouse.PyMouse()
TIME_STEP = 0.01


def move(target_pos, speed=1000):
    """ Interpolate the mouse to a specific position
    :param tuple target_pos: X and Y target coordinates on the display
    :param speed: speed of the motion, if this is None the move result into a direct step to the target_pos coordinates
    :return:
    """
    start_pos = mouse.position()
    start_pos_vec = np.array(start_pos)
    distance_vec = np.array(target_pos) - np.array(start_pos)

    if speed is not None:
        distance = np.linalg.norm(distance_vec)
        num_of_steps = int(ceil(distance/(speed*TIME_STEP)))
        positions = [start_pos_vec + distance_vec/float(num_of_steps)*(pos_index + 1.0)
                     for pos_index in range(num_of_steps)]
        print "position start: ", start_pos
        for pos in positions:
            mouse.move(int(pos[0]), int(pos[1]))
            # print "with index: ", pos
            time.sleep(TIME_STEP)
    print "last index: ", target_pos[0], target_pos[1]
    mouse.move(target_pos[0], target_pos[1])
    time.sleep(TIME_STEP)

# move((100., 100.), (1000., 200.), speed=None)


def double_click(pos):
    mouse.click(pos[0], pos[1])
    mouse.click(pos[0], pos[1])

# double_click(pos=(400, 10))


def drag_and_drop(from_pos, to_pos, speed=None, button=1):
    """Drag and drop movement

    The function is doing the hole motion interpolated if speed is not None. After reaching the from_pos the the mouse
    is pressed and at the target position the mouse is released
    """
    # print "START drag_and_drop"
    move(target_pos=from_pos, speed=speed)
    # print "position", mouse.position()
    # print "PRESS: ", from_pos
    mouse.press(x=from_pos[0], y=from_pos[1], button=button)
    time.sleep(TIME_STEP*10)
    move(target_pos=to_pos, speed=speed)
    # print "position", mouse.position()
    # print "RELEASE: ", to_pos
    time.sleep(TIME_STEP*10)
    mouse.release(x=to_pos[0], y=to_pos[1], button=button)
    # print "STOP drag_and_drop"

# time.sleep(3)
# input_pos = mouse.position()
# print "### INIT", input_pos
# # drag_and_drop(from_pos=(1200, 10), to_pos=(2000, 100))
# mouse.move(1200, 530)
# # input_pos
# drag_and_drop(from_pos=input_pos, to_pos=(1600, 530), speed=500)
# time.sleep(1)
# mouse.move(2000, 530)
# # drag_and_drop(from_pos=(1500, 530), to_pos=(1600, 530), speed=500)
# drag_and_drop(from_pos=(1600, 530), to_pos=(1500, 530), speed=500)

# print pymouse
# print mouse
