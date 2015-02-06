import time
import math


def distance_to_food(my_pose, food_pose):
    x_diff = my_pose[0] - food_pose[0]
    y_diff = my_pose[1] - food_pose[1]
    return math.sqrt(x_diff**(2.0) + y_diff**(2.0))


def execute(self, inputs, outputs, external_modules, gvm):
    print "Check for food state: executing ..."
    while True:
        [my_x, my_y, my_theta] = external_modules["ros"].instance.get_position_of_turtle("bot_turtle")
        #print "Check for food state: my current position is %s %s %s " % (str(my_x), str(my_y), str(my_theta))
        for i in range(4):
            turtle_name = "food" + str(i)
            #print turtle_name
            if external_modules["ros"].instance.turtle_exists(turtle_name):
                [x, y, theta] = external_modules["ros"].instance.get_position_of_turtle(turtle_name)
                distance = distance_to_food([my_x, my_y, my_theta], [x, y, theta])
                #print "Check for food state: distance to turtle food%s is %s" % (str(i), str(distance))
                if distance < 3.0:
                    print "%s is near enough to get munched" % turtle_name
                    outputs["target turtle"] = turtle_name
                    return 0
        time.sleep(0.5)
