import time

def execute(self, inputs, outputs, external_modules, gvm):
    print "inputs of calculate_position_difference: ", inputs
    [x, y, theta] = external_modules["ros"].instance.get_position_of_turtle("bot_turtle")
    print "position of bot_turtle ", x, y, theta
    outputs["distance to user turtle"] = {}
    outputs["distance to user turtle"]["x-diff"] = inputs["user turtle position"]["x"] - x
    outputs["distance to user turtle"]["y-diff"] = inputs["user turtle position"]["y"] - y
    outputs["distance to user turtle"]["theta"] = theta
    #time.sleep(1)
    return 0
