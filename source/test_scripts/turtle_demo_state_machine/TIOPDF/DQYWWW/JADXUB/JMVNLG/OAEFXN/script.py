import time

def execute(self, inputs, outputs, external_modules, gvm):
    [x, y, theta] = external_modules["ros"].instance.get_position_of_turtle1()
    print "position of user turtle ", x, y, theta
    outputs["user turtle position"] = {}
    outputs["user turtle position"]["x"] = x
    outputs["user turtle position"]["y"] = y
    #time.sleep(1.0)
    return 0
