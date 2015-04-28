import time

def execute(self, inputs, outputs, gvm):

    turtle_name = inputs["turtle_name"]

    while not gvm.variable_exist(turtle_name + "/x"):
        print "Waiting for global variable to be created"
        time.sleep(0.5)

    print gvm.get_variable(turtle_name + "/x")
    print gvm.get_variable(turtle_name + "/y")
    print gvm.get_variable(turtle_name + "/phi")
    return 0
