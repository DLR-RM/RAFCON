import time

def execute(self, inputs, outputs, gvm):
    print "Hello world: Execute loop state 1"
    # time.sleep(1)
    counter = gvm.get_variable("counter")
    gvm.set_variable("counter", counter + 1)
    return 3
