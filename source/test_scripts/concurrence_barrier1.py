import time

def execute(self, inputs, outputs):
    print "concurrence_barrier1: Hello world"
    print inputs
    print outputs
    outputs["FirstDataOutputPort"] = 10.0
    self.print_state_information()
    for i in range(3):
        time.sleep(1)
        print "concurrence_barrier1 state active"
    return 3
