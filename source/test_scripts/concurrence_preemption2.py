import time

def execute(self, inputs, outputs):
    print "concurrence_barrier2: Hello world"
    print inputs
    print outputs
    outputs["FirstDataOutputPort"] = 10.0
    self.print_state_information()
    for i in range(5):
        time.sleep(1)
        print "concurrence_barrier2 state active"
        if self.preempted:
            print "concurrence_barrier2: I was preempted"
            return 2
    return 3
