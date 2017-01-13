import time

def execute(self, inputs, outputs, gvm):
    print "concurrence_barrier2: Hello world"
    print inputs
    print outputs
    for i in range(5):
        time.sleep(inputs["input_data_port1"])
        print "concurrence_barrier2 state active"
        if self.preempted:
            print "concurrence_barrier2: I was preempted"
            tmp = gvm.get_variable("state1_exit_code")
            gvm.set_variable("preempted_state2_code", tmp+"XD34G")
            return -2
    return 3
