import time

def execute(self, inputs, outputs, gvm):
    print "concurrence_barrier1: Hello world"
    print inputs
    print outputs
    for i in range(2):
        time.sleep(inputs["input_data_port1"])
        print "concurrence_barrier1 state active"
    gvm.set_variable("state1_exit_code", "DF3LF")
    return 3
