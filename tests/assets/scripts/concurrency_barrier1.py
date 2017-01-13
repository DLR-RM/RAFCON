import time


def execute(self, inputs, outputs, gvm):
    print "concurrence_barrier1: Hello world"
    outputs["output_data_port1"] = 10.0
    for i in range(3):
        time.sleep(inputs["input_data_port1"])
        print "concurrence_barrier1 state active"
    gvm.set_variable("var_x", 10)
    return 3
