import time

def execute(self, inputs, outputs, gvm):
    print "concurrence_barrier2: Hello world"
    print inputs
    print outputs
    outputs["output_data_port1"] = 10.0
    self.print_state_information()
    for i in range(3):
        time.sleep(inputs["input_data_port1"])
        print "concurrence_barrier2 state active"
    gvm.set_variable("var_y", 20)
    return 3
