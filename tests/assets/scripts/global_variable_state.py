def execute(self, inputs, outputs, gvm):
    print "Global variable test state is executing ..."
    print inputs
    print outputs
    gvm.set_variable("varX", 10)
    outputs["output_data_port1"] = inputs["input_data_port1"] + 10.0 + gvm.get_variable("varX")
    return 3
