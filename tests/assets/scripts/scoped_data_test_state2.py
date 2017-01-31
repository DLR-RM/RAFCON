def execute(self, inputs, outputs, gvm):
    print "Executing scoped data test state 2 ..."
    print inputs
    outputs["data_output_port1"] = inputs["data_input_port1"] + 10.0
    print outputs
    return 3
