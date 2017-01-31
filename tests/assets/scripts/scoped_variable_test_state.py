def execute(self, inputs, outputs, gvm):
    print "Executing scoped variable test state ..."
    print inputs
    tmp = inputs["input_data_port1"] + inputs["input_data_port2"]
    outputs["output_data_port1"] = tmp
    print outputs
    if tmp >= 42.0:
        return 0
    else:
        return 1
