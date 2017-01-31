def execute(self, inputs, outputs, gvm):
    print "Executing first execution state ..."
    print inputs
    print outputs
    outputs["data_output_port1"] = inputs["data_input_port1"] + 10.0
    outputs["faulty_output_port"] = "string_but_should_be_float"
    # outputs["faulty_output_port"] = 1
    return 3
