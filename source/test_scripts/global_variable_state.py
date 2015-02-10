def execute(self, inputs, outputs, external_modules, gvm):
    print "Global variable test state is executing ..."
    print inputs
    print outputs
    outputs["output_data_port1"] = inputs["input_data_port1"] + 10.0 + gvm.get_variable("varX")
    return 3
