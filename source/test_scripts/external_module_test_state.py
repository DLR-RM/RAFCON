def execute(self, inputs, outputs, external_modules, gvm):
    print "External module test is executing... "
    print inputs
    print outputs
    outputs["output_data_port1"] = external_modules["em1"].instance.custom_function_with_parameters(10.0, inputs["input_data_port1"])
    self.print_state_information()
    return 0
