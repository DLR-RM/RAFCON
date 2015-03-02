def execute(self, inputs, outputs, gvm):
    print "Executing library_execution_state1 ..."
    print inputs
    print outputs
    outputs["data_output_port1"] = inputs["data_input_port1"] + 10.0
    self.print_state_information()
    return 0
