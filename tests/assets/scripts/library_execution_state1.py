def execute(self, inputs, outputs, gvm):
    print "Executing library_execution_state1 ..."
    print inputs
    print outputs
    outputs["data_output_port1"] = inputs["data_input_port1"] + 10.0
    self.logger.info("outputs[data_output_port1]: {0}".format(str(outputs["data_output_port1"])))
    return 0
