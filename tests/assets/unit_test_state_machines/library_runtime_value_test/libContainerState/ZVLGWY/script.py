
def execute(self, inputs, outputs, gvm):
    self.logger.info("Add all output values")
    outputs["output0"] = inputs["input0"] + inputs["input1"] + inputs["input2"] + inputs["input3"] + inputs["input4"]
    self.logger.info("Sum: {0}".format(outputs["output0"]))
    return 0
