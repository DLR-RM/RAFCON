
def execute(self, inputs, outputs, gvm):
    self.logger.info("input: {0}".format(inputs["input0"]))
    outputs["output0"] = inputs["input0"] + 10
    self.logger.info("output: {0}".format(outputs["output0"]))
    return 0