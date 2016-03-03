
def execute(self, inputs, outputs, gvm):
    self.logger.debug("Hello world")
    outputs["out"] = inputs["in0"] + inputs["in1"]
    self.logger.info("Final result ist {0}".format(str(outputs["out"])))
    return 0
