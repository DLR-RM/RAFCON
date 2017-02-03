
def execute(self, inputs, outputs, gvm):
    self.logger.debug("Hello world")
    outputs["out"] = 5
    return 0
