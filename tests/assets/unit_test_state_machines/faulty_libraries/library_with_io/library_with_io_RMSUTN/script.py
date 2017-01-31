
def execute(self, inputs, outputs, gvm):
    self.logger.debug("Set global variable x to 1")
    gvm.set_variable("x", 1)
    return 0
