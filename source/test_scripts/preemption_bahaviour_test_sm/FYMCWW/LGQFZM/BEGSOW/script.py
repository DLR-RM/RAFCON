

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Hello world: s2")
    gvm.set_variable("s2", 1.0)
    return 0

