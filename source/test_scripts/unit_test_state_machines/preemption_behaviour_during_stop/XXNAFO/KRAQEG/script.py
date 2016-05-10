
def execute(self, inputs, outputs, gvm):
    self.logger.debug("Hello world: s3")
    gvm.set_variable("s3", 1)
    return 0
