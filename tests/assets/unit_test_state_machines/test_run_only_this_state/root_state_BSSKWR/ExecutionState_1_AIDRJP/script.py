
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    gvm.set_variable("test_value", 2)
    return "success"
