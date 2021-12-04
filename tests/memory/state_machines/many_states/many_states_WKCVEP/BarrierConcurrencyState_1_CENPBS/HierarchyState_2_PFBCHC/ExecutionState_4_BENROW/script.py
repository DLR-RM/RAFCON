
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    self.preemptive_wait(0.01)
    return "success"
