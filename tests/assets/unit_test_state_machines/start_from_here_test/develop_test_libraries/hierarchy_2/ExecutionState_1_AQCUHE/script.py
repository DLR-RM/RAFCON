
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    self.preemptive_wait(0.4)
    gvm.set_variable("gvm_set", True)
    return "success" 
