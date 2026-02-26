
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    self.preemptive_wait(1)
    outputs["output_2"] = 44
    return "success"
