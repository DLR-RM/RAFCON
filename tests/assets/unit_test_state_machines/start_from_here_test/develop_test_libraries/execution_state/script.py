
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    self.preemptive_wait(0.1)
    outputs["output_2"] = "some manual output"
    outputs["output_3"] = [0, 1, 2, 3, 4]
    outputs["output_4"] = 7
    return "success"
