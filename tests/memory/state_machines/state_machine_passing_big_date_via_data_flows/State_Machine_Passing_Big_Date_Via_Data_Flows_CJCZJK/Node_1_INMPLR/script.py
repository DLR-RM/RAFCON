
def execute(self, inputs, outputs, gvm):
    #Takes huge file from input and passes it to next
    Huge_Dict = inputs["input_0"]
    outputs["output_1"] = Huge_Dict
    self.logger.info("Hello {}".format(self.name))
    #self.preemptive_wait(2)
    return "success"
