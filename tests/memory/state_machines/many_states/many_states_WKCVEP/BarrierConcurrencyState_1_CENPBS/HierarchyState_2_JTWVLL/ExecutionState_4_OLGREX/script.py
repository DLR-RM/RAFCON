
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    
    outputs["output_0"] = inputs["inout_0"]
    outputs["output_1"] = inputs["input_1"]
    outputs["output_2"] = inputs["input_2"]
    outputs["output_3"] = inputs["input_3"]
    outputs["output_4"] = inputs["input_4"]
    
    self.preemptive_wait(0.01)
    return "success"
