
def execute(self, inputs, outputs, gvm):
    # Takes file
    Huge_File = inputs["input_0"]
    counter = inputs["input_3"]
    
    # Cheks Iteration value
    # Either pass back or end
    if counter < 3:
        outputs["output_2"] = Huge_File
        outputs["output_1"] = counter + 1
    else:
        return "success_1"
        
    self.logger.info("Hello {}".format(self.name))
    #self.preemptive_wait(2)
    return "success"
