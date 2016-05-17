

def execute(self, inputs, outputs, gvm):
    self.logger.info( "Handling hierarchy abortion ... ")
    
    if "error" in inputs:
        self.logger.info(str(type(inputs["error"]).__name__) +  ": " + str(inputs["error"]))
        outputs["error_check"] = "successfull"
    
    return 0
