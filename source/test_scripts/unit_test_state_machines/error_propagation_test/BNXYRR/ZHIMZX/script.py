

def execute(self, inputs, outputs, gvm):
    print "Handling hierarchy abortion ... "
    
    if "error" in inputs:
        print type(inputs["error"]).__name__, ":", inputs["error"]
        outputs["error_check"] = "successfull"
    
    return 0
