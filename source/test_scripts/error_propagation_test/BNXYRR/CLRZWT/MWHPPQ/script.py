

def execute(self, inputs, outputs, gvm):
    print "Handling execution abortion ... "
    
    if "error" in inputs:
        print inputs["error"]
    
    return 0

