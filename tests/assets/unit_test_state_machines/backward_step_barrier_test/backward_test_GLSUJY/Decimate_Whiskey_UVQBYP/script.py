def execute(self, inputs, outputs, gvm):
    import time
 
    outputs['number'] = inputs['number'] - 1
    time.sleep(0.2)
    print("number of whiskey: ", outputs['number'])
 
    return 0
    
def backward_execute(self, inputs, outputs, gvm):
    print("Backward execute ", self.name)