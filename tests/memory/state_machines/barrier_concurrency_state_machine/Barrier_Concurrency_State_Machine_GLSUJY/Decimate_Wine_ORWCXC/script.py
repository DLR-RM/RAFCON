def execute(self, inputs, outputs, gvm):
    import time
 
    outputs['number'] = inputs['number'] - 1
    print("number of wine: ", outputs['number'])
 
    return 0

def backward_execute(self, inputs, outputs, gvm):
    print("Backward execute ", self.name)