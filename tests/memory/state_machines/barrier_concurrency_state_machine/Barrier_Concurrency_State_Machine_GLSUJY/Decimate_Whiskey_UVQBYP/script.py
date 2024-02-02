def execute(self, inputs, outputs, gvm):
    import time
 
    outputs['number'] = inputs['number'] - 1
    print("number of whiskey: ", outputs['number'])
    if inputs['number'] > 0:
        return 0
    else:
        return 1
    
def backward_execute(self, inputs, outputs, gvm):
    print("Backward execute ", self.name)