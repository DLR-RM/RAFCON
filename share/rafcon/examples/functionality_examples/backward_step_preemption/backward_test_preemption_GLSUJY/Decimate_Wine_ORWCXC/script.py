def execute(self, inputs, outputs, gvm):
    import time
 
    input_number = inputs['number']
    if input_number is None:
        input_number = 40
    outputs['number'] = input_number - 1
    time.sleep(0.2)
    print("number of wine: ", outputs['number'])
 
    return 0

def backward_execute(self, inputs, outputs, gvm):
    print("Backward execute ", self.name)