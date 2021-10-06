# State "Decimate bottles"
def execute(self, inputs, outputs, gvm):
    import time
 
    outputs['number'] = inputs['number'] - 1
 
    return 0