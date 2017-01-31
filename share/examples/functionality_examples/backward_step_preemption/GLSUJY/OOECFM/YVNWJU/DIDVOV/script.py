# State "Decimate bottles"
def execute(self, inputs, outputs, gvm):
    import time
 
    outputs['number'] = inputs['number'] - 1
    time.sleep(3.5)
 
    return 0