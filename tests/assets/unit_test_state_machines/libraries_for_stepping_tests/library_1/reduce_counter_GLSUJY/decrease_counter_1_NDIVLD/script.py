

def execute(self, inputs, outputs, gvm):
    import time
    
    outputs['counter'] = inputs['counter'] - 1
    time.sleep(0.01)
    
    return 0

