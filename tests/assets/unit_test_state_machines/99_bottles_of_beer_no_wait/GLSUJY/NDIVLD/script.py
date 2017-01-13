

def execute(self, inputs, outputs, gvm):
    import time
    
    outputs['bottles'] = inputs['bottles'] - 1
    time.sleep(0.1)
    
    return 0

