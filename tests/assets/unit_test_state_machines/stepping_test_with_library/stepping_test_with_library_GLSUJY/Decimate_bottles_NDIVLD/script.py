import time

def execute(self, inputs, outputs, gvm):
    outputs['bottles'] = inputs['bottles'] - 1
    time.sleep(0.01)
    return 0