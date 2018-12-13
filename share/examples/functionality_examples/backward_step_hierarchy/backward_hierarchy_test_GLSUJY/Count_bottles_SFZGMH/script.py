from __future__ import print_function
import time

def execute(self, inputs, outputs, gvm):
    import time
    time.sleep(0.2)
    if inputs['bottles'] > 0:
        return 1
    return 0

def backward_execute(self, inputs, outputs, gvm):
    print("Backward execute ", self.name)
    time.sleep(0.2)
