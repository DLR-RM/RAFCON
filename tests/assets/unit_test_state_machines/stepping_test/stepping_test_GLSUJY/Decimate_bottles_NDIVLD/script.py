from __future__ import print_function
import time

def execute(self, inputs, outputs, gvm):
    outputs['bottles'] = inputs['bottles'] - 1
    #time.sleep(0.2)
    return 0

def backward_execute(self, inputs, outputs, gvm):
    print("Backward execute ", self.name)
    time.sleep(1.0)