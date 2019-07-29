from __future__ import print_function
# State "Decimate bottles"
def execute(self, inputs, outputs, gvm):
    import time
 
    outputs['number'] = inputs['number'] - 1
    
    while True:
        print("State ", self.name, " waits for beeing preempted")
        time.sleep(1.0)
        if self.preempted:
            return 0
 
    return 0