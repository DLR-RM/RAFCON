import time

def execute(self, inputs, outputs, gvm):
    time.sleep(0.2)
    gvm.set_variable("bottles", inputs["bottles"])
    if inputs['bottles'] > 0:
        return 1
    return 0

def backward_execute(self, inputs, outputs, gvm):
    print "Backward execute ", self.name
    time.sleep(1.0)
