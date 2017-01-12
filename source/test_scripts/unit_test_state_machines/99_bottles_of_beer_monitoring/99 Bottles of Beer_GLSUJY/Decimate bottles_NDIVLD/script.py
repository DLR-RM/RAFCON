

def execute(self, inputs, outputs, gvm):
    import time
    
    outputs['bottles'] = inputs['bottles'] - 1
    time.sleep(0.01)
    if not gvm.variable_exists("decimate_counter"):
        gvm.set_variable("decimate_counter", 1)
    else:
        gvm.set_variable("decimate_counter", 1 + gvm.get_variable("decimate_counter"))
    
    
    return 0

def backward_execute(self, inputs, outputs, gvm):
    print "Backward stepping decimate state"
    gvm.set_variable("decimate_counter", gvm.get_variable("decimate_counter") - 1)
    