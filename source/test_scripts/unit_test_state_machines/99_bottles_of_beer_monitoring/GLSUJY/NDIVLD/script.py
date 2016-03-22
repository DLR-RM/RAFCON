

def execute(self, inputs, outputs, gvm):
    import time
    
    outputs['bottles'] = inputs['bottles'] - 1
    time.sleep(0.1)
    if not gvm.variable_exists("decimate_counter"):
        gvm.set_variable("decimate_counter", 1)
    else:
        gvm.set_variable("decimate_counter", 1 + gvm.get_variable("sing_counter"))
    
    
    return 0

