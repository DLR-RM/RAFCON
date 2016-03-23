

def execute(self, inputs, outputs, gvm):
    import time
    time.sleep(0.01)
    
    if not gvm.variable_exists("count_counter"):
        gvm.set_variable("count_counter", 1)
    else:
        gvm.set_variable("count_counter", 1 + gvm.get_variable("count_counter"))
    
    
    if inputs['bottles'] > 0:
        return 1
    return 0

