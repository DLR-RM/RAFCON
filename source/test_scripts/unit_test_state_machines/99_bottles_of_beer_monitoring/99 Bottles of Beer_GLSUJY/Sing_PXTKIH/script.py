

def execute(self, inputs, outputs, gvm):
    import time
    
    bottles = inputs['bottles']
    if bottles is None:
        return -1
        
    self.logger.info(str(bottles) + " bottles of beer on the wall, " + str(bottles) + " bottles of beer.")
    bottles -= 1
    self.logger.info("Take one down and pass it around, " + str(bottles) + " bottles of beer on the wall.")
    time.sleep(0.01)
    if not gvm.variable_exists("sing_counter"):
        gvm.set_variable("sing_counter", 1)
    else:
        gvm.set_variable("sing_counter", 1 + gvm.get_variable("sing_counter"))
    
    return "done"
    
def backward_execute(self, inputs, outputs, gvm):
    print "Backward stepping sing state"
    gvm.set_variable("sing_counter", gvm.get_variable("sing_counter") - 1)
    
    