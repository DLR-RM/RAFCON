import time

def execute(self, inputs, outputs, gvm):
    bottles = inputs['bottles']
    if bottles is None:
        return -1
        
    self.logger.info(str(bottles) + " bottles of beer on the wall, " + str(bottles) + " bottles of beer.")
    bottles -= 1
    self.logger.info("Take one down and pass it around, " + str(bottles) + " bottles of beer on the wall.")
    time.sleep(0.2)
    
    return "done"
    
def backward_execute(self, inputs, outputs, gvm):
    print "This is the first backward step test"
    time.sleep(1.0)
    return execute(self, inputs, outputs, gvm)