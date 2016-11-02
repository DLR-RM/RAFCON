import time

def execute(self, inputs, outputs, gvm):
    while True:
        time.sleep(0.010)
        if self.preempted:
            print "i was preempted"
            break    
#        beer = gvm.get_variable('beers')
#        if beer >= 3:
#            break
    gvm.set_variable("whiskey",3)
    return 0
    
def backward_execute(self, inputs, outputs, gvm):
    self.logger.debug("backward step 3")
    whiskey = gvm.get_variable("whiskey")
    gvm.set_variable("whiskey", whiskey -1)
    return 0