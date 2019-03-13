from __future__ import print_function
import time

def execute(self, inputs, outputs, gvm):
    while True:
        # explicitely no preemptive_wait here!
        time.sleep(0.010)
        if self.preempted or self.parent.preempted:
            print("i was preempted")
            gvm.set_variable("whiskey",3)
            return -2  # prempted    
    #gvm.set_variable("whiskey",3)
    #return 0
    
def backward_execute(self, inputs, outputs, gvm):
    self.logger.debug("backward step 3")
    whiskey = gvm.get_variable("whiskey")
    gvm.set_variable("whiskey", whiskey -1)
    return 0