import time

def execute(self, inputs, outputs, gvm):
    gvm.set_variable("whiskey", 3)
    while True:
        # explicitely no preemptive_wait here!
        time.sleep(0.010)
        if self.preempted or self.parent.preempted:
            print("i was preempted")
            gvm.set_variable("whiskey", 4)
            return -2  # prempted    
    #gvm.set_variable("whiskey",3)
    #return 0
    
def backward_execute(self, inputs, outputs, gvm):
    self.logger.debug("backward step 3")
    whiskey = gvm.get_variable("whiskey")
    # -2 because it was increased two times above
    gvm.set_variable("whiskey", whiskey - 2)
    return 0