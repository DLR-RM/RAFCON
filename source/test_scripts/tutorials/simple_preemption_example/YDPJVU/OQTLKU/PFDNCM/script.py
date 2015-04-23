import time

def execute(self, inputs, outputs, gvm):
    
    count = 0
    while count < 5:
        time.sleep(1)
        if self.preempted:
            self.logger.info("preempt sleep 2")
            return -2
        else:
            count += 1
            self.logger.info("sleeped %s seconds State2" % count)
    self.logger.info("finished sleep 2")
    return 0