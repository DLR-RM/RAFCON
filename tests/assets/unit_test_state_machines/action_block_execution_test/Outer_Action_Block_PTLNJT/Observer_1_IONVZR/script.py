

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Observer 1 start")
    wait_time = float(gvm.get_variable("wait_observer_1", default=10))
    self.logger.debug("Waiting for {0} s".format(wait_time))
    self.preemptive_wait(wait_time)
    
    if gvm.get_variable("observer_1_abort", default=False):
        return "aborted"
        
    if gvm.get_variable("observer_1_exception", default=False):
         a = 1 / 0
         
    if not self.preempted:
        gvm.set_variable("observer_1_finish", True)
        
    self.logger.info("Observer 1 stops, preempted: {0}".format(self.preempted))
    return 0

