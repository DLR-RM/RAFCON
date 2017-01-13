

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Inner Observer 2 start")
    wait_time = float(gvm.get_variable("wait_inner_observer_2", default=10))
    self.logger.debug("Waiting for {0} s".format(wait_time))
    self.preemptive_wait(wait_time)
    
    if not self.preempted:
        gvm.set_variable("inner_observer_2_finish", True)
        
    self.logger.info("Inner Observer 2 stops, preempted: {0}".format(self.preempted))
    
    return 0

