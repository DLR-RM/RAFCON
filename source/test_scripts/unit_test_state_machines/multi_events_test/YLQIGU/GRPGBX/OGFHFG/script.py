def print_state_events(state):
    state.logger.debug("Value of preemted event: {0}".format(str(state.preempted)))
    state.logger.debug("Value of started event: {0}".format(str(state.started)))
    state.logger.debug("Value of paused event: {0}".format(str(state.paused)))

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Hello world")
    self.logger.debug("Value of _interruption_events: {0}".format(str(self._interrupted)))
    self.logger.debug("Value of _unpause_events: {0}".format(str(self._unpaused)))
    
    gvm.set_variable("sm_status", 0)
    
    self.wait_for_interruption()
    print_state_events(self)
    if self.paused:
        gvm.set_variable("paused", True)
    
    gvm.set_variable("sm_status", 1)
    
    self.wait_for_unpause()
    print_state_events(self)
    if self.started:
        gvm.set_variable("running", True)

    gvm.set_variable("sm_status", 2)

    self.wait_for_interruption()
    print_state_events(self)
    if self.preempted:
        gvm.set_variable("preempted", True)

    return 0
