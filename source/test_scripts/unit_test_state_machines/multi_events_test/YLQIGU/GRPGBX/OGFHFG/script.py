def print_state_events(state):
    state.logger.debug("Value of preemted event: {0}".format(str(state.preempted)))
    state.logger.debug("Value of running event: {0}".format(str(state.running)))
    state.logger.debug("Value of paused event: {0}".format(str(state.paused)))

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Hello world")
    self.logger.debug("Value of execution_events: {0}".format(str(self.execution_events)))
    
    gvm.set_variable("sm_status", 0)
    
    self.wait_for_execution_events()
    print_state_events(self)
    if self.paused:
        gvm.set_variable("paused", True)
    
    gvm.set_variable("sm_status", 1)
    
    self.wait_for_execution_events()
    print_state_events(self)
    if self.running:
        gvm.set_variable("running", True)

    gvm.set_variable("sm_status", 2)

    self.wait_for_execution_events()
    print_state_events(self)
    if self.preempted:
        gvm.set_variable("preempted", True)

    return 0
