from rafcon.core.states.execution_state import ExecutionState
import time

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Insert state")
    e = ExecutionState()
    self.parent.add_state(e)
    self.parent.add_transition(self.state_id, 0, e.state_id, None)
    self.parent.add_transition(e.state_id, 0, self.parent.state_id, 0)
    return 0
