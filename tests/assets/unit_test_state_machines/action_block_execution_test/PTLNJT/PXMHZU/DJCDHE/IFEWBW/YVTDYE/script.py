

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Inner exit handler")
    gvm.set_variable("inner_exit_handler", True)
    prev_state = self.get_previously_executed_state()
    prev_outcome = prev_state.final_outcome
    return prev_outcome.outcome_id

