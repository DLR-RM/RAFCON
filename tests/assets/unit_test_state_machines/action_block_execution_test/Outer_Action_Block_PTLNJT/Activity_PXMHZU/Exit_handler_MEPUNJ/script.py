

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Exit handler")
    gvm.set_variable("exit_handler", True)
    prev_state = self.get_previously_executed_state()
    prev_outcome = prev_state.final_outcome
    self.logger.info("Prev outcome: {0}".format(prev_outcome))
    return prev_outcome.outcome_id

