from exceptions import ZeroDivisionError

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Executing decider state")
    if isinstance(self.get_errors_for_state_name("Second"), ZeroDivisionError):
        return 1
    else:
        return 0
