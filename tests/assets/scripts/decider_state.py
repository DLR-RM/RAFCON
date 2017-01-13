from exceptions import ZeroDivisionError

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Executing decider state.")
    print "Data passed to the decider state: "
    print inputs
    print outputs
    print self.get_outcome_for_state_name("FirstState")
    print self.get_outcome_for_state_name("SecondState")
    print self.get_outcome_for_state_id("id_of_state_1")
    print self.get_outcome_for_state_id("id_of_state_2")

    error = self.get_errors_for_state_name("SecondState")
    print error
    if isinstance(error, ZeroDivisionError):
        return 4
    else:
        return 3