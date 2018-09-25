from builtins import str
from exceptions import ZeroDivisionError

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Executing decider state")
    self.logger.debug("state-inputs: %s" % str(inputs))
    # to make decisions based on the outcome of the concurrent child states use:
    # "self.get_outcome_for_state_name(<name_of_state>) for accessing the outcome by specifying the name (not necessarily unique, first match is used) of the state
    # or self.get_outcome_for_state_id(<id_of_state>) for accessing the outcome by specifying the id (unique) of the state
    # example:
    # if self.get_outcome_for_state_name("Second").name == "success":
    #     return 0
    # here the error of the state "Second" is used to make a decision
    if isinstance(self.get_errors_for_state_name("Second"), ZeroDivisionError):
        return 1
    else:
        return 0