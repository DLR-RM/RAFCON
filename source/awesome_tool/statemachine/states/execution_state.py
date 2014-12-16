"""
.. module:: execution_state
   :platform: Unix, Windows
   :synopsis: A module to represent a state for executing arbitrary functions

.. moduleauthor:: Sebastian Brunner


"""

from statemachine.states.state import State
from utils import log
logger = log.get_logger(__name__)
from statemachine.outcome import Outcome


class ExecutionState(State):

    """A class to represent a state for executing arbitrary functions

    This kind of state does not have any child states.
    """

    def __init__(self, name=None, state_id=None, input_keys=None, output_keys=None, outcomes=None, sm_status=None,
                 path=None, filename=None):

        State.__init__(self, name, state_id, input_keys, output_keys, outcomes, sm_status, path, filename)

    def print_state_information(self):
        """Prints information about the state

        """
        print "---  \nState information of state: %s" % self.name
        print "Id of the state: " + self.state_id
        print "---"

    def _execute(self, execute_inputs, execute_outputs):
        """Calls the custom execute function of the script.py of the state

        """
        self.script.load_and_build_module()
        outcome_id = self.script.execute(self, execute_inputs, execute_outputs)
        return self.outcomes[outcome_id]

    def run(self):
        try:

            #initialize data structures
            input_data = self.input_data
            output_data = self.output_data
            if not isinstance(input_data, dict):
                raise TypeError("states must be of type dict")
            if not isinstance(output_data, dict):
                raise TypeError("states must be of type dict")

            self.check_input_data_type(input_data)

            logger.debug("Starting state with id %s" % self._state_id)
            outcome = self._execute(input_data, output_data)

            #check output data
            self.check_output_data_type(output_data)

            if self.concurrency_queue:
                self.concurrency_queue.put(self.state_id)

            self.final_outcome = outcome
            return

        except RuntimeError:
            self.final_outcome = Outcome(1, "aborted")
            return