"""
.. module:: execution_state
   :platform: Unix, Windows
   :synopsis: A module to represent a state for executing arbitrary functions

.. moduleauthor:: Sebastian Brunner


"""

import sys

from statemachine.states.state import State
from utils import log
logger = log.get_logger(__name__)

from statemachine.outcome import Outcome


class ExecutionState(State):

    """A class to represent a state for executing arbitrary functions

    This kind of state does not have any child states.
    """

    def __init__(self, name=None, state_id=None, input_keys={}, output_keys={}, outcomes={}, sm_status=None):

        State.__init__(self, name, state_id, input_keys, output_keys, outcomes, sm_status)

    def _execute(self,execute_inputs, execute_outputs):

        execute_outputs["MyFirstDataOutpuPort"] = 10.0
        #TODO: implement
        #call execute of script here
        outcome = self._outcomes[5]
        return outcome

    def run(self, *args, **kwargs):

        try:

            #initialize data structures
            input_data = kwargs["inputs"]
            output_data = kwargs["outputs"]
            if not isinstance(input_data, dict):
                raise TypeError("states must be of type dict")
            if not isinstance(output_data, dict):
                raise TypeError("states must be of type dict")

            self.check_input_data_type(input_data)

            logger.debug("Starting state with id %s" % self._state_id)
            outcome = self._execute(input_data, output_data)

            #check output data
            for key, value in self.output_data_ports.iteritems():
                #if outputs_data[key]:  # exception for None value?
                #check for primitive data types
                if not str(type(output_data[key]).__name__) == value.data_type:
                    #check for classes
                    if not isinstance(output_data[key], getattr(sys.modules[__name__], value.data_type)):
                        raise TypeError("Input of execute function must be of type %s" % str(value.data_type))
                        exit()

                kwargs["outputs"][key] = output_data[key]

            self.check_output_data_type(output_data)

            return outcome

        except RuntimeError:
            return Outcome("aborted")