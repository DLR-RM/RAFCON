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

    def __init__(self, name=None, state_id=None, input_keys={}, output_keys={}, outcomes={}, sm_status=None):

        State.__init__(self, name, state_id, input_keys, output_keys, outcomes, sm_status)

    def _execute(self):
        #TODO: implement
        #call execute of script here
        outcome = self._outcomes[5]
        return outcome

    def run(self, *args, **kwargs):
        try:
            logger.debug("Starting state with id %s" % self._state_id)
            outcome = self._execute()
            return outcome

        except RuntimeError:
            return Outcome("aborted")