"""
.. module:: execution_state
   :platform: Unix, Windows
   :synopsis: A module to represent a state for executing arbitrary functions

.. moduleauthor:: Sebastian Brunner


"""

import traceback

from gtkmvc import Observable
from rafcon.statemachine.states.state import State
from rafcon.utils import log
logger = log.get_logger(__name__)
from rafcon.statemachine.outcome import Outcome
from rafcon.statemachine.script import Script
from rafcon.statemachine.enums import StateExecutionState


class ExecutionState(State):

    """A class to represent a state for executing arbitrary functions

    This kind of state does not have any child states.
    """

    yaml_tag = u'!ExecutionState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 path=None, filename=None, check_path=True):

        State.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes)
        self._script = None
        self.script = Script(path, filename, check_path=check_path, state=self)
        self.logger = log.get_logger(self.name)
        # here all persistent variables that should be available for the next state run should be stored
        self.persistent_variables = {}

    @classmethod
    def from_dict(cls, dictionary):
        name = dictionary['name']
        state_id = dictionary['state_id']
        input_data_ports = dictionary['input_data_ports']
        output_data_ports = dictionary['output_data_ports']
        outcomes = dictionary['outcomes']
        state = cls(name, state_id, input_data_ports, output_data_ports, outcomes, check_path=False)
        try:
            state.description = dictionary['description']
        except TypeError:
            pass
        return state

    def _execute(self, execute_inputs, execute_outputs, backward_execution=False):
        """Calls the custom execute function of the script.py of the state

        """
        self.script.build_module()

        outcome_item = self.script.execute(self, execute_inputs, execute_outputs, backward_execution)

        # in the case of backward execution the outcome is not relevant
        if backward_execution:
            return

        # If the state was preempted, the state must be left on the preempted outcome
        if self.preempted:
            return Outcome(-2, "preempted")

        # Outcome id was returned
        if outcome_item in self.outcomes:
            return self.outcomes[outcome_item]

        # Outcome name was returned
        for outcome_id, outcome in self.outcomes.iteritems():
            if outcome.name == outcome_item:
                return self.outcomes[outcome_id]

        logger.error("Returned outcome for execution state '{0}' not existing: {1}".format(self.name, outcome_item))
        return Outcome(-1, "aborted")

    def run(self):
        """ This defines the sequence of actions that are taken when the execution state is executed

        :return:
        """
        if self.backward_execution:
            self.setup_backward_run()
        else:
            self.setup_run()
        try:

            if self.backward_execution:
                logger.debug("Backward executing state with id %s and name %s" % (self._state_id, self.name))
                self._execute(self.input_data, self.output_data, backward_execution=True)
                # outcome handling is not required as we are in backward mode and the execution order is fixed
                self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
                logger.debug("Finished backward executing state with id %s and name %s" % (self._state_id, self.name))
                return self.finalize()

            else:
                logger.debug("Executing state with id %s and name %s" % (self._state_id, self.name))
                outcome = self._execute(self.input_data, self.output_data)

                self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
                # check output data
                self.check_output_data_type()

                return self.finalize(outcome)

        except Exception, e:
            logger.error("State {0} had an internal error: {1}\n{2}".format(self.name,
                                                                            str(e), str(traceback.format_exc())))
            # write error to the output_data of the state
            self.output_data["error"] = e
            self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
            return self.finalize(Outcome(-1, "aborted"))

#########################################################################
# Properties for all class fields that must be observed by gtkmvc
#########################################################################

    @State.name.setter
    @Observable.observed
    def name(self, name):
        State.name.fset(self, name)
        self.logger = log.get_logger(self.name)

    @property
    def script(self):
        """Property for the _script field

        """
        return self._script

    @script.setter
    @Observable.observed
    def script(self, script):
        if not isinstance(script, Script):
            raise TypeError("script must be of type Script")
        self._script = script

    @Observable.observed
    def set_script_text(self, new_text):
        """
        Sets the text of the script. This function can be overridden to prevent setting the script under certain
        circumstances.
        :param new_text: The new text to replace to old text with.
        :return: Returns True if the script was successfully set.
        """
        self.script.script = new_text
        return True
