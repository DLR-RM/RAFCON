"""
.. module:: execution_state
   :platform: Unix, Windows
   :synopsis: A module to represent a state for executing arbitrary functions

.. moduleauthor:: Sebastian Brunner


"""

import traceback
import sys
import os
from copy import copy, deepcopy

from gtkmvc import Observable

from rafcon.core.states.state import State
from rafcon.core.decorators import lock_state_machine
from rafcon.core.state_elements.outcome import Outcome
from rafcon.core.script import Script
from rafcon.core.states.state import StateExecutionStatus

from rafcon.utils import log
logger = log.get_logger(__name__)


class ExecutionState(State):
    """A class to represent a state for executing arbitrary functions

    This kind of state does not have any child states.
    """

    yaml_tag = u'!ExecutionState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 path=None, filename=None, check_path=True):

        State.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes)
        self._script = None
        self.script = Script(path, filename, check_path=check_path, parent=self)
        self.logger = log.get_logger(self.name)
        # here all persistent variables that should be available for the next state run should be stored
        self.persistent_variables = {}

    def __eq__(self, other):
        # logger.info("compare method \n\t\t\t{0} \n\t\t\t{1}\n{2}\n{3}".format(self, other, self.script_text, other.script_text))
        if not isinstance(other, self.__class__):
            return False
        return str(self) == str(other) and self.script_text == other.script_text

    def __copy__(self):
        input_data_ports = {elem_id: copy(elem) for elem_id, elem in self._input_data_ports.iteritems()}
        output_data_ports = {elem_id: copy(elem) for elem_id, elem in self._output_data_ports.iteritems()}
        outcomes = {elem_id: copy(elem) for elem_id, elem in self._outcomes.iteritems()}
        state = self.__class__(self.name, self.state_id, input_data_ports, output_data_ports, outcomes, None)
        state.script_text = deepcopy(self.script_text)
        state.description = deepcopy(self.description)
        return state

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    @lock_state_machine
    def update_hash(self, obj_hash):
        super(ExecutionState, self).update_hash(obj_hash)
        obj_hash.update(self.script.script)

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
        except (TypeError, KeyError):  # (Very) old state machines do not have a description field
            import traceback
            formatted_lines = traceback.format_exc().splitlines()
            logger.warning("Erroneous description for state '{1}': {0}".format(formatted_lines[-1], dictionary['name']))
        return state

    def _execute(self, execute_inputs, execute_outputs, backward_execution=False):
        """Calls the custom execute function of the script.py of the state

        """
        self._script.build_module()

        outcome_item = self._script.execute(self, execute_inputs, execute_outputs, backward_execution)

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

        logger.error("Returned outcome of {0} not existing: {1}".format(self, outcome_item))
        return Outcome(-1, "aborted")

    def run(self):
        """ This defines the sequence of actions that are taken when the execution state is executed

        :return:
        """
        logger.debug("Running {0}{1}".format(self, " (backwards)" if self.backward_execution else ""))
        if self.backward_execution:
            self.setup_backward_run()
        else:
            self.setup_run()

        try:

            if self.backward_execution:
                self._execute(self.input_data, self.output_data, backward_execution=True)
                # outcome handling is not required as we are in backward mode and the execution order is fixed
                self.state_execution_status = StateExecutionStatus.WAIT_FOR_NEXT_STATE
                return self.finalize()

            else:
                outcome = self._execute(self.input_data, self.output_data)

                self.state_execution_status = StateExecutionStatus.WAIT_FOR_NEXT_STATE
                # check output data
                self.check_output_data_type()

                return self.finalize(outcome)

        except Exception as e:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            formatted_exc = traceback.format_exception(exc_type, exc_value, exc_traceback)
            truncated_exc = []
            for line in formatted_exc:
                if os.path.join("rafcon", "statemachine") not in line:
                    truncated_exc.append(line)
            logger.error("{0} had an internal error: {1}: {2}\n{3}".format(self, type(e).__name__, e,
                                                                           ''.join(truncated_exc)))
            # write error to the output_data of the state
            self.output_data["error"] = e
            self.state_execution_status = StateExecutionStatus.WAIT_FOR_NEXT_STATE
            return self.finalize(Outcome(-1, "aborted"))

#########################################################################
# Properties for all class fields that must be observed by gtkmvc
#########################################################################

    @State.name.setter
    @lock_state_machine
    def name(self, name):
        # Important: fset is calling the setter of the property "name" of the State base class
        # this will also trigger the name validity checks defined in the base class setter method
        # and its decorator function of the observable pattern
        State.name.fset(self, name)
        self.logger = log.get_logger(self.name)

    @property
    def script(self):
        """Returns the property for the _script field.

        """
        return self._script

    @script.setter
    @lock_state_machine
    @Observable.observed
    def script(self, script):
        if script.parent is not self:
            raise AttributeError("The script of a ExecutionState has to reference the state it-self.")
        self._script = script

    @property
    def script_text(self):
        return self._script.script

    @script_text.setter
    @lock_state_machine
    @Observable.observed
    def script_text(self, text):
        self._script.script = text
