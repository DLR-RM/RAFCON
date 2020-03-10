# Copyright (C) 2014-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Michael Vilzmann <michael.vilzmann@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: execution_state
   :synopsis: A module to represent a state for executing arbitrary functions

"""

from builtins import str
import traceback
import sys
import os
from copy import copy, deepcopy

from gtkmvc3.observable import Observable

from rafcon.core.states.state import State
from rafcon.core.decorators import lock_state_machine
from rafcon.core.state_elements.logical_port import Outcome
from rafcon.core.script import Script
from rafcon.core.states.state import StateExecutionStatus
from rafcon.core.execution.execution_history import CallType
from rafcon.core.config import global_config

from rafcon.utils import log
logger = log.get_logger(__name__)


class ExecutionState(State):
    """A class to represent a state for executing arbitrary functions

    This kind of state does not have any child states.
    """

    yaml_tag = u'!ExecutionState'
    
    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None,
                 income=None, outcomes=None, path=None, filename=None, check_path=True, safe_init=True):
        State.__init__(self, name, state_id, input_data_ports, output_data_ports, income, outcomes, safe_init=safe_init)
        self._script = None
        self.script = Script(path, filename, parent=self)
        self.logger = log.get_logger(self.name)
        # here all persistent variables that should be available for the next state run should be stored
        self.persistent_variables = {}
        # safe_init doesn't affect the constructor yet

    def __hash__(self):
        return id(self)

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        return str(self) == str(other) and self.script_text == other.script_text

    def __copy__(self):
        input_data_ports = {key: copy(self._input_data_ports[key]) for key in self._input_data_ports.keys()}
        output_data_ports = {key: copy(self._output_data_ports[key]) for key in self._output_data_ports.keys()}
        income = copy(self._income)
        outcomes = {elem_id: copy(self._outcomes[elem_id]) for elem_id in self._outcomes.keys()}
        state = self.__class__(self.name, self.state_id, input_data_ports, output_data_ports, income, outcomes, None,
                               safe_init=False)

        state.script_text = deepcopy(self.script_text)

        state._description = deepcopy(self.description)
        state._semantic_data = deepcopy(self.semantic_data)
        state._file_system_path = self.file_system_path
        return state

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    @lock_state_machine
    def update_hash(self, obj_hash):
        super(ExecutionState, self).update_hash(obj_hash)
        obj_hash.update(self.get_object_hash_string(self.script.script))

    @classmethod
    def from_dict(cls, dictionary):
        name = dictionary['name']
        state_id = dictionary['state_id']
        input_data_ports = dictionary['input_data_ports']
        output_data_ports = dictionary['output_data_ports']
        income = dictionary.get('income', None)  # older state machine versions don't have this set
        outcomes = dictionary['outcomes']
        safe_init = global_config.get_config_value("LOAD_SM_WITH_CHECKS", True)
        state = cls(name, state_id, input_data_ports, output_data_ports, income, outcomes, safe_init=safe_init)
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
        for outcome_id, outcome in self.outcomes.items():
            if outcome.name == outcome_item:
                return self.outcomes[outcome_id]

        logger.error("Returned outcome of {0} not existing: {1}".format(self, outcome_item))
        return Outcome(-1, "aborted")

    def run(self):
        """ This defines the sequence of actions that are taken when the execution state is executed

        :return:
        """
        if self.is_root_state:
            self.execution_history.push_call_history_item(self, CallType.EXECUTE, None, self.input_data)

        logger.debug("Running {0}{1}".format(self, " (backwards)" if self.backward_execution else ""))
        if self.backward_execution:
            self.setup_backward_run()
        else:
            self.setup_run()

        try:
            outcome = self._execute(self.input_data, self.output_data, self.backward_execution)
            self.state_execution_status = StateExecutionStatus.WAIT_FOR_NEXT_STATE

            if self.backward_execution:
                # outcome handling is not required as we are in backward mode and the execution order is fixed
                result = self.finalize()
            else:
                # check output data
                self.check_output_data_type()
                result = self.finalize(outcome)

            if self.is_root_state:
                self.execution_history.push_return_history_item(self, CallType.EXECUTE, None, self.output_data)
            return result
        except Exception as e:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            formatted_exc = traceback.format_exception(exc_type, exc_value, exc_traceback)
            truncated_exc = []
            for line in formatted_exc:
                if os.path.join("rafcon", "core") not in line:
                    truncated_exc.append(line)
            logger.error("{0} had an internal error: {1}: {2}\n{3}".format(self, type(e).__name__, e,
                                                                           ''.join(truncated_exc)))
            # write error to the output_data of the state
            self.output_data["error"] = e
            self.state_execution_status = StateExecutionStatus.WAIT_FOR_NEXT_STATE
            return self.finalize(Outcome(-1, "aborted"))

#########################################################################
# Properties for all class fields that must be observed by gtkmvc3
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
