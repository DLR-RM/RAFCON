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
# Lukas Becker <lukas.becker@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: barrier_concurrency_state
   :synopsis: A module to represent a barrier concurrency state for the state machine

"""

from builtins import str

from gtkmvc3.observable import Observable

from rafcon.core.custom_exceptions import RecoveryModeException
from rafcon.core.state_elements.logical_port import Outcome
from rafcon.core.decorators import lock_state_machine
from rafcon.core.states.concurrency_state import ConcurrencyState
from rafcon.core.states.state import StateExecutionStatus
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.container_state import ContainerState
from rafcon.core.constants import UNIQUE_DECIDER_STATE_ID
from rafcon.utils import log
from rafcon.core.config import global_config
logger = log.get_logger(__name__)


class BarrierConcurrencyState(ConcurrencyState):
    """ The barrier concurrency holds a list of states that are executed in parallel. It waits until all states
        finished their execution before it returns.

        Note: In the backward execution case the decider state does not have to be backward executed, as it only
        decides the outcome of the barrier concurrency state. In a backward execution the logic flow obviously already
        exists.

        The order of history items for the concurrency state is:
        Call - Concurrency - Return
        and for the backward case:
        Return - Concurrency - Call

        For the children of the concurrency state the history items are:
        In the forward case:
        - Call: Before calling the child
        - Return: After executing the child
        In the backward case:
        - Pop Return: Before backward executing the child
        - Pop Call: After backward executing the child

        The decider state is not considered in the backward execution case.

    """
    yaml_tag = u'!BarrierConcurrencyState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None,
                 income=None, outcomes=None, states=None, transitions=None, data_flows=None, start_state_id=None,
                 scoped_variables=None, decider_state=None, load_from_storage=False, safe_init=True):
        self.__init_running = True
        states = {} if states is None else states
        if decider_state is not None:
            if isinstance(decider_state, DeciderState):
                decider_state._state_id = UNIQUE_DECIDER_STATE_ID
                states[UNIQUE_DECIDER_STATE_ID] = decider_state
            else:
                logger.warning("Argument decider_state has to be instance of DeciderState not {}".format(decider_state))

        if not load_from_storage and UNIQUE_DECIDER_STATE_ID not in states:
            states[UNIQUE_DECIDER_STATE_ID] = DeciderState(name='Decider', state_id=UNIQUE_DECIDER_STATE_ID)

        # TODO figure out how to solve those two clinch better of copy/add state and already existing transitions #1 #2
        ConcurrencyState.__init__(self, name, state_id, input_data_ports, output_data_ports, income, outcomes, states,
                                  transitions, data_flows, start_state_id, scoped_variables, safe_init=safe_init)

        for state_id, state in self.states.items():
            if state_id != UNIQUE_DECIDER_STATE_ID:
                for outcome in self.states[state_id].outcomes.values():
                    # TODO figure out how to solve this clinch better #3
                    match = [t.from_state == state_id and t.from_outcome == outcome.outcome_id for t in self.transitions.values()]
                    if not outcome.outcome_id < 0 and not any(match):
                        try:
                            self.add_transition(from_state_id=state_id, from_outcome=outcome.outcome_id,
                                                to_state_id=UNIQUE_DECIDER_STATE_ID, to_outcome=None)
                        except (ValueError, RecoveryModeException) as e:
                            if "transition origin already connected to another transition" not in str(e):
                                logger.error("default decider state transition could not be added: {}".format(e))
                                raise
        self.__init_running = False

    def run(self):
        """ This defines the sequence of actions that are taken when the barrier concurrency state is executed

        :return:
        """
        logger.debug("Starting execution of {0}{1}".format(self, " (backwards)" if self.backward_execution else ""))
        self.setup_run()

        # data to be accessed by the decider state
        child_errors = {}
        final_outcomes_dict = {}
        decider_state = self.states[UNIQUE_DECIDER_STATE_ID]

        try:
            concurrency_history_item = self.setup_forward_or_backward_execution()
            self.start_child_states(concurrency_history_item, decider_state)

            # print("bcs1")

            #######################################################
            # wait for all child threads to finish
            #######################################################
            for history_index, state in enumerate(self.states.values()):
                # skip the decider state
                if state is not decider_state:
                    self.join_state(state, history_index, concurrency_history_item)
                    self.add_state_execution_output_to_scoped_data(state.output_data, state)
                    self.update_scoped_variables_with_output_dictionary(state.output_data, state)
                    # save the errors of the child state executions for the decider state
                    if 'error' in state.output_data:
                        child_errors[state.state_id] = (state.name, state.output_data['error'])
                    final_outcomes_dict[state.state_id] = (state.name, state.final_outcome)

            # print("bcs2")

            #######################################################
            # handle backward execution case
            #######################################################
            if self.backward_execution:
                # print("bcs2.1.")
                return self.finalize_backward_execution()
            else:
                # print("bcs2.2.")
                self.backward_execution = False

            # print("bcs3")

            #######################################################
            # execute decider state
            #######################################################
            decider_state_error = self.run_decider_state(decider_state, child_errors, final_outcomes_dict)

            # print("bcs4")

            #######################################################
            # handle no transition
            #######################################################
            transition = self.get_transition_for_outcome(decider_state, decider_state.final_outcome)
            if transition is None:
                # final outcome is set here
                transition = self.handle_no_transition(decider_state)
            # if the transition is still None, then the child_state was preempted or aborted, in this case return
            decider_state.state_execution_status = StateExecutionStatus.INACTIVE

            # print("bcs5")

            if transition is None:
                self.output_data["error"] = RuntimeError("state aborted")
            else:
                if decider_state_error:
                    self.output_data["error"] = decider_state_error
                self.final_outcome = self.outcomes[transition.to_outcome]

            # print("bcs6")

            return self.finalize_concurrency_state(self.final_outcome)

        except Exception as e:
            logger.exception("{0} had an internal error:".format(self))
            self.output_data["error"] = e
            self.state_execution_status = StateExecutionStatus.WAIT_FOR_NEXT_STATE
            return self.finalize(Outcome(-1, "aborted"))

    def run_decider_state(self, decider_state, child_errors, final_outcomes_dict):
        """ Runs the decider state of the barrier concurrency state. The decider state decides on which outcome the
        barrier concurrency is left.

        :param decider_state: the decider state of the barrier concurrency state
        :param child_errors: error of the concurrent branches
        :param final_outcomes_dict: dictionary of all outcomes of the concurrent branches
        :return:
        """
        decider_state.state_execution_status = StateExecutionStatus.ACTIVE
        # forward the decider specific data
        decider_state.child_errors = child_errors
        decider_state.final_outcomes_dict = final_outcomes_dict
        # standard state execution
        decider_state.input_data = self.get_inputs_for_state(decider_state)
        decider_state.output_data = self.create_output_dictionary_for_state(decider_state)
        decider_state.start(self.execution_history, backward_execution=False)
        decider_state.join()
        decider_state_error = None
        if decider_state.final_outcome.outcome_id == -1:
            if 'error' in decider_state.output_data:
                decider_state_error = decider_state.output_data['error']
        # standard output data processing
        self.add_state_execution_output_to_scoped_data(decider_state.output_data, decider_state)
        self.update_scoped_variables_with_output_dictionary(decider_state.output_data, decider_state)
        return decider_state_error

    def _check_transition_validity(self, check_transition):
        """ Transition of BarrierConcurrencyStates must least fulfill the condition of a ContainerState.
        Start transitions are forbidden in the ConcurrencyState.

        :param check_transition: the transition to check for validity
        :return:
        """
        valid, message = super(BarrierConcurrencyState, self)._check_transition_validity(check_transition)
        if not valid:
            return False, message

        # Only the following transitions are allowed in barrier concurrency states:
        # - Transitions from the decider state to the parent state\n"
        # - Transitions from not-decider states to the decider state\n"
        # - Transitions from not_decider states from aborted/preempted outcomes to the
        #   aborted/preempted outcome of the parent

        from_state_id = check_transition.from_state
        to_state_id = check_transition.to_state
        from_outcome_id = check_transition.from_outcome
        to_outcome_id = check_transition.to_outcome

        if from_state_id == UNIQUE_DECIDER_STATE_ID:
            if to_state_id != self.state_id:
                return False, "Transition from the decider state must go to the parent state"
        else:
            if to_state_id != UNIQUE_DECIDER_STATE_ID:
                if from_outcome_id not in [-2, -1] or to_outcome_id not in [-2, -1]:
                    return False, "Transition from this state must go to the decider state. The only exception are " \
                                  "transition from aborted/preempted to the parent aborted/preempted outcomes"

        return True, message

    @lock_state_machine
    def add_state(self, state, storage_load=False):
        """Overwrite the parent class add_state method

         Add automatic transition generation for the decider_state.

        :param state: The state to be added
        :return:
        """
        state_id = super(BarrierConcurrencyState, self).add_state(state)
        if not storage_load and not self.__init_running and not state.state_id == UNIQUE_DECIDER_STATE_ID:
            # the transitions must only be created for the initial add_state call and not during each load procedure
            for o_id, o in list(state.outcomes.items()):
                if not o_id == -1 and not o_id == -2:
                    self.add_transition(state.state_id, o_id, self.states[UNIQUE_DECIDER_STATE_ID].state_id, None)
        return state_id

    @ContainerState.states.setter
    @lock_state_machine
    @Observable.observed
    def states(self, states):
        """ Overwrite the setter of the container state base class as special handling for the decider state is needed.

        :param states: the dictionary of new states
        :raises exceptions.TypeError: if the states parameter is not of type dict
        """
        # First safely remove all existing states (recursively!), as they will be replaced
        state_ids = list(self.states.keys())
        for state_id in state_ids:
            # Do not remove decider state, if teh new list of states doesn't contain an alternative one
            if state_id == UNIQUE_DECIDER_STATE_ID and UNIQUE_DECIDER_STATE_ID not in states:
                continue
            self.remove_state(state_id)
        if states is not None:
            if not isinstance(states, dict):
                raise TypeError("states must be of type dict")
            # Ensure that the decider state is added first, as transition to this states will automatically be
            # created when adding further states
            decider_state = states.pop(UNIQUE_DECIDER_STATE_ID, None)
            if decider_state is not None:
                self.add_state(decider_state)
            for state in states.values():
                self.add_state(state)

    def remove_state(self, state_id, recursive=True, force=False, destroy=True):
        """ Overwrite the parent class remove state method by checking if the user tries to delete the decider state

        :param state_id: the id of the state to remove
        :param recursive: a flag to indicate a recursive disassembling of all substates
        :param force: a flag to indicate forcefully deletion of all states (important of the decider state in the
                barrier concurrency state)
        :param destroy: a flag which indicates if the state should not only be disconnected from the state but also
                destroyed, including all its state elements
        :raises exceptions.AttributeError: if the state_id parameter is the decider state
        """
        if state_id == UNIQUE_DECIDER_STATE_ID and force is False:
            raise AttributeError("You are not allowed to delete the decider state.")
        else:
            return ContainerState.remove_state(self, state_id, recursive=recursive, force=force, destroy=destroy)

    @classmethod
    def from_dict(cls, dictionary):
        states = None if 'states' not in dictionary else dictionary['states']
        transitions = dictionary['transitions']
        data_flows = dictionary['data_flows']
        safe_init = global_config.get_config_value("LOAD_SM_WITH_CHECKS", True)
        state = cls(name=dictionary['name'],
                    state_id=dictionary['state_id'],
                    input_data_ports=dictionary['input_data_ports'],
                    output_data_ports=dictionary['output_data_ports'],
                    outcomes=dictionary['outcomes'],
                    states=None,
                    transitions=transitions if states else None,
                    data_flows=data_flows if states else None,
                    scoped_variables=dictionary['scoped_variables'],
                    load_from_storage=True,
                    safe_init=safe_init)
        try:
            state.description = dictionary['description']
        except (TypeError, KeyError):  # (Very) old state machines do not have a description field
            import traceback
            formatted_lines = traceback.format_exc().splitlines()
            logger.warning("Erroneous description for state '{1}': {0}".format(formatted_lines[-1], dictionary['name']))

        if states:
            return state
        else:
            return state, dictionary['transitions'], dictionary['data_flows']


class DeciderState(ExecutionState):
    """A class to represent a state for deciding the exit of barrier concurrency state.

    This type of ExecutionState has initial always the UNIQUE_DECIDER_STATE_ID.

    """

    yaml_tag = u'!DeciderState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, income=None,
                 outcomes=None, path=None, filename=None, safe_init=True):

        if state_id is None:
            state_id = UNIQUE_DECIDER_STATE_ID
        ExecutionState.__init__(self, name, state_id, input_data_ports, output_data_ports, income, outcomes, path,
                                filename, safe_init=safe_init)

        self.child_errors = {}
        self.final_outcomes_dict = {}

    def get_outcome_for_state_name(self, name):
        """ Returns the final outcome of the child state specified by name.

        Note: This is utility function that is used by the programmer to make a decision based on the final outcome
        of its child states. A state is not uniquely specified by the name, but as the programmer normally does not want
        to use state-ids in his code this utility function was defined.

        :param name: The name of the state to get the final outcome for.
        :return:
        """
        return_value = None
        for state_id, name_outcome_tuple in self.final_outcomes_dict.items():
            if name_outcome_tuple[0] == name:
                return_value = name_outcome_tuple[1]
                break
        return return_value

    def get_outcome_for_state_id(self, state_id):
        """ Returns the final outcome of the child state specified by the state_id.

        :param state_id: The id of the state to get the final outcome for.
        :return:
        """
        return_value = None
        for s_id, name_outcome_tuple in self.final_outcomes_dict.items():
            if s_id == state_id:
                return_value = name_outcome_tuple[1]
                break
        return return_value

    def get_errors_for_state_name(self, name):
        """ Returns the error message of the child state specified by name.

        Note: This is utility function that is used by the programmer to make a decision based on the final outcome
        of its child states. A state is not uniquely specified by the name, but as the programmer normally does not want
        to use state-ids in his code this utility function was defined.

        :param name: The name of the state to get the error message for
        :return:
        """
        return_value = None
        for state_id, name_outcome_tuple in self.child_errors.items():
            if name_outcome_tuple[0] == name:
                return_value = name_outcome_tuple[1]
                break
        return return_value
