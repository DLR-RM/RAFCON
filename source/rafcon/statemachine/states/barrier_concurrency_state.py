"""
.. module:: barrier_concurrency_state
   :platform: Unix, Windows
   :synopsis: A module to represent a barrier concurrency state for the state machine

.. moduleauthor:: Sebastian Brunner


"""

import traceback

from gtkmvc import Observable

from rafcon.utils import log

logger = log.get_logger(__name__)
from rafcon.statemachine.state_elements.outcome import Outcome
from rafcon.statemachine.states.concurrency_state import ConcurrencyState
from rafcon.statemachine.enums import StateExecutionState
from rafcon.statemachine.execution.execution_history import ConcurrencyItem
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.container_state import ContainerState
from rafcon.statemachine.enums import UNIQUE_DECIDER_STATE_ID


class BarrierConcurrencyState(ConcurrencyState):
    """ The barrier concurrency holds a list of states that are executed in parallel. It waits until all states
        finished their execution before it returns.

        Note: In the backward execution case the decider state does not have to be backward executed, as it only
        decides the outcome of the barrier concurrency state. In a backward execution the logic flow obviously already
        exists.

    """
    yaml_tag = u'!BarrierConcurrencyState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 states=None, transitions=None, data_flows=None, start_state_id=None, scoped_variables=None,
                 v_checker=None, decider_state=None, load_from_storage=False):

        if not load_from_storage:
            if states is not None and UNIQUE_DECIDER_STATE_ID not in states:
                states[UNIQUE_DECIDER_STATE_ID] = (DeciderState(name='Decider', state_id=UNIQUE_DECIDER_STATE_ID))

        ConcurrencyState.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes,
                                  states, transitions, data_flows, start_state_id, scoped_variables, v_checker)

        if not load_from_storage and UNIQUE_DECIDER_STATE_ID not in self.states:
            self.add_state(DeciderState(name='Decider', state_id=UNIQUE_DECIDER_STATE_ID))

        for state_id, state in self.states.iteritems():
            if not state_id == UNIQUE_DECIDER_STATE_ID:
                for outcome in self.states[state_id].outcomes.values():
                    if not outcome.outcome_id < 0:
                        self.add_transition(from_state_id=state_id, from_outcome=outcome.outcome_id,
                                            to_state_id=UNIQUE_DECIDER_STATE_ID, to_outcome=None)

    def run(self):
        """ This defines the sequence of actions that are taken when the barrier concurrency state is executed

        :return:
        """
        logger.debug("Starting execution of {0}{1}".format(self, " (backwards)" if self.backward_execution else ""))
        self.setup_run()
        # data to be accessed by the decider
        child_errors = {}
        final_outcomes_dict = {}
        decider_state = self.states[UNIQUE_DECIDER_STATE_ID]

        try:

            #######################################################
            # start child threads
            #######################################################

            if self.backward_execution:
                history_item = self.execution_history.get_last_history_item()
                assert isinstance(history_item, ConcurrencyItem)
                # history_item.state_reference must be "self" in this case

            else:  # forward_execution
                history_item = self.execution_history.add_concurrency_history_item(self, len(self.states))

            self.state_execution_status = StateExecutionState.EXECUTE_CHILDREN
            # start all threads
            for key, state in self.states.iteritems():
                # skip the decider state
                if key is not decider_state.state_id:
                    state_input = self.get_inputs_for_state(state)
                    state_output = self.create_output_dictionary_for_state(state)
                    state.input_data = state_input
                    state.output_data = state_output

            for history_index, state in enumerate(self.states.itervalues()):
                state.start(history_item.execution_histories[history_index], self.backward_execution)

            #######################################################
            # wait for all child threads to finish
            #######################################################

            for key, state in self.states.iteritems():
                # skip the decider state
                if key is not decider_state.state_id:
                    state.join()
                    self.add_state_execution_output_to_scoped_data(state.output_data, state)
                    self.update_scoped_variables_with_output_dictionary(state.output_data, state)
                    state.state_execution_status = StateExecutionState.INACTIVE
                    # save the errors of the child state executions
                    if 'error' in state.output_data:
                        child_errors[state.state_id] = (state.name, state.output_data['error'])
                    final_outcomes_dict[state.state_id] = (state.name, state.final_outcome)

            # in the backward executing case, only backward execute the entry function and return
            if len(self.states) > 0:
                first_state = self.states.itervalues().next()
                if first_state.backward_execution:
                    # backward_execution needs to be True to signal the parent container state the backward execution
                    self.backward_execution = True
                    # pop the ConcurrencyItem as we are leaving the barrier concurrency state
                    last_history_item = self.execution_history.pop_last_item()
                    assert isinstance(last_history_item, ConcurrencyItem)

                    # do not write the output of the entry script
                    self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
                    return self.finalize()
                else:
                    self.backward_execution = False

            #######################################################
            # execute decider state
            #######################################################

            decider_state.state_execution_status = StateExecutionState.ACTIVE
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

            # transition calculation
            transition = self.get_transition_for_outcome(decider_state, decider_state.final_outcome)

            if transition is None:
                transition = self.handle_no_transition(decider_state)
            # it the transition is still None, then the child_state was preempted or aborted, in this case return
            decider_state.state_execution_status = StateExecutionState.INACTIVE
            if transition is None:
                # this is the case if the user stops the sm execution, this will be caught at the end of the run method
                raise RuntimeError("decider_state stopped")

            outcome = self.outcomes[transition.to_outcome]

            #######################################################
            # decider execution finished
            #######################################################

            self.write_output_data()
            self.check_output_data_type()
            self.output_data['error'] = decider_state_error

            self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE

            if self.preempted:
                outcome = Outcome(-2, "preempted")

            return self.finalize(outcome)

        except Exception, e:
            logger.error("{0} had an internal error: {1}\n{2}".format(self, str(e), str(traceback.format_exc())))
            self.output_data["error"] = e
            self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
            return self.finalize(Outcome(-1, "aborted"))

    def _check_transition_validity(self, check_transition):
        # Transition of BarrierConcurrencyStates must least fulfill the condition of a ContainerState
        # Start transitions are already forbidden in the ConcurrencyState
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

    # @Observable.observed
    # def remove_transition(self, transition_id, force=False):
    #     """ Overwrite the parent class remove_transition method by checking if the user tries to delete a transition of
    #     a non decider state and prevents the operation in this case.
    #
    #     :param transition_id: the id of the transition to remove
    #     :return:
    #     """
    #
    #     transition = self.transitions[transition_id]
    #
    #     if transition.to_state == UNIQUE_DECIDER_STATE_ID and not force:
    #         raise ValueError("Transitions to the decider state cannot be removed")
    #
    #     ContainerState.remove_transition(self, transition_id)

    @Observable.observed
    def add_state(self, state, storage_load=False):
        """
        Overwrite the parent class add_state method by adding the automatic transition generation for the decider_state.

        :param state: The state to be added
        :return:
        """
        state_id = ContainerState.add_state(self, state)
        if not storage_load and state.state_id is not UNIQUE_DECIDER_STATE_ID:
            # the transitions must only be created for the initial add_state call and not during each load procedure
            for o_id, o in state.outcomes.iteritems():
                if not o_id == -1 and not o_id == -2:
                    self.add_transition(state.state_id, o_id, self.states[UNIQUE_DECIDER_STATE_ID].state_id, None)
        return state_id

    @ContainerState.states.setter
    @Observable.observed
    def states(self, states):
        # First safely remove all existing states (recursively!), as they will be replaced
        state_ids = self.states.keys()
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
            for state in states.itervalues():
                self.add_state(state)

    @Observable.observed
    def remove_state(self, state_id, recursive_deletion=True, force=False):
        """ Overwrite the parent class remove state method by checking if the user tries to delete the decider state

        :param state_id: the id of the state to remove
        :param recursive_deletion: a flag to indicate a recursive deletion of all substates
        :return:
        """
        if state_id == UNIQUE_DECIDER_STATE_ID and force is False:
            raise AttributeError("You are not allowed to delete the decider state.")
        else:
            ContainerState.remove_state(self, state_id, recursive_deletion)

    @classmethod
    def from_dict(cls, dictionary):
        states = None if 'states' not in dictionary else dictionary['states']
        transitions = dictionary['transitions']
        data_flows = dictionary['data_flows']
        state = cls(name=dictionary['name'],
                    state_id=dictionary['state_id'],
                    input_data_ports=dictionary['input_data_ports'],
                    output_data_ports=dictionary['output_data_ports'],
                    outcomes=dictionary['outcomes'],
                    states=None,
                    transitions=transitions if states else None,
                    data_flows=data_flows if states else None,
                    scoped_variables=dictionary['scoped_variables'],
                    v_checker=None,
                    load_from_storage=True)
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

    """A class to represent a state for deciding the exit of barrier concurrency state

    """

    yaml_tag = u'!DeciderState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 path=None, filename=None, check_path=True):

        ExecutionState.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes, path,
                                filename, check_path)

        self.child_errors = {}
        self.final_outcomes_dict = {}

    def get_outcome_for_state_name(self, name):
        """
        Returns the final outcome of the child state specified by name.

        Note: This is utility function that is used by the programmer to make a decision based on the final outcome
        of its child states. A state is not uniquely specified by the name, but as the programmer normally does not want
        to use state-ids in his code this utility function was defined.

        :param name: The name of the state to get the final outcome for.
        :return:
        """
        return_value = None
        for state_id, name_outcome_tuple in self.final_outcomes_dict.iteritems():
            if name_outcome_tuple[0] == name:
                return_value = name_outcome_tuple[1]
                break
        return return_value

    def get_outcome_for_state_id(self, state_id):
        """
        Returns the final outcome of the child state specified by the state_id.

        :param state_id: The id of the state to get the final outcome for.
        :return:
        """
        return_value = None
        for s_id, name_outcome_tuple in self.final_outcomes_dict.iteritems():
            if s_id == state_id:
                return_value = name_outcome_tuple[1]
                break
        return return_value

    def get_errors_for_state_name(self, name):
        """
        Returns the error message of the child state specified by name.

        Note: This is utility function that is used by the programmer to make a decision based on the final outcome
        of its child states. A state is not uniquely specified by the name, but as the programmer normally does not want
        to use state-ids in his code this utility function was defined.

        :param name: The name of the state to get the error message for
        :return:
        """
        return_value = None
        for state_id, name_outcome_tuple in self.child_errors.iteritems():
            if name_outcome_tuple[0] == name:
                return_value = name_outcome_tuple[1]
                break
        return return_value