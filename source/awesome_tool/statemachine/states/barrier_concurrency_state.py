"""
.. module:: barrier_concurrency_state
   :platform: Unix, Windows
   :synopsis: A module to represent a barrier concurrency state for the state machine

.. moduleauthor:: Sebastian Brunner


"""

import traceback
from gtkmvc import Observable

from awesome_tool.utils import log
logger = log.get_logger(__name__)
from awesome_tool.statemachine.outcome import Outcome
from awesome_tool.statemachine.states.concurrency_state import ConcurrencyState
from awesome_tool.statemachine.enums import StateExecutionState
from awesome_tool.statemachine.execution.execution_history import ConcurrencyItem
from awesome_tool.statemachine.states.execution_state import ExecutionState
from awesome_tool.statemachine.states.container_state import ContainerState
from awesome_tool.statemachine.enums import UNIQUE_DECIDER_STATE_ID


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
                 v_checker=None, path=None, filename=None, check_path=True, decider_state=None,
                 load_from_storage=False):

        ConcurrencyState.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes,
                                  states, transitions, data_flows, start_state_id, scoped_variables, v_checker, path,
                                  filename, check_path=check_path)

        if not load_from_storage:
            self.add_state(DeciderState(state_id=UNIQUE_DECIDER_STATE_ID))

    def run(self):
        """ This defines the sequence of actions that are taken when the barrier concurrency state is executed

        :return:
        """
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
                logger.debug("Backward executing barrier concurrency state with id %s and name %s" % (self._state_id, self.name))

                history_item = self.execution_history.get_last_history_item()
                assert isinstance(history_item, ConcurrencyItem)
                # history_item.state_reference must be "self" in this case

            else:  # forward_execution
                logger.debug("Executing barrier concurrency state with id %s" % self._state_id)
                history_item = self.execution_history.add_concurrency_history_item(self, len(self.states))

            self.state_execution_status = StateExecutionState.EXECUTE_CHILDREN
            # start all threads
            history_index = 0
            for key, state in self.states.iteritems():
                # skip the decider state
                if key is not decider_state.state_id:
                    state_input = self.get_inputs_for_state(state)
                    state_output = self.create_output_dictionary_for_state(state)
                    state.input_data = state_input
                    state.output_data = state_output
                    state.start(history_item.execution_histories[history_index], self.backward_execution)
                    history_index += 1

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
                    logger.debug("Backward leave the barrier concurrency state with name %s" % (self.name))
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
            logger.debug("Executing the decider state of the concurrency barrier state")
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
            logger.error("Runtime error: {0}\n{1}".format(e, str(traceback.format_exc())))
            self.output_data["error"] = e
            self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
            return self.finalize(Outcome(-1, "aborted"))

    @Observable.observed
    def add_transition(self, from_state_id, from_outcome, to_state_id=None, to_outcome=None, transition_id=None,
                       skip_decider_id_flag=False):
        """Adds a transition to the container state.

        :param from_state_id: The source state of the transition
        :param from_outcome: The outcome of the source state to connect the transition to
        :param to_state_id: The target state of the transition
        :param to_outcome: The target outcome of a container state
        :param transition_id: An optional transition id for the new transition
        """

        transition_id = self.check_transition_id(transition_id)
        self.basic_transition_checks(from_state_id, from_outcome, to_state_id, to_outcome, transition_id)
        self.check_if_outcome_already_connected(from_state_id, from_outcome)

        if not skip_decider_id_flag:
            if from_state_id == UNIQUE_DECIDER_STATE_ID:
                if to_outcome is None:
                    raise AttributeError("Transition from the decider state must have the parent state as target!")
            else:
                if not from_outcome == -2 and not from_outcome == -1:
                    raise AttributeError("In barrier concurrency states only transitions that originates from the "
                                         "decider state or from aborted or preempted outcomes of a normal "
                                         "concurrent state are allowed!")
                else:
                    if not to_state_id == UNIQUE_DECIDER_STATE_ID and to_outcome is None:
                        raise AttributeError("Transition from the preemption and abortion outcome can only be "
                                             "connected to the decider state or the parent state.")

        self.create_transition(from_state_id, from_outcome, to_state_id, to_outcome, transition_id)
        return transition_id

    @Observable.observed
    def remove_transition(self, transition_id, force=False):
        """ Overwrite the parent class remove_transition method by checking if the user tries to delete a transition of
        a non decider state and prevents the operation in this case.

        :param transition_id: the id of the transition to remove
        :return:
        """

        transition = self.transitions[transition_id]
        print transition

        if not transition.from_state == UNIQUE_DECIDER_STATE_ID and not force:
            if not transition.from_outcome == -1 and not transition.from_outcome == -2:
                raise AttributeError("You are not allowed to remove a transition from a non-decider state!")
            else:
                ContainerState.remove_transition(self, transition_id)
        else:
            ContainerState.remove_transition(self, transition_id)


    @Observable.observed
    def add_state(self, state, storage_load=False):
        """
        Overwrite the parent class add_state method by adding the automatic transition generation for the decider_state.

        :param state: The state to be added
        :return:
        """
        ContainerState.add_state(self, state)
        if not storage_load and state.state_id is not UNIQUE_DECIDER_STATE_ID:
            # the transitions must only be created for the inital add_state call and not during each load procedure
            for o_id, o in state.outcomes.iteritems():
                if not o_id == -1 and not o_id == -2:
                    self.add_transition(state.state_id, o_id, self.states[UNIQUE_DECIDER_STATE_ID].state_id, skip_decider_id_flag=True)

    @Observable.observed
    def remove_state(self, state_id, recursive_deletion=True):
        """ Overwrite the parent class remove state method by checking if the user tries to delete the decider state

        :param state_id: the id of the state to remove
        :param recursive_deletion: a flag to indicate a recursive deletion of all substates
        :return:
        """
        if state_id == UNIQUE_DECIDER_STATE_ID:
            raise AttributeError("You are not allowed to delete the decider state.")
        else:
            ContainerState.remove_state(self, state_id, recursive_deletion)

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = ContainerState.get_container_state_yaml_dict(data)
        node = dumper.represent_mapping(cls.yaml_tag, dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node, deep=True)
        state = BarrierConcurrencyState(name=dict_representation['name'],
                                        state_id=dict_representation['state_id'],
                                        input_data_ports=dict_representation['input_data_ports'],
                                        output_data_ports=dict_representation['output_data_ports'],
                                        outcomes=dict_representation['outcomes'],
                                        states=None,
                                        transitions=dict_representation['transitions'],
                                        data_flows=dict_representation['data_flows'],
                                        scoped_variables=dict_representation['scoped_variables'],
                                        v_checker=None,
                                        path=dict_representation['path'],
                                        filename=dict_representation['filename'],
                                        check_path=False,
                                        load_from_storage=True)
        try:
            state.description = dict_representation['description']
        except (ValueError, TypeError, KeyError):
            pass
        return state


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

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = ExecutionState.get_execution_state_yaml_dict(data)
        node = dumper.represent_mapping(cls.yaml_tag, dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node, deep=True)
        name = dict_representation['name']
        state_id = dict_representation['state_id']
        input_data_ports = dict_representation['input_data_ports']
        output_data_ports = dict_representation['output_data_ports']
        outcomes = dict_representation['outcomes']
        path = dict_representation['path']
        filename = dict_representation['filename']
        state = DeciderState(name, state_id, input_data_ports, output_data_ports, outcomes, path, filename,
                             check_path=False)
        try:
            state.description = dict_representation['description']
        except (ValueError, TypeError, KeyError):
            pass
        return state