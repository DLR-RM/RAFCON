"""
.. module:: preemptive_concurrency_state
   :platform: Unix, Windows
   :synopsis: A module to represent a preemptive concurrency state for the state machine

.. moduleauthor:: Sebastian Brunner


"""

import Queue
import yaml
import traceback

from awesome_tool.utils import log

logger = log.get_logger(__name__)
from awesome_tool.statemachine.outcome import Outcome
from awesome_tool.statemachine.states.concurrency_state import ConcurrencyState
from awesome_tool.statemachine.states.container_state import ContainerState
from awesome_tool.statemachine.enums import StateExecutionState
from awesome_tool.statemachine.enums import MethodName
from awesome_tool.statemachine.execution.execution_history import CallItem, ReturnItem, ConcurrencyItem


class PreemptiveConcurrencyState(ConcurrencyState, yaml.YAMLObject):
    """ The preemptive concurrency state has a set of substates which are started when the preemptive concurrency state
    executes. The execution of preemptive concurrency state waits for the first substate to return, preempts all other
    substates and finally returns self.

    """

    yaml_tag = u'!PreemptiveConcurrencyState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 states=None, transitions=None, data_flows=None, start_state_id=None, scoped_variables=None,
                 v_checker=None, path=None, filename=None, check_path=True):

        ConcurrencyState.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes, states,
                                  transitions, data_flows, start_state_id, scoped_variables, v_checker, path, filename,
                                  check_path=check_path)

    def run(self):
        """ This defines the sequence of actions that are taken when the preemptive concurrency state is executed

        :return:
        """
        self.setup_run()

        try:
            #######################################################
            # start threads
            #######################################################

            if self.backward_execution:
                logger.debug("Backward executing preemptive concurrency state with id %s and name %s" % (self._state_id, self.name))

                last_history_item = self.execution_history.pop_last_item()
                assert isinstance(last_history_item, ReturnItem)
                self.scoped_data = last_history_item.scoped_data

                scoped_variables_as_dict = {}
                self.get_scoped_variables_as_dict(scoped_variables_as_dict)
                self.exit(scoped_variables_as_dict, backward_execution=True)
                # do not write the output of the exit script
                # pop the remaining CallItem of the last barrier concurrency run from the history
                last_history_item = self.execution_history.pop_last_item()
                assert isinstance(last_history_item, CallItem)
                self.scoped_data = last_history_item.scoped_data

                history_item = self.execution_history.get_last_history_item()
                assert isinstance(history_item, ConcurrencyItem)
                # history_item.state_reference must be "self" in this case

            else:  # forward_execution
                logger.debug("Executing preemptive concurrency state with id %s" % self._state_id)

                # handle data for the entry script
                scoped_variables_as_dict = {}
                self.execution_history.add_call_history_item(self, MethodName.ENTRY, self)
                self.get_scoped_variables_as_dict(scoped_variables_as_dict)
                self.enter(scoped_variables_as_dict)
                self.add_enter_exit_script_output_dict_to_scoped_data(scoped_variables_as_dict)
                self.execution_history.add_return_history_item(self, MethodName.ENTRY, self)

                history_item = self.execution_history.add_concurrency_history_item(self, len(self.states))

            self.state_execution_status = StateExecutionState.EXECUTE_CHILDREN
            concurrency_queue = Queue.Queue(maxsize=0)  # infinite Queue size
            queue_ids = 0
            history_index = 0
            for key, state in self.states.iteritems():
                state.concurrency_queue = concurrency_queue
                state.concurrency_queue_id = queue_ids
                queue_ids += 1

                state_input = self.get_inputs_for_state(state)
                state_output = self.create_output_dictionary_for_state(state)
                state.input_data = state_input
                state.output_data = state_output
                state.start(history_item.execution_histories[history_index], self.backward_execution)
                history_index += 1

            #######################################################
            # wait for the first threads to finish
            #######################################################

            finished_thread_id = concurrency_queue.get()
            self.states[finished_thread_id].join()

            # in the backward executing case, only backward execute the entry function and return
            if self.states[finished_thread_id].backward_execution:
                # wait until all states finished their jobs in the backward execution case
                for key, state in self.states.iteritems():
                    state.join()
                    state.state_execution_status = StateExecutionState.INACTIVE

                # backward_execution needs to be True to signal the parent container state the backward execution
                self.backward_execution = True
                logger.debug("Backward-executing entry script of preemption concurrency state %s" % self.name)
                # pop the ConcurrencyItem as we are leaving the barrier concurrency state
                last_history_item = self.execution_history.pop_last_item()
                assert isinstance(last_history_item, ConcurrencyItem)

                last_history_item = self.execution_history.pop_last_item()
                assert isinstance(last_history_item, ReturnItem)
                self.scoped_data = last_history_item.scoped_data
                # the last_history_item is the ReturnItem from the entry function,
                # thus backward execute the entry function
                scoped_variables_as_dict = {}
                self.get_scoped_variables_as_dict(scoped_variables_as_dict)
                self.enter(scoped_variables_as_dict, backward_execution=True)

                last_history_item = self.execution_history.pop_last_item()
                assert isinstance(last_history_item, CallItem)
                self.scoped_data = last_history_item.scoped_data

                # do not write the output of the entry script
                # final outcome is not important as the execution order is fixed during backward stepping
                self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
                if self.concurrency_queue:
                    self.concurrency_queue.put(self.state_id)
                return

            else:
                self.backward_execution = False

            # preempt all child states
            for state_id, state in self.states.iteritems():
                state.recursively_preempt_states()

            for key, state in self.states.iteritems():
                state.join()
                state.concurrency_queue = None
                # add the data of all child states to the scoped data and the scoped variables
                self.add_state_execution_output_to_scoped_data(state.output_data, state)
                self.update_scoped_variables_with_output_dictionary(state.output_data, state)
                state.state_execution_status = StateExecutionState.INACTIVE

            # handle data for the exit script
            scoped_variables_as_dict = {}
            self.execution_history.add_call_history_item(self, MethodName.EXIT, self)
            self.get_scoped_variables_as_dict(scoped_variables_as_dict)
            self.exit(scoped_variables_as_dict)
            self.add_enter_exit_script_output_dict_to_scoped_data(scoped_variables_as_dict)
            self.execution_history.add_return_history_item(self, MethodName.EXIT, self)

            self.write_output_data()

            self.check_output_data_type()

            self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE

            if self.concurrency_queue:
                self.concurrency_queue.put(self.state_id)

            if self.preempted:
                self.final_outcome = Outcome(-2, "preempted")
                return

            transition = self.get_transition_for_outcome(self.states[finished_thread_id],
                                                         self.states[finished_thread_id].final_outcome)

            if transition is None:
                transition = self.handle_no_transition(self.states[finished_thread_id])
            # it the transition is still None, then the state was preempted or aborted, in this case return
            if transition is None:
                if self.final_outcome.outcome_id == -1:
                    self.output_data["error"] = RuntimeError("state aborted")
                return

            self.final_outcome = self.outcomes[transition.to_outcome]
            return

        except Exception, e:
            logger.error("Runtime error %s %s" % (e, str(traceback.format_exc())))
            self.final_outcome = Outcome(-1, "aborted")
            self.output_data["error"] = e
            self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
            return

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = ContainerState.get_container_state_yaml_dict(data)
        node = dumper.represent_mapping(cls.yaml_tag, dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node, deep=True)
        state = PreemptiveConcurrencyState(name=dict_representation['name'],
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
                                           check_path=False)
        try:
            state.description = dict_representation['description']
        except (ValueError, TypeError, KeyError):
            pass
        return state
