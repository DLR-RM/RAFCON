"""
.. module:: barrier_concurrency_state
   :platform: Unix, Windows
   :synopsis: A module to represent a barrier concurrency state for the state machine

.. moduleauthor:: Sebastian Brunner


"""

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


class BarrierConcurrencyState(ConcurrencyState, yaml.YAMLObject):
    """ The barrier concurrency holds a list of states that are executed in parallel. It waits until all states
        finished their execution before it returns.
    """
    yaml_tag = u'!BarrierConcurrencyState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 states=None, transitions=None, data_flows=None, start_state_id=None, scoped_variables=None,
                 v_checker=None, path=None, filename=None, check_path=True):

        ConcurrencyState.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes,
                                  states, transitions, data_flows, start_state_id, scoped_variables, v_checker, path,
                                  filename, check_path=check_path)

    def run(self):
        """ This defines the sequence of actions that are taken when the barrier concurrency state is executed

        :return:
        """
        self.setup_run()

        try:

            #######################################################
            # start threads
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
                state_input = self.get_inputs_for_state(state)
                state_output = self.create_output_dictionary_for_state(state)
                state.input_data = state_input
                state.output_data = state_output
                state.start(history_item.execution_histories[history_index], self.backward_execution)
                history_index += 1

            #######################################################
            # wait for all threads to finish
            #######################################################

            for key, state in self.states.iteritems():
                state.join()
                self.add_state_execution_output_to_scoped_data(state.output_data, state)
                self.update_scoped_variables_with_output_dictionary(state.output_data, state)
                state.state_execution_status = StateExecutionState.INACTIVE

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
                    # final outcome is not important as the execution order is fixed during backward stepping
                    self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
                    if self.concurrency_queue:
                        self.concurrency_queue.put(self.state_id)
                    logger.debug("Backward leave the barrier concurrency state with name %s" % (self.name))
                    return
                else:
                    self.backward_execution = False

            self.write_output_data()
            self.check_output_data_type()

            self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE

            if self.concurrency_queue:
                self.concurrency_queue.put(self.state_id)

            # check the outcomes of all states for aborted or preempted
            # check as well if the states were stopped
            for key, state in self.states.iteritems():
                # This is the case if execution was stopped
                if state.final_outcome is None:
                    quit()
                if state.final_outcome.outcome_id == -2:
                    self.final_outcome = Outcome(-2, "preempted")
                    return
                if state.final_outcome.outcome_id == -1:
                    self.final_outcome = Outcome(-1, "aborted")
                    self.output_data["error"] = state.output_data["error"]
                    return

            if self.preempted:
                self.final_outcome = Outcome(-2, "preempted")
                return

            self.final_outcome = Outcome(0, "success")
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
                                        check_path=False)
        try:
            state.description = dict_representation['description']
        except (ValueError, TypeError, KeyError):
            pass
        return state