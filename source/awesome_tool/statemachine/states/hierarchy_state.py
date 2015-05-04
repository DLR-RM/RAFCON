"""
.. module:: hierarchy_state
   :platform: Unix, Windows
   :synopsis: A module to represent a hierarchy state for the state machine

.. moduleauthor:: Sebastian Brunner


"""
import yaml
import traceback

from awesome_tool.statemachine.states.container_state import ContainerState
from awesome_tool.utils import log
logger = log.get_logger(__name__)
from awesome_tool.statemachine.outcome import Outcome
from awesome_tool.statemachine.enums import StateExecutionState
import awesome_tool.statemachine.singleton as singleton
from awesome_tool.statemachine.enums import MethodName
from awesome_tool.statemachine.execution.execution_history import CallItem, ReturnItem


class HierarchyState(ContainerState, yaml.YAMLObject):

    """A class tto represent a hierarchy state for the state machine

    The hierarchy state holds several child states, that can be container states on their own
    """

    yaml_tag = u'!HierarchyState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 states=None, transitions=None, data_flows=None, start_state_id=None, scoped_variables=None,
                 v_checker=None, path=None, filename=None, check_path=True):

        ContainerState.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes, states,
                                transitions, data_flows, start_state_id, scoped_variables, v_checker, path, filename,
                                check_path=check_path)

    def run(self):
        """ This defines the sequence of actions that are taken when the hierarchy state is executed

        The input_data and output_data comes with a mapping from names to values,
        to transfer the data to the correct ports, the input_data.port_id has to be retrieved again
        :return:
        """

        if self.backward_execution:
            self.setup_backward_run()
        else:  # forward_execution
            self.setup_run()

        try:

            state = None
            if self.backward_execution:
                logger.debug("Backward executing hierarchy state with id %s and name %s" % (self._state_id, self.name))

                last_history_item = self.execution_history.pop_last_item()
                assert isinstance(last_history_item, ReturnItem)
                self.scoped_data = last_history_item.scoped_data

                scoped_variables_as_dict = {}
                self.get_scoped_variables_as_dict(scoped_variables_as_dict)
                self.exit(scoped_variables_as_dict, backward_execution=True)
                # do not write the output of the exit script
                # pop the remaining CallItem of the last hierarchy run from the history
                last_history_item = self.execution_history.pop_last_item()
                assert isinstance(last_history_item, CallItem)
                self.scoped_data = last_history_item.scoped_data

            else:  # forward_execution
                logger.debug("Executing hierarchy state with id %s and name %s" % (self._state_id, self.name))
                # handle data for the entry script
                scoped_variables_as_dict = {}
                self.execution_history.add_call_history_item(self, MethodName.ENTRY, self)
                self.get_scoped_variables_as_dict(scoped_variables_as_dict)
                self.enter(scoped_variables_as_dict)
                self.add_enter_exit_script_output_dict_to_scoped_data(scoped_variables_as_dict)
                self.execution_history.add_return_history_item(self, MethodName.ENTRY, self)
                state = self.get_start_state(set_final_outcome=True)

            ########################################################
            # children execution loop
            ########################################################
            last_error = None
            self.state_execution_status = StateExecutionState.EXECUTE_CHILDREN
            # depending on the execution mode pause execution
            logger.debug("Handling execution mode")
            execution_signal = singleton.state_machine_execution_engine.handle_execution_mode(self)
            while state is not self:

                self.backward_execution = False
                if self.preempted:
                    state.state_execution_status = StateExecutionState.INACTIVE
                    break

                last_history_item = None
                if execution_signal == "stop":
                    # this will be caught at the end of the run method
                    raise RuntimeError("state stopped")
                elif execution_signal == "backward_step":
                    self.backward_execution = True
                    last_history_item = self.execution_history.pop_last_item()
                    assert isinstance(last_history_item, ReturnItem)
                    self.scoped_data = last_history_item.scoped_data
                    state = last_history_item.state_reference

                if state is self:  # only in a backward execution case
                    # the last_history_item is the ReturnItem from the entry function,
                    # thus backward execute the entry function
                    scoped_variables_as_dict = {}
                    self.get_scoped_variables_as_dict(scoped_variables_as_dict)
                    self.enter(scoped_variables_as_dict, backward_execution=True)
                    # do not write the output of the entry script to the scoped data
                    # final outcome is not important as the execution order is fixed during backward stepping
                    # pop the last history item as it is the call item of the last entry execution
                    last_history_item = self.execution_history.pop_last_item()
                    assert isinstance(last_history_item, CallItem)
                    # copy the scoped_data of the history from the point before the state was executed
                    self.scoped_data = last_history_item.scoped_data
                    self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
                    if self.concurrency_queue:
                        self.concurrency_queue.put(self.state_id)
                    return  # outcome is not important as it is a backward execution
                else:  # forward step
                    logger.debug("Executing next state '{0}' (id {1}, type {2}, backwards: {3}".format(
                        state.name, state.state_id, type(state), self.backward_execution))
                    if not self.backward_execution:  # only add history item if it is not a backward execution
                        self.execution_history.add_call_history_item(state, MethodName.EXECUTE, self)
                    state.input_data = self.get_inputs_for_state(state)
                    state.output_data = self.create_output_dictionary_for_state(state)
                    if last_error is not None:
                        state.input_data["error"] = last_error
                    last_error = None
                    # execute the state
                    state.start(self.execution_history, backward_execution=self.backward_execution)
                    state.join()
                    if state.final_outcome is not None:
                        if state.final_outcome.outcome_id == -1:  # if the state aborted save the error
                            last_error = state.output_data["error"]
                    if state.backward_execution:
                        # the item popped now from the history will be a CallItem and will contain the scoped data,
                        # that was valid before executing the state
                        last_history_item = self.execution_history.pop_last_item()
                        assert isinstance(last_history_item, CallItem)
                        # copy the scoped_data of the history from the point before the state was executed
                        self.scoped_data = last_history_item.scoped_data
                        # go to the next state as it was a backward execution
                        state.state_execution_status = StateExecutionState.INACTIVE
                    else:
                        self.add_state_execution_output_to_scoped_data(state.output_data, state)
                        # print "---------------------- scoped data -----------------------"
                        # for key, value in self.scoped_data.iteritems():
                        #     print key, value
                        self.update_scoped_variables_with_output_dictionary(state.output_data, state)
                        self.execution_history.add_return_history_item(state, MethodName.EXECUTE, self)
                        # not explicitly connected preempted outcomes are implicit connected to parent preempted outcome
                        transition = self.get_transition_for_outcome(state, state.final_outcome)

                        if transition is None:
                            transition = self.handle_no_transition(state)
                        # it the transition is still None, then the state was preempted or aborted, in this case return
                        if transition is None:
                            state.state_execution_status = StateExecutionState.INACTIVE
                            # concurrency flag and active flag is set in self.handle_no_transition()
                            break

                        state.state_execution_status = StateExecutionState.INACTIVE
                        state = self.get_state_for_transition(transition)

                    # depending on the execution mode pause execution
                    logger.debug("Handling execution mode")
                    execution_signal = singleton.state_machine_execution_engine.handle_execution_mode(self)

            ########################################################
            # children execution loop end
            ########################################################

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

            # notify other threads that wait for this thread to finish
            if self.concurrency_queue:
                self.concurrency_queue.put(self.state_id)

            if self.preempted:
                self.final_outcome = Outcome(-2, "preempted")
                return

            # At least one child state was executed (if no child state was executed, the income is connected to an
            # outcome and the final_outcome is set by the get_start_state method)
            if transition is not None:
                self.final_outcome = self.outcomes[transition.to_outcome]
            logger.debug("Return from hierarchy state %s", self.name)
            return

        except Exception, e:
            if str(e) == "state stopped":
                logger.debug("State %s was stopped!" % self.name)
            else:
                logger.error("State %s had an internal error: %s %s" % (self.name, str(e), str(traceback.format_exc())))
            # notify other threads that wait for this thread to finish
            if self.concurrency_queue:
                self.concurrency_queue.put(self.state_id)
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
        state = HierarchyState(name=dict_representation['name'],
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