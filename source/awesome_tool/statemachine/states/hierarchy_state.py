"""
.. module:: hierarchy_state
   :platform: Unix, Windows
   :synopsis: A module to represent a hierarchy state for the state machine

.. moduleauthor:: Sebastian Brunner


"""
import traceback

from awesome_tool.statemachine.states.container_state import ContainerState
from awesome_tool.utils import log
logger = log.get_logger(__name__)
from awesome_tool.statemachine.outcome import Outcome
from awesome_tool.statemachine.enums import StateExecutionState
import awesome_tool.statemachine.singleton as singleton
from awesome_tool.statemachine.enums import MethodName
from awesome_tool.statemachine.execution.execution_history import CallItem, ReturnItem
from awesome_tool.statemachine.enums import StateMachineExecutionStatus

class HierarchyState(ContainerState):

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
        """ This defines the sequence of actions that are taken when the hierarchy child_state is executed

        The input_data and output_data comes with a mapping from names to values,
        to transfer the data to the correct ports, the input_data.port_id has to be retrieved again
        :return:
        """

        if self.backward_execution:
            self.setup_backward_run()
        else:  # forward_execution
            self.setup_run()

        try:

            child_state = None
            last_error = None
            last_state = None
            last_transition = None
            self.state_execution_status = StateExecutionState.EXECUTE_CHILDREN
            if self.backward_execution:
                logger.debug("Backward executing hierarchy child_state with id %s and name %s" % (self._state_id, self.name))
                last_history_item = self.execution_history.pop_last_item()
                assert isinstance(last_history_item, ReturnItem)
                self.scoped_data = last_history_item.scoped_data

            else:  # forward_execution
                logger.debug("Executing hierarchy child_state with id %s and name %s" % (self._state_id, self.name))
                self.execution_history.add_call_history_item(self, MethodName.CALL_CONTAINER_STATE, self)
                child_state = self.get_start_state(set_final_outcome=True)
                if child_state is None:
                    child_state = self.handle_no_start_state()

            ########################################################
            # children execution loop start
            ########################################################

            while child_state is not self:

                # depending on the execution mode pause execution
                logger.debug("Handling execution mode")
                execution_signal = singleton.state_machine_execution_engine.handle_execution_mode(self)

                self.backward_execution = False
                if self.preempted:
                    logger.debug("Preempted flag: True")
                    if last_transition.from_outcome == -2:
                        # normally execute the next state
                        logger.debug("Execute the preemption handling state for state %s" % str(last_state.name))
                    else:
                        break

                if execution_signal is StateMachineExecutionStatus.STOPPED:
                    # this will be caught at the end of the run method
                    raise RuntimeError("child_state stopped")
                elif execution_signal == StateMachineExecutionStatus.BACKWARD_STEP:
                    self.backward_execution = True
                    last_history_item = self.execution_history.pop_last_item()
                    if last_history_item.state_reference is self:
                        # if the the next child_state in the history is self exit this hierarchy-state
                        break
                    assert isinstance(last_history_item, ReturnItem)
                    self.scoped_data = last_history_item.scoped_data
                    child_state = last_history_item.state_reference

                if child_state is None:  # This is only the case if this hierarchy-state is started in backward mode, but
                    # the the user directly switches to the forward execution mode
                    break

                if not self.backward_execution:  # only add history item if it is not a backward execution
                    self.execution_history.add_call_history_item(child_state, MethodName.EXECUTE, self)

                child_state.input_data = self.get_inputs_for_state(child_state)
                child_state.output_data = self.create_output_dictionary_for_state(child_state)
                if last_error is not None:
                    child_state.input_data['error'] = last_error
                last_error = None
                logger.debug("Executing next child_state '{0}' (id {1}, type {2}, backwards: {3}".format(
                    child_state.name, child_state.state_id, type(child_state), self.backward_execution))
                child_state.start(self.execution_history, backward_execution=self.backward_execution)
                child_state.join()

                if child_state.final_outcome is not None:  # final outcome can be None if only one state in a
                    # hierarchy state is executed and immediately backward executed
                    if child_state.final_outcome.outcome_id == -1:  # if the child_state aborted save the error
                        last_error = ""
                        if 'error' in child_state.output_data:
                            last_error = child_state.output_data['error']

                if child_state.backward_execution:
                    child_state.state_execution_status = StateExecutionState.INACTIVE
                    # the item popped now from the history will be a CallItem and will contain the scoped data,
                    # that was valid before executing the child_state
                    last_history_item = self.execution_history.pop_last_item()
                    assert isinstance(last_history_item, CallItem)
                    # copy the scoped_data of the history from the point before the child_state was executed
                    self.scoped_data = last_history_item.scoped_data
                    logger.debug("Finished backward executing the child state with name %s!" % (child_state.name))

                    # this is a look-ahead step to directly leave this hierarchy-state if the last child_state
                    # was executed; this leads to the backward and forward execution of a hierarchy child_state having the
                    # exact same number of steps
                    last_history_item = self.execution_history.get_last_history_item()
                    if last_history_item.state_reference is self:
                        logger.debug("Leaving hierarchy child_state as the last child state was backward executed!")
                        last_history_item = self.execution_history.pop_last_item()
                        assert isinstance(last_history_item, CallItem)
                        self.scoped_data = last_history_item.scoped_data
                        break

                else:
                    self.add_state_execution_output_to_scoped_data(child_state.output_data, child_state)
                    self.update_scoped_variables_with_output_dictionary(child_state.output_data, child_state)
                    self.execution_history.add_return_history_item(child_state, MethodName.EXECUTE, self)
                    # not explicitly connected preempted outcomes are implicit connected to parent preempted outcome
                    transition = self.get_transition_for_outcome(child_state, child_state.final_outcome)

                    if transition is None:
                        transition = self.handle_no_transition(child_state)
                    # it the transition is still None, then the child_state was preempted or aborted, in this case return
                    child_state.state_execution_status = StateExecutionState.INACTIVE
                    if transition is None:
                        break

                    last_state = child_state
                    last_transition = transition
                    child_state = self.get_state_for_transition(transition)
                    if transition is not None and child_state is self:
                        self.final_outcome = self.outcomes[transition.to_outcome]

            ########################################################
            # children execution loop end
            ########################################################

            if not self.backward_execution:
                self.execution_history.add_return_history_item(self, MethodName.CALL_CONTAINER_STATE, self)

            self.write_output_data()
            self.check_output_data_type()
            # add error message from child_state to own output_data
            self.output_data['error'] = last_error

            self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE

            if self.preempted:
                self.final_outcome = Outcome(-2, "preempted")

            logger.debug("Returning from hierarchy state {0}".format(self.name))
            return self.finalize(self.final_outcome)

        except Exception, e:
            if str(e) == "child_state stopped" or str(e) == "state stopped":
                logger.debug("State '{0}' was stopped!".format(self.name))
            else:
                logger.error("State '{0}' had an internal error: {1}\n{2}".format(
                    self.name, str(e), str(traceback.format_exc())))

            self.output_data["error"] = e
            self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
            return self.finalize(Outcome(-1, "aborted"))

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
                               transitions=None,
                               data_flows=None,
                               scoped_variables=dict_representation['scoped_variables'],
                               v_checker=None,
                               check_path=False)
        try:
            state.description = dict_representation['description']
        except (ValueError, TypeError, KeyError):
            pass
        return state, dict_representation['transitions'], dict_representation['data_flows']
