"""
.. module:: hierarchy_state
   :platform: Unix, Windows
   :synopsis: A module to represent a hierarchy state for the state machine

.. moduleauthor:: Sebastian Brunner


"""
import traceback
import copy

from rafcon.utils import log
from rafcon.core.states.container_state import ContainerState
from rafcon.core.state_elements.outcome import Outcome
import rafcon.core.singleton as singleton
from rafcon.core.execution.execution_history import CallItem, ReturnItem
from rafcon.core.execution.execution_status import StateMachineExecutionStatus
from rafcon.core.states.state import StateExecutionStatus
from rafcon.core.execution.execution_history import CallType

logger = log.get_logger(__name__)


class HierarchyState(ContainerState):

    """A class tto represent a hierarchy state for the state machine

    The hierarchy state holds several child states, that can be container states on their own
    """

    yaml_tag = u'!HierarchyState'

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 states=None, transitions=None, data_flows=None, start_state_id=None, scoped_variables=None):

        ContainerState.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes, states,
                                transitions, data_flows, start_state_id, scoped_variables)
        self.handling_execution_mode = False

        self.child_state = None
        self.last_error = None
        self.last_child = None
        self.last_transition = None

    def _initialize_hierarchy(self):
        """ This function covers the whole initialization routine before executing a hierarchy state.
        :return:
        """
        logger.debug("Starting execution of {0}{1}".format(self, " (backwards)" if self.backward_execution else ""))

        # reset variables
        self.child_state = None
        self.last_error = None
        self.last_child = None
        self.last_transition = None

        if self.backward_execution:
            self.setup_backward_run()
        else:  # forward_execution
            self.setup_run()

        self.state_execution_status = StateExecutionStatus.WAIT_FOR_NEXT_STATE
        if self.backward_execution:
            last_history_item = self.execution_history.pop_last_item()
            assert isinstance(last_history_item, ReturnItem)
            self.scoped_data = last_history_item.scoped_data

        else:  # forward_execution
            self.execution_history.push_call_history_item(self, CallType.CONTAINER, self, self.input_data)
            self.child_state = self.get_start_state(set_final_outcome=True)
            if self.child_state is None:
                self.child_state = self.handle_no_start_state()

    def run(self):
        """ This defines the sequence of actions that are taken when the hierarchy is executed. A hierarchy state
        executes all its child states recursively. Principally this code collects all input data for the next
        child state, executes it, stores its output data and determines the next state
        based on the outcome of the child state.
        :return:
        """

        try:
            self._initialize_hierarchy()
            while self.child_state is not self:
                self.handling_execution_mode = True
                execution_mode = singleton.state_machine_execution_engine.handle_execution_mode(self, self.child_state)
                self.handling_execution_mode = False
                if self.state_execution_status is not StateExecutionStatus.EXECUTE_CHILDREN:
                    self.state_execution_status = StateExecutionStatus.EXECUTE_CHILDREN

                self.backward_execution = False
                if self.preempted:
                    if self.last_transition and self.last_transition.from_outcome == -2:
                        logger.debug("Execute preemption handling for '{0}'".format(self.child_state))
                    else:
                        break
                elif execution_mode == StateMachineExecutionStatus.BACKWARD:
                    break_loop = self._handle_backward_execution_before_child_execution()
                    if break_loop:
                        break
                # This is only the case if this hierarchy-state is started in backward mode,
                # but the user directly switches to the forward execution mode
                if self.child_state is None:
                    break

                self._execute_current_child()

                if self.backward_execution:
                    break_loop = self._handle_backward_execution_after_child_execution()
                    if break_loop:
                        break
                else:
                    break_loop = self._handle_forward_execution_after_child_execution()
                    if break_loop:
                        break
            return self._finalize_hierarchy()

        except Exception, e:
            logger.error("{0} had an internal error: {1}\n{2}".format(self, str(e), str(traceback.format_exc())))
            self.output_data["error"] = e
            self.state_execution_status = StateExecutionStatus.WAIT_FOR_NEXT_STATE
            return self.finalize(Outcome(-1, "aborted"))

    def _handle_backward_execution_before_child_execution(self):
        """ Sets up all data after receiving a backward execution step from the execution engine
        :return: a flag to indicate if normal child state execution should abort
        """
        self.backward_execution = True
        last_history_item = self.execution_history.pop_last_item()
        if last_history_item.state_reference is self:
            # if the the next child_state in the history is self exit this hierarchy-state
            if self.child_state:
                # do not set the last state to inactive before executing the new one
                self.child_state.state_execution_status = StateExecutionStatus.INACTIVE
            return True
        assert isinstance(last_history_item, ReturnItem)
        self.scoped_data = last_history_item.scoped_data
        self.child_state = last_history_item.state_reference
        return False

    def _execute_current_child(self):
        """ Collect all data for a child state and execute it.
        :return:
        """

        self.child_state.input_data = self.get_inputs_for_state(self.child_state)
        self.child_state.output_data = self.create_output_dictionary_for_state(self.child_state)

        # process data of last state
        if self.last_error:
            self.child_state.input_data['error'] = copy.deepcopy(self.last_error)
        self.last_error = None
        if self.last_child:
            # do not set the last state to inactive before executing the new one
            self.last_child.state_execution_status = StateExecutionStatus.INACTIVE

        if not self.backward_execution:  # only add history item if it is not a backward execution
            self.execution_history.push_call_history_item(
                self.child_state, CallType.EXECUTE, self, self.child_state.input_data)

        self.child_state.start(self.execution_history, backward_execution=self.backward_execution)
        self.child_state.join()

        if self.preempted:
            if self.backward_execution:
                # this is the case if the user backward step through its state machine and stops it
                # as preemption behaviour in backward mode is not defined, set the state to forward mode
                # to ensure clean state machine shutdown
                self.backward_execution = False

        # set last_error and self.last_child
        if self.child_state.final_outcome is not None:  # final outcome can be None if only one state in a
            # hierarchy state is executed and immediately backward executed
            if self.child_state.final_outcome.outcome_id == -1:  # if the child_state aborted save the error
                self.last_error = ""
                if 'error' in self.child_state.output_data:
                    self.last_error = self.child_state.output_data['error']
        self.last_child = self.child_state

    def _handle_backward_execution_after_child_execution(self):
        """Cleanup the former child state execution and prepare for the next state execution in the backward
        execution case.
        :return: a flag to indicate if normal child state execution should abort
        """
        self.child_state.state_execution_status = StateExecutionStatus.WAIT_FOR_NEXT_STATE
        # the item popped now from the history will be a CallItem and will contain the scoped data,
        # that was valid before executing the child_state
        last_history_item = self.execution_history.pop_last_item()
        assert isinstance(last_history_item, CallItem)
        # copy the scoped_data of the history from the point before the child_state was executed
        self.scoped_data = last_history_item.scoped_data

        # this is a look-ahead step to directly leave this hierarchy-state if the last child_state
        # was executed; this leads to the backward and forward execution of a hierarchy child_state
        # having the exact same number of steps
        last_history_item = self.execution_history.get_last_history_item()
        if last_history_item.state_reference is self:
            last_history_item = self.execution_history.pop_last_item()
            assert isinstance(last_history_item, CallItem)
            self.scoped_data = last_history_item.scoped_data
            self.child_state.state_execution_status = StateExecutionStatus.INACTIVE
            return True
        return False

    def _handle_forward_execution_after_child_execution(self):
        """ Cleanup the former child state execution and prepare for the next state execution in the forward
        execution case.
        :return: a flag to indicate if normal child state execution should abort
        """
        self.add_state_execution_output_to_scoped_data(self.child_state.output_data, self.child_state)
        self.update_scoped_variables_with_output_dictionary(self.child_state.output_data, self.child_state)
        self.execution_history.push_return_history_item(
            self.child_state, CallType.EXECUTE, self, self.child_state.output_data)
        # not explicitly connected preempted outcomes are implicit connected to parent preempted outcome
        transition = self.get_transition_for_outcome(self.child_state, self.child_state.final_outcome)

        if transition is None:
            transition = self.handle_no_transition(self.child_state)
        # if the transition is still None, then the child_state was preempted or aborted, in this case
        # return
        if transition is None:
            return True

        self.last_transition = transition
        self.child_state = self.get_state_for_transition(transition)
        if transition is not None and self.child_state is self:
            self.final_outcome = self.outcomes[transition.to_outcome]

        if self.child_state is self:
            singleton.state_machine_execution_engine.notify_run_to_states(self)
        return False

    def _finalize_hierarchy(self):
        """ This function finalizes the execution of a hierarchy state. It sets the correct status and manages
        the output data handling.
        :return:
        """
        if self.last_child:
            self.last_child.state_execution_status = StateExecutionStatus.INACTIVE

        if not self.backward_execution:
            if self.last_error:
                self.output_data['error'] = copy.deepcopy(self.last_error)
            self.write_output_data()
            self.check_output_data_type()
            self.execution_history.push_return_history_item(self, CallType.CONTAINER, self, self.output_data)
            # add error message from child_state to own output_data

        self.state_execution_status = StateExecutionStatus.WAIT_FOR_NEXT_STATE

        if self.preempted:
            self.final_outcome = Outcome(-2, "preempted")

        return self.finalize(self.final_outcome)

