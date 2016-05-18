"""
.. module:: concurrency_state
   :platform: Unix, Windows
   :synopsis: A module to represent a concurrency state for the state machine

.. moduleauthor:: Sebastian Brunner


"""
import Queue

from gtkmvc import Observable

from rafcon.statemachine.states.container_state import ContainerState
from rafcon.statemachine.enums import CallType
from rafcon.statemachine.execution.execution_history import CallItem, ReturnItem, ConcurrencyItem
from rafcon.statemachine.enums import StateExecutionState
from rafcon.statemachine.state_elements.outcome import Outcome


class ConcurrencyState(ContainerState):
    """A class to represent a concurrency state for the state machine

    The concurrency state holds several child states, that can be container states again
    """

    def __init__(self, name=None, state_id=None, input_keys=None, output_keys=None, outcomes=None,
                 states=None, transitions=None, data_flows=None, start_state_id=None, scoped_variables=None,
                 v_checker=None):
        ContainerState.__init__(self, name, state_id, input_keys, output_keys, outcomes, states, transitions,
                                data_flows, start_state_id, scoped_variables, v_checker)

    def run(self, *args, **kwargs):
        raise NotImplementedError("The ContainerState.run() function has to be implemented!")

    def _check_start_transition(self, start_transition):
        return False, "No start transitions are allowed in concurrency state"

    def setup_forward_or_backward_execution(self):
        if self.backward_execution:
            # pop the return item of this concurrency state to get the correct scoped data
            last_history_item = self.execution_history.pop_last_item()
            assert isinstance(last_history_item, ReturnItem)
            self.scoped_data = last_history_item.scoped_data
            # get the concurrency item for the children execution historys
            concurrency_history_item = self.execution_history.get_last_history_item()
            assert isinstance(concurrency_history_item, ConcurrencyItem)

        else:  # forward_execution
            self.execution_history.push_call_history_item(self, CallType.CONTAINER, self)
            concurrency_history_item = self.execution_history.push_concurrency_history_item(self, len(self.states))
        return concurrency_history_item

    def start_child_states(self, concurrency_history_item, do_not_start_state=None):
        self.state_execution_status = StateExecutionState.EXECUTE_CHILDREN
        # actually the queue is not needed in the barrier concurrency case
        # to avoid code duplication both concurrency states have the same start child function
        concurrency_queue = Queue.Queue(maxsize=0)  # infinite Queue size

        for index, state in enumerate(self.states.itervalues()):
            if state is not do_not_start_state:

                state.input_data = self.get_inputs_for_state(state)
                state.output_data = self.create_output_dictionary_for_state(state)

                if not self.backward_execution:
                    # care for the history items; this item is only for execution visualization
                    concurrency_history_item.execution_histories[index].push_call_history_item(
                        state, CallType.EXECUTE, self, state.input_data)
                else:  # backward execution
                    last_history_item = concurrency_history_item.execution_histories[index].pop_last_item()
                    assert isinstance(last_history_item, ReturnItem)

                state.concurrency_queue = concurrency_queue
                state.concurrency_queue_id = index

                state.start(concurrency_history_item.execution_histories[index], self.backward_execution)
        return concurrency_queue

    def join_state(self, state, history_index, concurrency_history_item):
        state.join()
        state.state_execution_status = StateExecutionState.INACTIVE
        # care for the history items
        if not self.backward_execution:
            state.concurrency_queue = None
            # add the data of all child states to the scoped data and the scoped variables
            state.execution_history.push_return_history_item(state, CallType.EXECUTE, self, state.output_data)
        else:
            last_history_item = concurrency_history_item.execution_histories[history_index].pop_last_item()
            assert isinstance(last_history_item, CallItem)

    def finalize_backward_execution(self):
        # backward_execution needs to be True to signal the parent container state the backward execution
        self.backward_execution = True
        # pop the ConcurrencyItem as we are leaving the barrier concurrency state
        last_history_item = self.execution_history.pop_last_item()
        assert isinstance(last_history_item, ConcurrencyItem)

        last_history_item = self.execution_history.pop_last_item()
        assert isinstance(last_history_item, CallItem)
        # this copy is convenience and not required here
        self.scoped_data = last_history_item.scoped_data
        self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
        return self.finalize()

    def finalize_concurrency_state(self, outcome):
        final_outcome = outcome
        self.write_output_data()
        self.check_output_data_type()
        self.execution_history.push_return_history_item(self, CallType.CONTAINER, self)
        self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE

        if self.preempted:
            final_outcome = Outcome(-2, "preempted")
        return self.finalize(final_outcome)

