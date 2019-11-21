# Copyright (C) 2014-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>
# Sebastian Riedel <sebastian.riedel@dlr.de>

"""
.. module:: concurrency_state
   :synopsis: A module to represent a concurrency state for the state machine

"""
from future import standard_library
standard_library.install_aliases()
import queue

from gtkmvc3.observable import Observable

import rafcon.core.singleton as singleton
from rafcon.core.states.container_state import ContainerState
from rafcon.core.execution.execution_history import CallType
from rafcon.core.execution.execution_history import CallItem, ReturnItem, ConcurrencyItem
from rafcon.core.states.state import StateExecutionStatus
from rafcon.core.state_elements.logical_port import Outcome


class ConcurrencyState(ContainerState):
    """A class to represent a concurrency state for the state machine

    The concurrency state holds several child states, that can be container states again
    """

    def __init__(self, name=None, state_id=None, input_keys=None, output_keys=None,
                 income=None, outcomes=None, states=None, transitions=None, data_flows=None, start_state_id=None,
                 scoped_variables=None, safe_init=True):
        ContainerState.__init__(self, name, state_id, input_keys, output_keys, income, outcomes, states, transitions,
                                data_flows, start_state_id, scoped_variables, safe_init=safe_init)

    def run(self, *args, **kwargs):
        """ The abstract run method that has to be implemented by all concurrency states.

        :param args: list of optional arguments
        :param kwargs: optional keyword arguments
        :raises exceptions.AttributeError: if this method was not implemented by the subclass
        """
        raise NotImplementedError("The ContainerState.run() function has to be implemented!")

    def _check_start_transition(self, transition):
        """ Each concurrency state has to check the validity of their own transitions.

        :param transition: the transition to be checked
        :return:
        """
        return False, "No start transitions are allowed in concurrency state"

    def setup_forward_or_backward_execution(self):
        """ Sets up the execution of the concurrency states dependent on if the state is executed forward of backward.

        :return:
        """
        if self.backward_execution:
            # pop the return item of this concurrency state to get the correct scoped data
            last_history_item = self.execution_history.pop_last_item()
            assert isinstance(last_history_item, ReturnItem)
            self.scoped_data = last_history_item.scoped_data
            # get the concurrency item for the children execution historys
            concurrency_history_item = self.execution_history.get_last_history_item()
            assert isinstance(concurrency_history_item, ConcurrencyItem)

        else:  # forward_execution
            self.execution_history.push_call_history_item(self, CallType.CONTAINER, self, self.input_data)
            concurrency_history_item = self.execution_history.push_concurrency_history_item(self, len(self.states))
        return concurrency_history_item

    def start_child_states(self, concurrency_history_item, do_not_start_state=None):
        """ Utility function to start all child states of the concurrency state.

        :param concurrency_history_item: each concurrent child branch gets an execution history stack of this
                                        concurrency history item
        :param do_not_start_state: optionally the id of a state can be passed, that must not be started (e.g. in the
                                    case of the barrier concurrency state the decider state)
        :return:
        """
        self.state_execution_status = StateExecutionStatus.EXECUTE_CHILDREN
        # actually the queue is not needed in the barrier concurrency case
        # to avoid code duplication both concurrency states have the same start child function
        concurrency_queue = queue.Queue(maxsize=0)  # infinite Queue size

        for index, state in enumerate(self.states.values()):
            if state is not do_not_start_state:

                state.input_data = self.get_inputs_for_state(state)
                state.output_data = self.create_output_dictionary_for_state(state)
                state.concurrency_queue = concurrency_queue
                state.concurrency_queue_id = index

                state.generate_run_id()
                if not self.backward_execution:
                    # care for the history items; this item is only for execution visualization
                    concurrency_history_item.execution_histories[index].push_call_history_item(
                        state, CallType.EXECUTE, self, state.input_data)
                else:  # backward execution
                    last_history_item = concurrency_history_item.execution_histories[index].pop_last_item()
                    assert isinstance(last_history_item, ReturnItem)
                state.start(concurrency_history_item.execution_histories[index], self.backward_execution, False)

        return concurrency_queue

    def join_state(self, state, history_index, concurrency_history_item):
        """ a utility function to join a state

        :param state: the state to join
        :param history_index: the index of the execution history stack in the concurrency history item
                                for the given state
        :param concurrency_history_item: the concurrency history item that stores the execution history stacks of all
                                        children
        :return:
        """
        state.join()
        if state.backward_execution:
            self.backward_execution = True

        state.state_execution_status = StateExecutionStatus.INACTIVE
        # care for the history items
        if not self.backward_execution:
            state.concurrency_queue = None
            # add the data of all child states to the scoped data and the scoped variables
            state.execution_history.push_return_history_item(state, CallType.EXECUTE, self, state.output_data)
        else:
            last_history_item = concurrency_history_item.execution_histories[history_index].pop_last_item()
            assert isinstance(last_history_item, CallItem)

    def finalize_backward_execution(self):
        """ Utility function to finalize the backward execution of the concurrency state.

        :return:
        """
        # backward_execution needs to be True to signal the parent container state the backward execution
        self.backward_execution = True
        # pop the ConcurrencyItem as we are leaving the barrier concurrency state
        last_history_item = self.execution_history.pop_last_item()
        assert isinstance(last_history_item, ConcurrencyItem)

        last_history_item = self.execution_history.pop_last_item()
        assert isinstance(last_history_item, CallItem)
        # this copy is convenience and not required here
        self.scoped_data = last_history_item.scoped_data
        self.state_execution_status = StateExecutionStatus.WAIT_FOR_NEXT_STATE
        return self.finalize()

    def finalize_concurrency_state(self, outcome):
        """ Utility function to finalize the forward execution of the concurrency state.

        :param outcome:
        :return:
        """
        final_outcome = outcome
        self.write_output_data()
        self.check_output_data_type()
        self.execution_history.push_return_history_item(self, CallType.CONTAINER, self, self.output_data)
        self.state_execution_status = StateExecutionStatus.WAIT_FOR_NEXT_STATE

        singleton.state_machine_execution_engine._modify_run_to_states(self)

        if self.preempted:
            final_outcome = Outcome(-2, "preempted")
        return self.finalize(final_outcome)

