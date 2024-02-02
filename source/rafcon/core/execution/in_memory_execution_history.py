# Copyright (C) 2014-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>
# Sebastian Riedel <sebastian.riedel@dlr.de>
# ried_sa <Sebastian.Riedel@dlr.de>

"""
.. module:: execution_history
   :synopsis: A module for the history of one thread during state machine execution

"""
from collections.abc import Iterable, Sized

from rafcon.design_patterns.observer.observable import Observable

from rafcon.core.execution.base_execution_history import BaseExecutionHistory
from rafcon.utils import log
logger = log.get_logger(__name__)


class InMemoryExecutionHistory(BaseExecutionHistory, Observable, Iterable, Sized):
    """A class for the history of a state machine execution

        It stores all history elements in a stack wise fashion.

        :ivar initial_prev: optional link to a previous element for the first element pushed into this history of
                            type :class:`rafcon.core.execution.execution_history.HistoryItem`
    """

    def __init__(self, initial_prev=None, root_state_name="", consumer_manager=None):
        BaseExecutionHistory.__init__(self, initial_prev=initial_prev,
                                      root_state_name=root_state_name,
                                      consumer_manager=consumer_manager)
        Observable.__init__(self)
        Iterable.__init__(self)
        Sized.__init__(self)
        self._history_items = []
        logger.debug("InMemoryExecutionHistory has been created")

    def destroy(self):
        """Destroy the consumer and reset all variables
        """
        logger.verbose("Destroy execution history!")
        super(InMemoryExecutionHistory, self).destroy()
        if len(self._history_items) > 0:
            if self._history_items[0]:
                execution_history_iterator = iter(self)
                for history_item in execution_history_iterator:
                    history_item.destroy()
        self.destroyed = True
        self._history_items = None
        self.initial_prev = None

    def shutdown(self):
        """Stop all consumers including consumer plugins
        """
        self.consumer_manager.stop_consumers()

    def __iter__(self):
        return iter(self._history_items)

    def __len__(self):
        return len(self._history_items)

    def __getitem__(self, index):
        return self._history_items[index]

    def get_last_history_item(self):
        """Returns the history item that was added last

        :return: History item added last
        :rtype: rafcon.core.execution.execution_history_items.HistoryItem
        """
        try:
            return self[-1]
        except IndexError:  # this is the case for the very first executed state
            return None

    def _push_item(self, current_item):
        """Push history item to the queue

        :param current_item: the history item

        :return: History item added
        :rtype: rafcon.core.execution.execution_history_items.HistoryItem
        """
        try:
            self._history_items.append(current_item)
        except AttributeError:
            if self.destroyed:
                pass  # this is fine
            else:
                raise
        return current_item

    def _link_history_item(self, current_item):
        """Link the history item to the previous one

        :param current_item: the history item
        """
        last_history_item = self.get_last_history_item()
        if last_history_item is None:
            current_item.prev = self.initial_prev
        if last_history_item is not None:
            current_item.prev = last_history_item
            last_history_item.next = current_item

    @Observable.observed
    def push_call_history_item(self, state, call_type, state_for_scoped_data, input_data=None):
        """Adds a new call-history-item to the history item list

        A call history items stores information about the point in time where a method (entry, execute,
        exit) of certain state was called.

        :param state: the state that was called
        :param call_type: the call type of the execution step,
                            i.e. if it refers to a container state or an execution state
        :param state_for_scoped_data: the state of which the scoped data needs to be saved for further usages
            (e.g. backward stepping)
        """
        return_item = super(InMemoryExecutionHistory, self).push_call_history_item(
            state, call_type, state_for_scoped_data, input_data, link_and_feed_item_to_consumers=False)
        self._link_history_item(return_item)
        super(InMemoryExecutionHistory, self).feed_consumers(return_item)
        return self._push_item(return_item)

    @Observable.observed
    def push_return_history_item(self, state, call_type, state_for_scoped_data, output_data=None):
        """Adds a new return-history-item to the history item list

        A return history items stores information about the point in time where a method (entry, execute,
        exit) of certain state returned.

        :param state: the state that returned
        :param call_type: the call type of the execution step,
                            i.e. if it refers to a container state or an execution state
        :param state_for_scoped_data: the state of which the scoped data needs to be saved for further usages (e.g.
            backward stepping)
        """
        return_item = super(InMemoryExecutionHistory, self).push_return_history_item(
            state, call_type, state_for_scoped_data, output_data, link_and_feed_item_to_consumers=False)
        self._link_history_item(return_item)
        super(InMemoryExecutionHistory, self).feed_consumers(return_item)
        return self._push_item(return_item)

    @Observable.observed
    def push_concurrency_history_item(self, state, number_concurrent_threads):
        """Adds a new concurrency-history-item to the history item list

        A concurrent history item stores information about the point in time where a certain number of states is
        launched concurrently
        (e.g. in a barrier concurrency state).

        :param state: the state that launches the state group
        :param number_concurrent_threads: the number of states that are launched
        """
        return_item = super(InMemoryExecutionHistory, self).push_concurrency_history_item(
            state, number_concurrent_threads, link_and_feed_item_to_consumers=False)
        self._link_history_item(return_item)
        super(InMemoryExecutionHistory, self).feed_consumers(return_item)
        return self._push_item(return_item)

    @Observable.observed
    def push_state_machine_start_history_item(self, state_machine, run_id):
        """Adds a new state-machine-start-history-item to the history item list

        A state machine start history item stores information about the point in time where a state machine
        started to run

        :param state_machine: the state machine that started
        :param run_id: the run id
        """
        return_item = super(InMemoryExecutionHistory, self).push_state_machine_start_history_item(
            state_machine, run_id, feed_item_to_consumers=False)
        self._history_items.append(return_item)
        super(InMemoryExecutionHistory, self).feed_consumers(return_item)
        return return_item

    @Observable.observed
    def pop_last_item(self):
        """Delete and returns the last item of the history item list.

        :return: History item added last
        :rtype: rafcon.core.execution.execution_history_items.HistoryItem
        """
        try:
            return self._history_items.pop()
        except IndexError:
            logger.error("No item left in the history item list in the execution history.")
            return None
