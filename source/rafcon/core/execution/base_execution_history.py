from rafcon.core.execution.execution_history_items import CallItem, ReturnItem, ConcurrencyItem, \
    StateMachineStartItem, HistoryItem, ScopedDataItem
from rafcon.core.execution.consumer_manager import ExecutionHistoryConsumerManager

from rafcon.utils import log
logger = log.get_logger(__name__)


class BaseExecutionHistory(object):
    """A class for the history of a state machine execution

        :ivar initial_prev: optional link to a previous element for the first element pushed into this history of
                            type :class:`rafcon.core.execution.execution_history.HistoryItem`
    """

    def __init__(self, initial_prev=None, root_state_name="", consumer_manager=None):
        if not consumer_manager:
            self.consumer_manager = ExecutionHistoryConsumerManager(root_state_name)
        else:
            self.consumer_manager = consumer_manager
        self.initial_prev = initial_prev
        # saves the last history item in this variable in order to be able to get the pervious item id
        self.last_history_item = None
        self.destroyed = False
        logger.debug("BaseExecutionHistory has been created")

    def destroy(self):
        """ Destroy the consumer manager and reset all variables
        """
        self.initial_prev = None
        self.last_history_item = None
        self.consumer_manager.stop_consumers()

    def shutdown(self):
        """ Destroy the consumer manager and reset all variables
        """
        # we don't want to keep anyting of the last run in memory, thus we can simply destroy the execution history
        self.destroy()

    # For now implement a "len" functions, as the GUI expects the history to have a len function
    def __len__(self):
        return 0

    def get_last_history_item(self):
        """Returns the history item that was added last

        :return: History item added last
        :rtype: rafcon.core.execution.execution_history_items.HistoryItem
        """
        return self.last_history_item

    # TODO: delete the function from INEH? as it is redundant?
    def _link_item(self, current_item):
        """ Link the history item to the previous one

        :param current_item: the history item
        """
        last_history_item = self.get_last_history_item()
        if last_history_item is None:
            current_item.prev = self.initial_prev
        if last_history_item is not None:
            current_item.prev = last_history_item
            last_history_item.next = current_item
        self.last_history_item = current_item

    def feed_consumers(self, execution_history_item):
        """ Add execution history item to the dedicated queue of all consumers
        and notify their condition variables

        :param execution_history_item: the execution history item
        """
        self.consumer_manager.add_history_item_to_queue(execution_history_item)

    def push_call_history_item(self, state, call_type, state_for_scoped_data, input_data=None,
                               link_and_feed_item_to_consumers=True):
        """ Adds a new call-history-item to the history item list

        A call history items stores information about the point in time where a method (entry, execute,
        exit) of a certain state was called.

        :param state: the state that was called
        :param call_type: the call type of the execution step,
                            i.e. if it refers to a container state or an execution state
        :param state_for_scoped_data: the state of which the scoped data needs to be saved for further usages
            (e.g. backward stepping)
        :param link_and_feed_item_to_consumers: if the history item should be feed to all other consumers
        """
        from rafcon.core.states.library_state import LibraryState  # delayed imported on purpose
        if isinstance(state_for_scoped_data, LibraryState):
            state_for_scoped_data = state_for_scoped_data.state_copy
        history_item = CallItem(state, call_type, state_for_scoped_data, input_data, state.run_id)
        if link_and_feed_item_to_consumers and self.consumer_manager.consumers_exist:
            self._link_item(history_item)
            self.feed_consumers(history_item)
        return history_item

    def push_return_history_item(self, state, call_type, state_for_scoped_data, output_data=None,
                                 link_and_feed_item_to_consumers=True):
        """ Adds a new return-history-item to the history item list

        A return history items stores information about the point in time where a method (entry, execute,
        exit) of a certain state returned.

        :param state: the state that returned
        :param call_type: the call type of the execution step,
                            i.e. if it refers to a container state or an execution state
        :param state_for_scoped_data: the state of which the scoped data needs to be saved for further usages (e.g.
            backward stepping)
        """
        from rafcon.core.states.library_state import LibraryState  # delayed imported on purpose
        if isinstance(state_for_scoped_data, LibraryState):
            state_for_scoped_data = state_for_scoped_data.state_copy
        history_item = ReturnItem(state, call_type, state_for_scoped_data, output_data,
                                  state.run_id)
        if link_and_feed_item_to_consumers and self.consumer_manager.consumers_exist:
            self._link_item(history_item)
            self.feed_consumers(history_item)
        return history_item

    def push_concurrency_history_item(self, state, number_concurrent_threads, link_and_feed_item_to_consumers=True):
        """ Adds a new concurrency-history-item to the history item list

        A concurrent history item stores information about the point in time where a certain number of states is
        launched concurrently
        (e.g. in a barrier concurrency state).

        :param state: the state that launches the state group
        :param number_concurrent_threads: the number of states that are launched
        :param link_and_feed_item_to_consumers: if the history item should be feed to all other consumers
        """
        history_item = ConcurrencyItem(state, number_concurrent_threads, state.run_id, self.consumer_manager)
        if link_and_feed_item_to_consumers and self.consumer_manager.consumers_exist:
            self._link_item(history_item)
            self.feed_consumers(history_item)
        return history_item

    def push_state_machine_start_history_item(self, state_machine, run_id, feed_item_to_consumers=True):
        """ Adds a new state-machine-start-history-item to the history item list

        A state machine starts history item stores information about the point in time where a state machine
        started to run

        :param state_machine: the state machine that started
        :param run_id: the run id
        :param feed_item_to_consumers: if the history item should be feed to all other consumers
        """
        history_item = StateMachineStartItem(state_machine, run_id)
        if feed_item_to_consumers and self.consumer_manager.consumers_exist:
            self._link_item(history_item)
            self.feed_consumers(history_item)
        return history_item

