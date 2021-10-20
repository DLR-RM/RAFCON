from rafcon.core.execution.execution_history_items import CallItem, ReturnItem, ConcurrencyItem, \
    StateMachineStartItem, HistoryItem, ScopedDataItem
from rafcon.core.execution.consumer_manager import ExecutionHistoryConsumerManager

from rafcon.utils import log
logger = log.get_logger(__name__)


class BaseExecutionHistory(object):

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
        self.initial_prev = None
        self.last_history_item = None
        self.consumer_manager.stop_consumers()

    def shutdown(self):
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
        last_history_item = self.get_last_history_item()
        if last_history_item is None:
            current_item.prev = self.initial_prev
        if last_history_item is not None:
            current_item.prev = last_history_item
            last_history_item.next = current_item
        self.last_history_item = current_item

    def feed_consumers(self, execution_history_item):
        self.consumer_manager.add_history_item_to_queue(execution_history_item)

    def push_call_history_item(self, state, call_type, state_for_scoped_data, input_data=None,
                               link_and_feed_item_to_consumers=True):
        from rafcon.core.states.library_state import LibraryState  # delayed imported on purpose
        if isinstance(state_for_scoped_data, LibraryState):
            state_for_scoped_data = state_for_scoped_data.state_copy
        history_item = CallItem(state, call_type, state_for_scoped_data, input_data, state.run_id)
        if link_and_feed_item_to_consumers:
            self._link_item(history_item)
            self.feed_consumers(history_item)
        return history_item

    def push_return_history_item(self, state, call_type, state_for_scoped_data, output_data=None,
                                 link_and_feed_item_to_consumers=True):
        from rafcon.core.states.library_state import LibraryState  # delayed imported on purpose
        if isinstance(state_for_scoped_data, LibraryState):
            state_for_scoped_data = state_for_scoped_data.state_copy
        history_item = ReturnItem(state, call_type, state_for_scoped_data, output_data,
                                  state.run_id)
        if link_and_feed_item_to_consumers:
            self._link_item(history_item)
            self.feed_consumers(history_item)
        return history_item

    def push_concurrency_history_item(self, state, number_concurrent_threads, link_and_feed_item_to_consumers=True):
        history_item = ConcurrencyItem(state, number_concurrent_threads, state.run_id, self.consumer_manager)
        if link_and_feed_item_to_consumers:
            self._link_item(history_item)
            self.feed_consumers(history_item)
        return history_item

    def push_state_machine_start_history_item(self, state_machine, run_id, feed_item_to_consumers=True):
        history_item = StateMachineStartItem(state_machine, run_id)
        if feed_item_to_consumers:
            self.feed_consumers(history_item)
        return history_item

