"""
.. module:: execution_history
   :platform: Unix, Windows
   :synopsis: A module for the history of one thread during state machine execution

.. moduleauthor:: Sebastian Brunner


"""
import time
import copy
from collections import Iterable, Sized

from enum import Enum
from gtkmvc import Observable

from rafcon.utils import log
logger = log.get_logger(__name__)


class ExecutionHistory(Observable, Iterable, Sized):
    """A class for the history of a state machine execution

    It stores all history elements in a stack wise fashion.

    :ivar _history_items: a doubly linked list holding all history items of the
        type :class:`rafcon.core.execution.execution_history.HistoryItem`
    """

    def __init__(self):
        super(ExecutionHistory, self).__init__()
        self._history_items = []

    def __iter__(self):
        return iter(self._history_items)
            
    def __len__(self):
        return len(self._history_items)

    def __getitem__(self, index):
        return self._history_items[index]

    def get_last_history_item(self):
        """Returns the history item that was added last

        :return: History item added last
        :rtype: HistoryItem
        """
        try:
            return self[-1]
        except IndexError:  # this is the case for the very first executed state
            return None

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
        last_history_item = self.get_last_history_item()
        return_item = CallItem(state, last_history_item, call_type, state_for_scoped_data, input_data,
                               state.run_id)
        if last_history_item is not None:
            last_history_item.next = return_item
        self._history_items.append(return_item)
        return return_item

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
        return_item = ReturnItem(state, self.get_last_history_item(), call_type, state_for_scoped_data, output_data,
                                 state.run_id)
        if self.get_last_history_item() is not None:
            self.get_last_history_item().next = return_item

        self._history_items.append(return_item)
        return return_item

    @Observable.observed
    def push_concurrency_history_item(self, state, number_concurrent_threads):
        """Adds a new concurrency-history-item to the history item list

        A concurrent history item stores information about the point in time where a certain number of states is
        launched concurrently
        (e.g. in a barrier concurrency state).

        :param state: the state that launches the state group
        :param number_concurrent_threads: the number of states that are launched
        """
        return_item = ConcurrencyItem(state, self.get_last_history_item(), number_concurrent_threads, state.run_id)
        if self.get_last_history_item() is not None:
            self.get_last_history_item().next = return_item
        self._history_items.append(return_item)
        return return_item

    @Observable.observed
    def pop_last_item(self):
        """Delete and returns the last item of the history item list.

        :return: History item added last
        :rtype: HistoryItem
        """
        try:
            return self._history_items.pop()
        except IndexError:
            logger.error("No item left in the history item list in the execution history.")
            return None


class HistoryItem(object):
    """Class representing an entry within the history

    An abstract class that serves as a data structure to hold all important information of a certain point in time
    during the execution of a state machine. A history item is an element in a doubly linked history item list.

    :ivar state_reference: a reference to the state performing a certain action that is going to be saved
    :ivar path: the state path
    :ivar timestamp: the time of the call/return
    :ivar prev: the previous history item
    :ivar next: the next history item
    """

    def __init__(self, state, prev, run_id):
        self.state_reference = state
        self.path = state.get_path()
        self.timestamp = time.time()
        self.run_id = run_id
        self.prev = prev
        self.next = None

    def __str__(self):
        return "HistoryItem with reference state name %s (time: %s)" % (self.state_reference.name, self.timestamp)


class ScopedDataItem(HistoryItem):
    """A abstract class to represent history items which contains the scoped data of a state

    :ivar call_type: the call type of the execution step, i.e. if it refers to a container state or an execution state
    :ivar state_for_scoped_data: the state of which the scoped data will be stored as the context data that is necessary
        to re-execute the state
    """

    def __init__(self, state, prev, call_type, state_for_scoped_data, child_state_input_output_data, run_id):
        HistoryItem.__init__(self, state, prev, run_id)
        self.call_type = call_type
        self.scoped_data = copy.deepcopy(state_for_scoped_data._scoped_data)
        if child_state_input_output_data:
            self.child_state_input_output_data = copy.deepcopy(child_state_input_output_data)
        else:
            self.child_state_input_output_data = None

    def __str__(self):
        return "SingleItem %s" % (HistoryItem.__str__(self))


class CallItem(ScopedDataItem):
    """A history item to represent a state call
    """
    def __init__(self, state, prev, call_type, state_for_scoped_data, input_data, run_id):
        ScopedDataItem.__init__(self, state, prev, call_type, state_for_scoped_data, input_data, run_id)

    def __str__(self):
        return "CallItem %s" % (ScopedDataItem.__str__(self))


class ReturnItem(ScopedDataItem):
    """A history item to represent the return of a root state call
    """
    def __init__(self, state, prev, call_type, state_for_scoped_data, output_data, run_id):
        ScopedDataItem.__init__(self, state, prev, call_type, state_for_scoped_data, output_data, run_id)

        self.outcome = None
        self.outcome = state.final_outcome

    def __str__(self):
        return "ReturnItem %s" % (ScopedDataItem.__str__(self))


class ConcurrencyItem(HistoryItem):
    """A class to hold all the data for an invocation of several concurrent threads.
    """
    def __init__(self, container_state, prev, number_concurrent_threads, run_id):
        HistoryItem.__init__(self, container_state, prev, run_id)
        self.execution_histories = [ExecutionHistory() for _ in xrange(number_concurrent_threads)]

    def __str__(self):
        return "ConcurrencyItem %s" % (HistoryItem.__str__(self))


CallType = Enum('METHOD_NAME', 'EXECUTE CONTAINER')