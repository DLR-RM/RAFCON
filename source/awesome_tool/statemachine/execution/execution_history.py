"""
.. module:: execution_history
   :platform: Unix, Windows
   :synopsis: A module for the history of one thread during state machine execution

.. moduleauthor:: Sebastian Brunner


"""
import time
import copy

# from awesome_tool.statemachine.states.state import State
# from awesome_tool.statemachine.states.container_state import ContainerState
from awesome_tool.statemachine.enums import MethodName
from awesome_tool.statemachine.enums import StateType
from awesome_tool.utils import log
logger = log.get_logger(__name__)


class ExecutionHistory():

    """A class for the history of a state machine execution

    """

    def __init__(self):
        self.history_items = []

    def get_last_history_item(self):
        if len(self.history_items) >= 1:
            return self.history_items[len(self.history_items) - 1]
        else:  # this is the case for the very first executed state
            return None

    def add_call_history_item(self, state, method_name, state_for_scoped_data):
        # if not isinstance(state, State):
        #     raise AttributeError("state must be of type State")
        return_item = CallItem(state, self.get_last_history_item(), method_name, state_for_scoped_data)
        if self.get_last_history_item() is not None:
            self.get_last_history_item().next = return_item
        self.history_items.append(return_item)
        return return_item

    def add_return_history_item(self, state, method_name, state_for_scoped_data):
        # if not isinstance(state, State):
        #     raise AttributeError("state must be of type State")
        return_item = ReturnItem(state, self.get_last_history_item(), method_name, state_for_scoped_data)
        if self.get_last_history_item() is not None:
            self.get_last_history_item().next = return_item
        self.history_items.append(return_item)
        return return_item

    def add_concurrency_history_item(self, state, number_concurrent_threads):
        # if not isinstance(state, State):
        #     raise AttributeError("state must be of type State")
        return_item = ConcurrencyItem(state, self.get_last_history_item(), number_concurrent_threads)
        if self.get_last_history_item() is not None:
            self.get_last_history_item().next = return_item
        self.history_items.append(return_item)
        return return_item

    def pop_last_item(self):
        if len(self.history_items) >= 1:
            return_item = self.history_items[len(self.history_items) - 1]
            self.history_items.remove(return_item)
            return return_item
        else:
            logger.error("No item left in the history item list in the execution history.")
            return None


class HistoryItem():

    def __init__(self, state, prev):

        # if not isinstance(state, State):
        #     raise AttributeError("state must be of type State!")
        self.state_reference = state
        self.path = state.get_path()
        self.timestamp = time.time()
        self.prev = prev
        self.next = None
        self._name = state.name

    def __str__(self):
        return "History element with reference state name %s (time: %s)\n" % (self._name, self.timestamp)


class ScriptItem(HistoryItem):

    def __init__(self, state, prev, method_name):
        HistoryItem.__init__(self, state, prev)
        self.method_name = method_name

    def __str__(self):
        return "ScriptItem %s" % (HistoryItem.__str__(self))


class CallItem(ScriptItem):

    def __init__(self, state, prev, method_name, state_for_scoped_data):
        ScriptItem.__init__(self, state, prev, method_name)
        # copy the scoped_data
        self.scoped_data = copy.deepcopy(state_for_scoped_data._scoped_data)

    def __str__(self):
        return "CallItem %s" % (ScriptItem.__str__(self))


class ReturnItem(ScriptItem):

    def __init__(self, state, prev, method_name, state_for_scoped_data):
        ScriptItem.__init__(self, state, prev, method_name)

        # copy the scoped_data
        self.scoped_data = copy.deepcopy(state_for_scoped_data._scoped_data)

        self.outcome = None
        if method_name is MethodName.EXECUTE or method_name is MethodName.EXIT:
            self.outcome = state.final_outcome

    def __str__(self):
        return "ReturnItem %s" % (ScriptItem.__str__(self))


class ConcurrencyItem(HistoryItem):

    def __init__(self, container_state, prev, number_concurrent_threads):
        HistoryItem.__init__(self, container_state, prev)
        # if not isinstance(container_state, ContainerState):
        #     raise AttributeError("state must be of type ContainerState")
        self.execution_histories = {}
        self.execution_heads = []

        # by setting the execution heads to self an history item can traverse back through the Concurrency History Item
        # to higher levels
        for i in range(number_concurrent_threads):
            self.execution_heads.append(self)

        for i in range(number_concurrent_threads):
            self.execution_histories[i] = ExecutionHistory()

    def __str__(self):
        return "ConcurrencyItem %s" % (HistoryItem.__str__(self))

    def get_index_for_head(self, head):
        index = 0
        for h in self.execution_heads:
            # here we compare references
            if h is head:
                return index
            else:
                index += 1

    def add_call_history_item(self, head, state, method_name, state_for_scoped_data):
        # if not isinstance(state, State):
        #     raise AttributeError("state must be of type State")
        if not isinstance(head, HistoryItem):
            raise AttributeError("head must be of type HistoryItem")

        new_history_item = self.execution_histories[self.get_index_for_head(head)].add_call_history_item(
            state, method_name, state_for_scoped_data
        )
        self.execution_heads[self.get_index_for_head(head)] = new_history_item
        return new_history_item

    def add_return_history_item(self, head, state, method_name, state_for_scoped_data):
        # if not isinstance(state, State):
        #     raise AttributeError("state must be of type State")
        if not isinstance(head, HistoryItem):
            raise AttributeError("head must be of type HistoryItem")

        new_history_item = self.execution_histories[self.get_index_for_head(head)].add_return_history_item(
            state, method_name, state_for_scoped_data
        )
        self.execution_heads[self.get_index_for_head(head)] = new_history_item
        return new_history_item

    def add_concurrency_history_item(self, head, state):
        # if not isinstance(state, State):
        #     raise AttributeError("state must be of type State")
        if not isinstance(head, HistoryItem):
            raise AttributeError("head must be of type HistoryItem")

        new_history_item = self.execution_histories[self.get_index_for_head(head)].add_concurrency_history_item(
            state)
        self.execution_heads[self.get_index_for_head(head)] = new_history_item
        return new_history_item