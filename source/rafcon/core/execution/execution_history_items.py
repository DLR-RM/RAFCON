import copy
import json
import os
import pickle
import time
from enum import Enum

from rafcon.core.id_generator import history_item_id_generator
from rafcon.utils import log
logger = log.get_logger(__name__)


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

    def __init__(self, state, run_id):
        self._state_reference = state
        self.path = copy.deepcopy(state.get_path())
        self.timestamp = time.time()
        self.run_id = run_id
        self._prev = None
        self._next = None
        self.history_item_id = history_item_id_generator()
        self.state_type = str(type(state).__name__)

    def destroy(self):
        self._state_reference = None
        self.path = None
        self.timestamp = None
        self.run_id = None
        self._prev = None
        self._next = None
        self.history_item_id = None
        self.state_type = None

    @property
    def prev(self):
        """Property for the prev field
        """
        return self._prev

    @prev.setter
    def prev(self, prev):
        if not isinstance(prev, HistoryItem) and (prev is not None):
            raise TypeError("prev must be of type HistoryItem")
        self._prev = prev

    @property
    def next(self):
        """Property for the next field
        """
        return self._next

    @next.setter
    def next(self, next):
        if not isinstance(next, HistoryItem) and (next is not None):
            raise TypeError("next must be of type HistoryItem")
        self._next = next

    @property
    def state_reference(self):
        """Property for the state_reference field
        """
        return self._state_reference

    def __str__(self):
        return "HistoryItem with reference state name %s (time: %s)" % (self.state_reference.name, self.timestamp)

    def to_dict(self, pickled=True):
        record = dict()

        # here always the correct path is desired
        record['path'] = self.state_reference.get_path()
        record['path_by_name'] = self.state_reference.get_path(by_name=True)
        record['state_type'] = str(type(self.state_reference).__name__)

        from rafcon.core.states.library_state import LibraryState  # delayed imported on purpose
        if isinstance(self.state_reference, LibraryState):
            # in case of a Library State, all the data of the library itself should be used
            record['is_library'] = True
            # TODO: rename key to library_root_state_name? However, this will break many analysis scripts => next minor
            record['library_state_name'] = self.state_reference.state_copy.name
            record['library_name'] = self.state_reference.library_name
            record['library_path'] = self.state_reference.library_path
            target_state = self.state_reference.state_copy
        else:
            record['is_library'] = False
            record['library_state_name'] = None
            record['library_name'] = None
            record['library_path'] = None
            target_state = self.state_reference

        # there are 3 names of interest:
        # self.state_reference.library_name (<- name of the library on the filesystem)
        # self.state_reference.name (<- name of the user when using a library and changing the name)
        # self.state_reference.state_copy.name (<- the name of the library root state)
        record['state_name'] = self.state_reference.name
        record['timestamp'] = self.timestamp
        record['run_id'] = self.run_id  # library state and state copy have the same run_id
        record['history_item_id'] = self.history_item_id

        # semantic data
        semantic_data_dict = {}
        for k, v in target_state.semantic_data.items():
            try:
                semantic_data_dict[k] = pickle.dumps(v) if pickled else json.dumps(v)
            except Exception as e:
                semantic_data_dict['!' + k] = (str(e), str(v))
        record['semantic_data'] = semantic_data_dict

        record['description'] = target_state.description

        if self.prev is not None:
            record['prev_history_item_id'] = self.prev.history_item_id
        else:
            record['prev_history_item_id'] = None
        # store the specialized class name as item_type,
        # e.g. CallItem, ReturnItem, StatemachineStartItem when saved
        record['item_type'] = self.__class__.__name__
        return record


class StateMachineStartItem(HistoryItem):
    def __init__(self, state_machine, run_id):
        HistoryItem.__init__(self, state_machine.root_state, run_id)
        from rafcon.core.state_machine import StateMachine
        self.sm_dict = StateMachine.state_machine_to_dict(state_machine)
        self.prev = None
        self.os_environment = dict(os.environ)

    def __str__(self):
        return "StateMachineStartItem with name %s (time: %s)" % (self.sm_dict['root_state_storage_id'], self.timestamp)

    def to_dict(self, pickled=True):
        record = HistoryItem.to_dict(self, pickled=pickled)
        record.update(self.sm_dict)
        record['call_type'] = 'EXECUTE'
        record['state_name'] = 'StateMachineStartItem'
        record['state_type'] = 'StateMachine'
        record['path'] = ''
        record['path_by_name'] = ''
        record['os_environment'] = self.os_environment
        if self.prev is not None:
            record['prev_history_item_id'] = self.prev.history_item_id
        else:
            record['prev_history_item_id'] = None
        return record


class ScopedDataItem(HistoryItem):
    """A abstract class to represent history items which contains the scoped data of a state

    :ivar call_type: the call type of the execution step, i.e. if it refers to a container state or an execution state
    :ivar state_for_scoped_data: the state of which the scoped data will be stored as the context data that is necessary
        to re-execute the state
    """

    def __init__(self, state, call_type, state_for_scoped_data, child_state_input_output_data, run_id):
        HistoryItem.__init__(self, state, run_id)
        if call_type in CallType:
            self.call_type_str = call_type.name
        else:
            raise Exception('unkown calltype, neither CONTAINER nor EXECUTE')
        self.call_type = call_type
        self.scoped_data = {} if state_for_scoped_data is None else copy.deepcopy(state_for_scoped_data._scoped_data)
        self.child_state_input_output_data = copy.deepcopy(child_state_input_output_data)

    def to_dict(self, pickled=True):
        record = HistoryItem.to_dict(self, pickled=pickled)
        scoped_data_dict = {}
        for k, v in self.scoped_data.items():
            try:
                scoped_data_dict[v.name] = pickle.dumps(v.value) if pickled else json.dumps(v.value)
            except Exception as e:
                scoped_data_dict['!' + v.name] = (str(e), str(v.value))
        record['scoped_data'] = scoped_data_dict

        child_state_input_output_dict = {}
        for k, v in self.child_state_input_output_data.items():
            try:
                child_state_input_output_dict[k] = pickle.dumps(v) if pickled else json.dumps(v)
            except Exception as e:
                child_state_input_output_dict['!' + k] = (str(e), str(v))
        record['input_output_data'] = child_state_input_output_dict
        record['call_type'] = self.call_type_str
        return record

    def __str__(self):
        return "SingleItem %s" % (HistoryItem.__str__(self))


class CallItem(ScopedDataItem):
    """A history item to represent a state call
    """
    def __init__(self, state, call_type, state_for_scoped_data, input_data, run_id):
        ScopedDataItem.__init__(self, state, call_type, state_for_scoped_data, input_data, run_id)
        self.outcome = None

    def __str__(self):
        return "CallItem %s" % (ScopedDataItem.__str__(self))

    def to_dict(self, pickled=True):
        record = ScopedDataItem.to_dict(self, pickled=pickled)
        return record


class ReturnItem(ScopedDataItem):
    """A history item to represent the return of a root state call
    """
    def __init__(self, state, call_type, state_for_scoped_data, output_data, run_id):
        ScopedDataItem.__init__(self, state, call_type, state_for_scoped_data, output_data, run_id)
        self.outcome = copy.deepcopy(state.final_outcome)

    def __str__(self):
        return "ReturnItem %s" % (ScopedDataItem.__str__(self))

    def to_dict(self, pickled=True):
        record = ScopedDataItem.to_dict(self, pickled=pickled)
        if self.outcome is not None:
            record['outcome_name'] = self.outcome.to_dict()['name']
            record['outcome_id'] = self.outcome.to_dict()['outcome_id']
        else:
            record['outcome_name'] = 'None'
            record['outcome_id'] = -1
        return record


class ConcurrencyItem(HistoryItem):
    """A class to hold all the data for an invocation of several concurrent threads.
    """
    def __init__(self, container_state, number_concurrent_threads, run_id, consumer_manager):
        HistoryItem.__init__(self, container_state, run_id)
        self.execution_histories = []

        from rafcon.core.execution.in_memory_execution_history import InMemoryExecutionHistory
        from rafcon.core.execution.execution_history_factory import ExecutionHistoryFactory
        for i in range(number_concurrent_threads):
            execution_history = ExecutionHistoryFactory.get_execution_history(initial_prev=self,
                                                                              consumer_manager=consumer_manager)
            self.execution_histories.append(execution_history)

    def __str__(self):
        return "ConcurrencyItem %s" % (HistoryItem.__str__(self))

    def to_dict(self, pickled=True):
        record = HistoryItem.to_dict(self, pickled=pickled)
        record['call_type'] = 'CONTAINER'
        return record

    def destroy(self):
        for execution_history in self.execution_histories:
            execution_history.destroy()
        super(ConcurrencyItem, self).destroy()


CallType = Enum('METHOD_NAME', 'EXECUTE CONTAINER')