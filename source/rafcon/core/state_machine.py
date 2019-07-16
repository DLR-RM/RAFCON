# Copyright (C) 2014-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Michael Vilzmann <michael.vilzmann@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>
# Sebastian Riedel <sebastian.riedel@dlr.de>

"""
.. module:: state_machine
   :synopsis: A module to organize a state machine with all its main components

"""

from future.utils import string_types
from builtins import range
from contextlib import contextmanager
from copy import copy
from threading import RLock
from datetime import datetime

from gtkmvc3.observable import Observable
from jsonconversion.jsonobject import JSONObject

import rafcon
from rafcon.core.execution.execution_history import ExecutionHistory, ExecutionHistoryStorage
from rafcon.core.id_generator import generate_state_machine_id, run_id_generator
from rafcon.utils import log
from rafcon.utils.hashable import Hashable
from rafcon.utils.storage_utils import get_current_time_string
import time

from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE
from rafcon.core.config import global_config
import os

logger = log.get_logger(__name__)


class StateMachine(Observable, JSONObject, Hashable):
    """A class for to organizing all main components of a state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar int StateMachine.state_machine_id: the id of the state machine
    :ivar rafcon.core.states.state StateMachine.root_state: the root state of the state machine
    :ivar str StateMachine.base_path: the path, where to save the state machine
    """

    state_machine_id = None
    version = None

    old_marked_dirty = True

    _root_state = None
    _marked_dirty = True
    _file_system_path = None

    def __init__(self, root_state=None, version=None, creation_time=None, last_update=None, state_machine_id=None):
        Observable.__init__(self)

        self._modification_lock = RLock()

        if state_machine_id is None:
            self.state_machine_id = generate_state_machine_id()
        else:
            self.state_machine_id = state_machine_id

        if root_state:
            self.root_state = root_state

        if version:
            self.version = version

        if creation_time:
            self.creation_time = creation_time
        else:
            self.creation_time = get_current_time_string()

        # this case happens if old state machines are loaded which do not have statemachine.json yet
        if isinstance(last_update, datetime):
            # see https://stackoverflow.com/questions/8022161/python-converting-from-datetime-datetime-to-time-time
            last_update = time.mktime(last_update.timetuple()) + last_update.microsecond / 1E6

        if last_update:
            self.last_update = last_update
        else:
            self.last_update = get_current_time_string()

        self._execution_histories = []

        # specifies if this state machine supports saving states with state_name + state_id
        self._supports_saving_state_names = True

    def __copy__(self):
        sm = self.__class__(copy(self._root_state), self.version, self.creation_time, self.last_update)
        sm._marked_dirty = self._marked_dirty
        return sm

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    @classmethod
    def from_dict(cls, dictionary, state_machine_id=None):
        state_machine_version = dictionary['version'] if 'version' in dictionary else dictionary['state_machine_version']
        creation_time = dictionary['creation_time']
        last_update = dictionary['last_update']
        return cls(None, state_machine_version, creation_time, last_update, state_machine_id)

    def to_dict(self):
        return self.state_machine_to_dict(self)

    def update_hash(self, obj_hash):
        self.root_state.update_hash(obj_hash)

    @staticmethod
    def state_machine_to_dict(state_machine):
        from rafcon.core.storage.storage import get_storage_id_for_state
        dict_representation = {
            'root_state_storage_id': get_storage_id_for_state(state_machine.root_state),
            'state_machine_version': state_machine.version,
            'used_rafcon_version': rafcon.__version__,
            'creation_time': state_machine.creation_time,
            'last_update': state_machine.last_update,
        }
        return dict_representation

    def start(self):
        """Starts the execution of the root state.
        """
        # load default input data for the state
        self._root_state.input_data = self._root_state.get_default_input_values_for_state(self._root_state)
        self._root_state.output_data = self._root_state.create_output_dictionary_for_state(self._root_state)
        new_execution_history = self._add_new_execution_history()
        new_execution_history.push_state_machine_start_history_item(self, run_id_generator())
        self._root_state.start(new_execution_history)

    def join(self):
        """Wait for root state to finish execution"""
        self._root_state.join()
        # execution finished, close execution history log file (if present)
        if len(self._execution_histories) > 0:
            if self._execution_histories[-1].execution_history_storage is not None:
                set_read_and_writable_for_all = global_config.get_config_value("EXECUTION_LOG_SET_READ_AND_WRITABLE_FOR_ALL", False)
                self._execution_histories[-1].execution_history_storage.close(set_read_and_writable_for_all)
        from rafcon.core.states.state import StateExecutionStatus
        self._root_state.state_execution_status = StateExecutionStatus.INACTIVE

    def get_modification_lock(self):
        return self._modification_lock

    @contextmanager
    def modification_lock(self, blocking=True):
        """Get modification lock in with() statement

        :param bool blocking: When True, block until the lock is unlocked, then set it to locked
        """
        try:
            yield self.acquire_modification_lock(blocking)
        finally:
            self.release_modification_lock()

    # @Observable.observed
    def acquire_modification_lock(self, blocking=True):
        """Acquires the modification lock of the state machine

        This must be used for all methods, that perform any modifications on the state machine

        :param bool blocking: When True, block until the lock is unlocked, then set it to locked
        :return: True if lock was acquired, False else
        :rtype: bool
        """
        return self._modification_lock.acquire(blocking)

    # @Observable.observed
    def release_modification_lock(self):
        """Releases the acquired state machine modification lock.
        """
        self._modification_lock.release()

    #########################################################################
    # Properties for all class fields that must be observed by gtkmvc3
    #########################################################################

    @property
    def root_state(self):
        """Property for the _root_state field
        """
        return self._root_state

    @root_state.setter
    @Observable.observed
    def root_state(self, root_state):
        # if not isinstance(root_state, State):
        #     raise AttributeError("root_state has to be of type State")
        if root_state is not None:
            from rafcon.core.states.state import State
            if not isinstance(root_state, State):
                raise AttributeError("root_state has to be of type State")
            root_state.parent = self
        self._root_state = root_state

    @property
    def execution_histories(self):
        return self._execution_histories

    @Observable.observed
    def clear_execution_histories(self):
        del self._execution_histories[:]

    def destroy_execution_histories(self):
        for execution_history in self._execution_histories:
            execution_history.destroy()
        self.clear_execution_histories()

    @Observable.observed
    def _add_new_execution_history(self):
        new_execution_history = ExecutionHistory()

        if global_config.get_config_value("EXECUTION_LOG_ENABLE", False):
            base_dir = global_config.get_config_value("EXECUTION_LOG_PATH", "%RAFCON_TEMP_PATH_BASE/execution_logs")
            if base_dir.startswith('%RAFCON_TEMP_PATH_BASE'):
                base_dir = base_dir.replace('%RAFCON_TEMP_PATH_BASE', RAFCON_TEMP_PATH_BASE)
            if not os.path.exists(base_dir):
                os.makedirs(base_dir)
            shelve_name = os.path.join(base_dir, '%s_rafcon_execution_log_%s.shelve' %
                                       (time.strftime('%Y-%m-%d-%H:%M:%S', time.localtime()),
                                        self.root_state.name.replace(' ', '-')))
            execution_history_store = ExecutionHistoryStorage(shelve_name)
            new_execution_history.set_execution_history_storage(execution_history_store)
        self._execution_histories.append(new_execution_history)
        return new_execution_history

    @Observable.observed
    def change_root_state_type(self, new_state_class):
        from rafcon.gui.helpers.state import create_new_state_from_state_with_type

        state = self.root_state
        state_id = state.state_id

        new_state = create_new_state_from_state_with_type(state, new_state_class)
        assert new_state.state_id == state_id
        # logger.info("ASSIGN NEW STATE")
        self.root_state = new_state  # Also sets the parent of root_state to self

        return new_state

    @property
    def marked_dirty(self):
        """Property for the _marked_dirty field
        """
        return self._marked_dirty

    @marked_dirty.setter
    @Observable.observed
    def marked_dirty(self, marked_dirty):
        # print("sm-core: marked dirty changed from ", self._marked_dirty, " to ", marked_dirty)
        if not isinstance(marked_dirty, bool):
            raise AttributeError("marked_dirty has to be of type bool")
        self.old_marked_dirty = self._marked_dirty
        self._marked_dirty = marked_dirty

    def get_state_by_path(self, path, as_check=False):
        if not path:
            logger.debug("No start state specified!")
            return None
        from rafcon.core.states.library_state import LibraryState
        from rafcon.core.states.execution_state import ExecutionState
        path_item_list = path.split('/')
        prev_state_id = path_item_list.pop(0)
        if not prev_state_id == self.root_state.state_id:
            logger.warning("First element in path ({0}) for method get_state_by_path has to be the root state "
                           "state_id ({1}).".format(path, self.root_state.state_id))
            return None
        state = self.root_state
        for state_id in path_item_list:
            if isinstance(state, LibraryState):
                state = state.state_copy
                assert state.state_id == state_id
            else:
                if not isinstance(state, ExecutionState) and state_id in state.states:
                    state = state.states[state_id]
                else:
                    note = ''
                    if isinstance(state, ExecutionState):
                        note = ", hierarchy-level ends at ExecutionState '{0}'".format(prev_state_id)
                    if not as_check:
                        logger.warning("Invalid path '{0}' for state machine '{1}'{2}".format(path,
                                                                                              self.state_machine_id,
                                                                                              note))
                    return None
            prev_state_id = state_id
        return state

    def get_last_execution_log_filename(self):
        if len(self._execution_histories) > 0:
            for i in range(len(self._execution_histories) - 1, -1, -1):
                if self._execution_histories[i].execution_history_storage is not None:
                    return self._execution_histories[i].execution_history_storage.filename
            return None
        else:
            return None

    @property
    def file_system_path(self):
        """Property for the _file_system_path field
        """
        return self._file_system_path

    @file_system_path.setter
    @Observable.observed
    def file_system_path(self, file_system_path):
        if not isinstance(file_system_path, string_types):
            raise AttributeError("file_system_path has to be a string")
        self._file_system_path = file_system_path

    @property
    def supports_saving_state_names(self):
        return self._supports_saving_state_names

    @supports_saving_state_names.setter
    def supports_saving_state_names(self, supports_saving_state_names):
        if not isinstance(supports_saving_state_names, bool):
            raise AttributeError("supports_saving_state_names has to be of type bool")
        self._supports_saving_state_names = supports_saving_state_names
