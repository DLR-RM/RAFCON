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
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: state_machine_manager
   :synopsis: A module to organize a open state machine with all its main components

"""

from rafcon.design_patterns.singleton import Singleton
from rafcon.design_patterns.observer.observable import Observable

from rafcon.core.state_machine import StateMachine

from rafcon.utils import log
logger = log.get_logger(__name__)


@Singleton
class StateMachineManager(Observable):
    """A class to organize all main components of a state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar _state_machines: a list of all state machines that are managed by the state machine manager
    :ivar _active_state_machine_id: the id of the currently active state machine
    """

    _active_state_machine_id = None

    def __init__(self, state_machines=None):
        Observable.__init__(self)

        self._state_machines = {}

        if state_machines is not None:
            for state_machine in state_machines:
                self.add_state_machine(state_machine)

    def delete_all_state_machines(self):
        for sm_id in list(self.state_machines.keys()):
            if not (sm_id == self.active_state_machine_id):
                self.remove_state_machine(sm_id)

    def open_state_machines(self, state_machine_path_by_sm_id):
        from rafcon.core.storage import storage
        for sm_id, sm_path in state_machine_path_by_sm_id.items():
            state_machine = storage.load_state_machine_from_path(sm_path, sm_id)
            self.add_state_machine(state_machine)

    def get_sm_id_for_root_state_id(self, root_state_id):
        for sm_id, sm in self.state_machines.items():
            if sm.root_state.state_id == root_state_id:
                return sm_id

        logger.debug("sm_id is not found as long root_state_id is not found or identity check failed")
        return None

    def has_dirty_state_machine(self):
        """Checks if one of the registered sm has the marked_dirty flag set to True (i.e. the sm was recently modified,
        without being saved)

        :return: True if contains state machine that is marked dirty, False otherwise.
        """
        for sm in self.state_machines.values():
            if sm.marked_dirty:
                return True
        return False

    def is_state_machine_open(self, file_system_path):
        for loaded_sm in self._state_machines.values():
            if loaded_sm.file_system_path == file_system_path:
                return True
        return False

    @Observable.observed
    def add_state_machine(self, state_machine):
        """Add a state machine to the list of managed state machines. If there is no active state machine set yet,
            then set as active state machine.

        :param state_machine: State Machine Object
        :raises exceptions.AttributeError: if the passed state machine was already added of is of a wrong type
        """
        if not isinstance(state_machine, StateMachine):
            raise AttributeError("State machine must be of type StateMachine")
        if state_machine.file_system_path is not None:
            if self.is_state_machine_open(state_machine.file_system_path):
                raise AttributeError("The state machine is already open {0}".format(state_machine.file_system_path))
        logger.debug("Add new state machine with id {0}".format(state_machine.state_machine_id))
        self._state_machines[state_machine.state_machine_id] = state_machine
        return state_machine.state_machine_id

    @Observable.observed
    def remove_state_machine(self, state_machine_id):
        """Remove the state machine for a specified state machine id from the list of registered state machines.

        :param state_machine_id: the id of the state machine to be removed
        """
        removed_state_machine = None
        if state_machine_id in self._state_machines:
            logger.debug("Remove state machine with id {0}".format(state_machine_id))
            removed_state_machine = self._state_machines.pop(state_machine_id)
        else:
            logger.error("There is no state_machine with state_machine_id: %s" % state_machine_id)
            return removed_state_machine

        # destroy execution history
        removed_state_machine.destroy_execution_histories()
        return removed_state_machine

    def remove_state_machine_by_path(self, state_machine_path):
        """ Remove an open state machine by path

        :param str state_machine_path: the state machine path
        """

        state_machine_ids = []
        for state_machine_id, state_machine in self._state_machines.items():
            if state_machine.file_system_path == state_machine_path:
                state_machine_ids.append(state_machine_id)
        for state_machine_id in state_machine_ids:
            self.remove_state_machine(state_machine_id)

    def get_active_state_machine(self):
        """Return a reference to the active state-machine
        """
        if self._active_state_machine_id in self._state_machines:
            return self._state_machines[self._active_state_machine_id]
        else:
            return None

    def get_state_machine(self, state_machine_id):
        """Return a reference to the state machine with the given id if it exists
        """
        if state_machine_id in self._state_machines.keys():
            return self._state_machines[state_machine_id]
        else:
            return None

    def get_open_state_machine_of_file_system_path(self, file_system_path):
        """Return a reference to the state machine with respective path if open
        """
        for sm in self.state_machines.values():
            if sm.file_system_path == file_system_path:
                return sm

    #########################################################################
    # Properties for all class fields that must be observed by gtkmvc3
    #########################################################################

    @property
    def state_machines(self):
        """Return a reference to the list of state machines

        Please use this with care and do not alter the list in any way!
        """
        return self._state_machines

    @property
    def active_state_machine_id(self):
        """Return the currently active state machine
        """
        return self._active_state_machine_id

    @active_state_machine_id.setter
    @Observable.observed
    def active_state_machine_id(self, state_machine_id):
        import rafcon.core.singleton as core_singletons
        if state_machine_id is not None:
            if state_machine_id not in self.state_machines.keys():
                raise AttributeError("State machine not in list of all state machines")
        if not core_singletons.state_machine_execution_engine.finished_or_stopped() and \
                state_machine_id != self._active_state_machine_id:
            raise AttributeError("Active state machine can not be changed because state machine execution is active.")

        self._active_state_machine_id = state_machine_id
