"""
.. module:: state_machine_manager
   :platform: Unix, Windows
   :synopsis: A module to organize a open state machine with all its main components

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable

from rafcon.statemachine.state_machine import StateMachine
from rafcon.statemachine.storage import storage
from rafcon.network.network_config import global_net_config

from rafcon.utils import log
logger = log.get_logger(__name__)


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
        self.active_state_machine_id = None
        sm_ids = [sm_id for sm_id in self.state_machines]
        for sm_id in sm_ids:
            self.remove_state_machine(sm_id)

    def refresh_state_machines(self, sm_ids, state_machine_id_to_path):
        from rafcon.statemachine.storage import storage
        for sm_idx in range(len(state_machine_id_to_path)):
            state_machine = storage.load_statemachine_from_path(state_machine_id_to_path[sm_ids[sm_idx]])
            self.add_state_machine(state_machine)

    def get_sm_id_for_root_state_id(self, root_state_id):
        for sm_id, sm in self.state_machines.iteritems():
            if sm.root_state.state_id == root_state_id:
                return sm_id

        logger.debug("sm_id is not found as long root_state_id is not found or identity check failed")
        return None

    def has_dirty_state_machine(self):
        """Checks if one of the registered sm has the marked_dirty flag set to True (i.e. the sm was recently modified,
        without being saved)

        :return: True if contains state machine that is marked dirty, False otherwise.
        """
        for sm in self.state_machines.itervalues():
            if sm.marked_dirty:
                return True
        return False

    def reset_dirty_flags(self):
        """Set all marked_dirty flags of the state machine to false."""
        for sm_id, sm in self.state_machines.iteritems():
            sm.marked_dirty = False

    @Observable.observed
    def add_state_machine(self, state_machine):
        """Add a state machine to the list of managed state machines. If there is no active state machine set yet,
            then set as active state machine.

        :param state_machine: State Machine Object
        """
        if not isinstance(state_machine, StateMachine):
            raise AttributeError("state_machine must be of type StateMachine")
        if state_machine.file_system_path is not None:
            for loaded_sm in self._state_machines.itervalues():
                if loaded_sm.file_system_path == state_machine.file_system_path:
                    raise AttributeError("The state-machine is already open")
        logger.debug("Add new state machine with id {0}".format(state_machine.state_machine_id))
        self._state_machines[state_machine.state_machine_id] = state_machine
        if self.active_state_machine_id is None:
            self.active_state_machine_id = state_machine.state_machine_id

    @Observable.observed
    def remove_state_machine(self, state_machine_id):
        """Remove the state machine for a specified state machine id from the list of registered state machines.

        :param state_machine_id: the id of the state machine to be removed
        """
        if state_machine_id in self._state_machines:
            del self._state_machines[state_machine_id]
            storage.clean_state_machine_paths(state_machine_id)
        else:
            logger.error("There is no state_machine with state_machine_id: %s" % state_machine_id)

        if state_machine_id is self.active_state_machine_id:
            if len(self._state_machines) > 0:
                self.active_state_machine_id = self._state_machines[self._state_machines.keys()[0]].state_machine_id
            else:
                self.active_state_machine_id = None

    def get_active_state_machine(self):
        """Return a reference to the active state-machine
        """
        if self._active_state_machine_id in self._state_machines:
            return self._state_machines[self._active_state_machine_id]
        else:
            return None

    #########################################################################
    # Properties for all class fields that must be observed by gtkmvc
    #########################################################################

    @property
    def state_machines(self):
        """Returns a reference to the list of state machines

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
        if state_machine_id is not None:
            if state_machine_id not in self.state_machines.keys():
                raise AttributeError("State machine not in list of all state machines")
        self._active_state_machine_id = state_machine_id
        active_state_machine = self.get_active_state_machine()
        if active_state_machine and active_state_machine.file_system_path:
            if global_net_config.get_config_value('NETWORK_CONNECTIONS'):
                from rafcon.network.singleton import network_connections
                network_connections.set_storage_base_path(active_state_machine.file_system_path)
