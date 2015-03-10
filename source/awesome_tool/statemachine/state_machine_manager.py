"""
.. module:: state_machine_manager
   :platform: Unix, Windows
   :synopsis: A module to organize a open state machine with all its main components

.. moduleauthor:: Sebastian Brunner


"""
from awesome_tool.utils import log
logger = log.get_logger(__name__)
from gtkmvc import Observable
from awesome_tool.statemachine.state_machine import StateMachine
import awesome_tool.statemachine.singleton


class StateMachineManager(Observable):

    """A class to organize all main components of a state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar _state_machine: a list of all state machine that are managed by the state machine manager
    :ivar _active_state_machine_id: the id of the currently active state machine

    """

    def __init__(self, state_machines=None):

        Observable.__init__(self)
        self._state_machines = {}
        self._active_state_machine_id = None

        if state_machines is not None:
            for state_machine in state_machines:
                self.add_state_machine(state_machine)

    def delete_all_state_machines(self):
        sm_keys = self.state_machines.keys()
        for key in sm_keys:
            self.remove_state_machine(key)

    def refresh_state_machines(self, sm_ids, state_machine_id_to_path):
        for sm_idx in range(len(state_machine_id_to_path)):
            [state_machine, version, creation_time] = awesome_tool.statemachine.singleton.\
                global_storage.load_statemachine_from_yaml(state_machine_id_to_path[sm_ids[sm_idx]])
            self.add_state_machine(state_machine)

    def get_sm_id_for_state(self, state):
        """
        Calculate the state_machine_id for the state
        :param state: the state to get the state id for
        :return:
        """
        state_path = state.get_path()
        path_item_list = state_path.split('/')
        root_state_id = path_item_list[0]

        for sm_id, sm in self.state_machines.iteritems():

            if sm.root_state.state_id == root_state_id:
                sm_state = sm.get_state_by_path(state_path)
                if sm_state and sm_state is state:
                    return sm_id
        return None

    @Observable.observed
    def add_state_machine(self, state_machine):
        """
        Add a state machine to the list of managed state machines. If there is no active state set yet, the active state
        :param state_machine:
        :return:
        """
        if not isinstance(state_machine, StateMachine):
            raise AttributeError("state_machine must be of type StateMachine")
        logger.debug("Add new state machine with id %s" % str(state_machine.state_machine_id))
        self._state_machines[state_machine.state_machine_id] = state_machine
        if self.active_state_machine_id is None:
            self.active_state_machine_id = state_machine.state_machine_id

    @Observable.observed
    def remove_state_machine(self, state_machine_id):
        """
        Remove the state machine for a specified state machine id from the list of registered state machines.
        :param state_machine_id: the id of the state machine to remove
        :return:
        """
        if state_machine_id is self.active_state_machine_id:
            self.active_state_machine_id = None
        if state_machine_id in self._state_machines:
            del self._state_machines[state_machine_id]
        else:
            logger.error("there is no valid argument state_machine_id: %s" % state_machine_id)

    def get_active_state_machine(self):
        """Return a reference to the active statemachine
        """
        if self._active_state_machine_id in self._state_machines:
            return self._state_machines[self._active_state_machine_id]
        else:
            logger.error("No active state machine specified!")

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

