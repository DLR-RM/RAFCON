"""
.. module:: state_machine_manager
   :platform: Unix, Windows
   :synopsis: A module to organize a open state machine with all its main components

.. moduleauthor:: Sebastian Brunner


"""
import os
from utils import log
logger = log.get_logger(__name__)
from gtkmvc import Observable


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

    def load_state_machine(self, directory):
        """
        Loads a state machine from a directory.
        :param directory: the directory where the state machine should be loaded from
        :return:
        """
        # TODO: implement
        logger.debug("StateMachine should be loaded now ... directory: %s" % directory)
        # load state machine
        # add state machine

    @Observable.observed
    def add_state_machine(self, state_machine):
        """
        Add a state machine to the list of managed state machines. If there is no active state set yet, the active state
        :param state_machine:
        :return:
        """
        self._state_machines[state_machine.state_machine_id] = state_machine
        if self.active_state_machine_id is None:
            self.active_state_machine_id = state_machine.state_machine_id

    @Observable.observed
    def remove_state_machine(self, state_machine_id=None):
        if state_machine_id in self._state_machines:
            del self._state_machines[state_machine_id]
        else:
            logger.error("there is no valid argument state_machine_id: %s" % state_machine_id)

    def get_active_state_machine(self):
        """Return a reference to the active statemachine
        """
        return self._state_machines[self._active_state_machine_id]

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
        if state_machine_id not in self.state_machines.keys():
            raise AttributeError("State machine not in list of all state machines")
        self._active_state_machine_id = state_machine_id

