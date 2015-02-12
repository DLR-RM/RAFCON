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

    """A class for to organizing all main components of a state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar _root_state: the root state of the state machine

    """

    def __init__(self, state_machines=None):

        Observable.__init__(self)
        self._root_state = None
        self._state_machines = {}
        self._active_state_machine = None

        if state_machines is not None:
            for state_machine in state_machines:
                self.add_state_machine(state_machine)

    def start(self):
        self._root_state.start()

    def load_state_machine(self, directory):
        logger.debug("StateMachine should be loaded now ... directory: %s" % directory)
        # load state machine
        # add state machine

    @Observable.observed
    def add_state_machine(self, state_machine):
        self._state_machines[state_machine.state_machine_id] = state_machine
        if self.active_state_machine is None:
            self.active_state_machine = state_machine.state_machine_id

    @Observable.observed
    def remove_state_machine(self, state_machine_id=None):
        if state_machine_id in self._state_machines:
            del self._state_machines[state_machine_id]
        else:
            logger.error("there is no valid argument state_machine_id: %s" % state_machine_id)

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
    def active_state_machine(self):
        """Return the currently active state machine
        """
        return self._active_state_machine

    @active_state_machine.setter
    @Observable.observed
    def active_state_machine(self, state_machine_id):
        if state_machine_id not in self.state_machines.keys():
            raise AttributeError("State machine not in list of all state machines")
        self._active_state_machine = state_machine_id