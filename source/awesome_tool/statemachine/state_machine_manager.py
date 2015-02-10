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
    _state_machines = {}
    __observable__ = ('_state_machines',)

    def __init__(self, state_machines=None):

        Observable.__init__(self)
        self._root_state = None
        if state_machines is not None:
            self.root_state = state_machines.values()[0].root_state

        # if type(state_machines) is list:
        #     for directory in state_machines:
        #         if os.path.isdir(directory):
        #             # TODO check if statemachine is in
        #             self.load_state_machine(directory)
        # else:
        #     logger.debug('state_machines has to be type-list of directories or instance of class-StateMachine')

    def start(self):
        self._root_state.start()

    def load_state_machine(self, directory):
        logger.debug("StateMachine should be loaded now ... directory: %s" % directory)
        # load state machine
        # add state machine

    @Observable.observed
    def add_state_machine(self, state_machine):
        logger.debug("StateMachine should be added to StateMachineManager")
        self._state_machines[state_machine.state_machine_id] = state_machine
        if self._root_state is None:
            self.root_state = state_machine.root_state

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
    def root_state(self):
        """Property for the _root_state field

        """
        return self._root_state

    @root_state.setter
    @Observable.observed
    def root_state(self, root_state):
        self._root_state = root_state