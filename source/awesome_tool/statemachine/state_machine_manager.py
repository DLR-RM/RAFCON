"""
.. module:: state_machine_manager
   :platform: Unix, Windows
   :synopsis: A module to organize a state machine with all its main components

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable

class StateMachineManager(Observable):

    """A class for to organizing all main components of a state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar _root_state: the root state of the state machine

    """

    def __init__(self, root_state=None):

        Observable.__init__(self)

        self._root_state = None
        self.root_state = root_state

    def start(self):
        self._root_state.start()

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