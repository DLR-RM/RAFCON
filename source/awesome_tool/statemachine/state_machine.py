"""
.. module:: state_machine_manager
   :platform: Unix, Windows
   :synopsis: A module to organize a state machine with all its main components

.. moduleauthor:: Rico Belder


"""

from gtkmvc import Observable

from statemachine.states.container_state import ContainerState
from statemachine.id_generator import generate_state_machine_id
from utils import log
logger = log.get_logger(__name__)


class StateMachine(Observable):

    """A class for to organizing all main components of a state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar state_machine_id: the id of the state machine
    :ivar root_state: the root state of the state machine

    """

    def __init__(self, root_state):

        Observable.__init__(self)

        assert isinstance(root_state, ContainerState)

        self.state_machine_id = generate_state_machine_id()
        self._root_state = None
        self.root_state = root_state

    def start(self):
        """
        Starts the execution of the root state.
        :return:
        """
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

    @Observable.observed
    def root_state_before_change(self, model, prop_name, instance, method_name, args, kwargs):
        pass

    @Observable.observed
    def root_state_after_change(self, model, prop_name, instance, method_name, result, args, kwargs):
        pass