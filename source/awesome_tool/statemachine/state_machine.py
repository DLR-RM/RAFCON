"""
.. module:: state_machine_manager
   :platform: Unix, Windows
   :synopsis: A module to organize a state machine with all its main components

.. moduleauthor:: Rico Belder


"""

from gtkmvc import Observable

# from statemachine.states.container_state import ContainerState
# from statemachine.states.state import State
from statemachine.id_generator import generate_state_machine_id
from utils import log
logger = log.get_logger(__name__)


class StateMachine(Observable):

    """A class for to organizing all main components of a state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar state_machine_id: the id of the state machine
    :ivar root_state: the root state of the state machine
    :ivar base_path: the path, where to save the state machine

    """

    def __init__(self, root_state=None):

        Observable.__init__(self)

        # if root_state is not None:
        #     assert isinstance(root_state, State)

        self.state_machine_id = generate_state_machine_id()
        self._root_state = None
        self.root_state = root_state
        self.base_path = None

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
        # if not isinstance(root_state, State):
        #     raise AttributeError("root_state has to be of type State")
        self._root_state = root_state

    def get_state_by_path(self, path):
        path_item_list = path.split('/')

        state = None
        if path_item_list.pop(0) == self.root_state.state_id:
            state = self.root_state
            for state_id in path_item_list:
                try:
                    state = state.states[state_id]
                except:
                    logger.warning("----- STATE MACHINE NOT FOUND ----- for path %s and state %s" % (path, state))
                    state = None
                    break
        return state

    @Observable.observed
    def root_state_before_change(self, model, prop_name, instance, method_name, args, kwargs):
        pass

    @Observable.observed
    def root_state_after_change(self, model, prop_name, instance, method_name, result, args, kwargs):
        pass