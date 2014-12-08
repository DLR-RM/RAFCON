"""
.. module:: container_state
   :platform: Unix, Windows
   :synopsis: A module to represent a generic container state in the statemachine

.. moduleauthor:: Sebastian Brunner


"""

from utils import log
logger = log.get_logger(__name__)

from state import State


class ContainerState(State):

    """A class for representing a state in the statemachine

    :ivar _states: the child states of the container state of the state:
    :ivar _transitions: transitions between all child states:
    :ivar _data_flows: data flows between all child states:
    :ivar _scope: common scope of container:
    :ivar _current_state: currently active state of the container:
    :ivar _scoped_keys: keys of all variables that can be accessed by each child state:
    :ivar _validity_checker: reference to an object that checks the validity of this container state:

    """

    def __init__(self):

        State.__init__(self)

        self._states = None
        self._transitions = None
        self._data_flows = None
        self._scope = None
        self._current_state = None
        self._scoped_keys = None
        self._validity_checker = None

    def run(self):
        logger.debug("Start container state with id %s", self._id)

    def enter(self):
        logger.debug("Calling enter() script of container state with id %s", self._id)

    def exit(self, transition):
        logger.debug("Calling exit() script of container state with id %s", self._id)

    def get_inputs_for_state(self, state):
        logger.debug("Return data inputs for container state with id %s", self._id)

    def get_outputs_for_state(self, state):
        logger.debug("Return data outputs for container state with id %s", self._id)

    def get_transition_for_outcome(self, state, outcome):
        logger.debug("Return transition for a specific child state and its specific outcome")

    def get_target_state_for_transition(self, transition):
        logger.debug("Return state for specific transition")