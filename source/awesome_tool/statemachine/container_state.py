"""
.. module:: container_state
   :platform: Unix, Windows
   :synopsis: A module to represent a generic container state in the state machine

.. moduleauthor:: Sebastian Brunner


"""

from utils import log
logger = log.get_logger(__name__)
from gtkmvc import Observable

from state import State
from transition import Transition
from outcome import Outcome
from data_flow import DataFlow
from scope_variable import ScopeVariable
from validity_checker import ValidityChecker


class ContainerState(State, Observable):

    """A class for representing a state in the statemachine

    :ivar _states: the child states of the container state of the state:
    :ivar _transitions: transitions between all child states:
    :ivar _data_flows: data flows between all child states:
    :ivar _scope: common scope of container:
    :ivar _current_state: currently active state of the container:
    :ivar _scoped_keys: keys of all variables that can be accessed by each child state:
    :ivar _validity_checker: reference to an object that checks the validity of this container state:

    """

    def __init__(self, states=None, transitions=None, data_flows=None, scope=None, scoped_keys=None, validity_checker=None):

        State.__init__(self)

        self._states = states
        self._transitions = transitions
        self._data_flows = data_flows
        self._scope = scope
        self._current_state = None
        self._scoped_keys = scoped_keys
        self._validity_checker = validity_checker

    def run(self):
        """Implementation of the abstract run() method of the :class:`threading.Thread`

        Should be filled with code, that should be executed for each container_state derivative. Right now only debug
        statements.
        """
        logger.debug("Start container state with id %s", self._state_id)

    def enter(self):
        """Called on entering the container state

        Here initialization of scoped variables and modules that are supposed to be used by the children take place.
        """
        logger.debug("Calling enter() script of container state with id %s", self._state_id)

    def exit(self, transition):
        """Called on exiting the container state

        Clean up code for the state and its variables is executed here.

        :param transition: The exit transition of the state

        """
        if not isinstance(transition, Transition):
            raise TypeError("ID must be of type Transition")
        logger.debug("Calling exit() script of container state with id %s", self._state_id)

    def get_inputs_for_state(self, state):
        """Return data inputs for a state

        """
        if not isinstance(state, State):
            raise TypeError("ID must be of type State")
        logger.debug("Return data inputs for container state with id %s", self._state_id)

    def get_outputs_for_state(self, state):
        """Return data inputs for a state

        """
        if not isinstance(state, State):
            raise TypeError("ID must be of type State")
        logger.debug("Return data outputs for container state with id %s", self._state_id)

    def get_transition_for_outcome(self, state, outcome):
        """Determines the next transition of a state.

        :param state: The state for which the transition is determined
        :param outcome: The outcome of the state, that is given in the first parameter
        """
        if not isinstance(state, State):
            raise TypeError("ID must be of type State")
        if not isinstance(outcome, Outcome):
            raise TypeError("ID must be of type Outcome")
        logger.debug("Return transition for a specific child state and its specific outcome")

    def get_target_state_for_transition(self, transition):
        """Determines the goal state of a transition.

        """
        if not isinstance(transition, Transition):
            raise TypeError("ID must be of type Transition")
        logger.debug("Return state for specific transition")

#########################################################################
# Properties for all class fields that must be observed by gtkmvc
#########################################################################

    @property
    def states(self):
        """Property for the _states field

        """
        return self._states

    @states.setter
    @Observable.observed
    def states(self, states):
        if not isinstance(states, (list, tuple)):
            raise TypeError("Name must be of type list or tuple")
        for s in states:
            if not isinstance(s, State):
                raise TypeError("Name must be of type State")
        self._states = states

    @property
    def transitions(self):
        """Property for the _transitions field

        """
        return self._transitions

    @transitions.setter
    @Observable.observed
    def transitions(self, transitions):
        if not isinstance(transitions, (list, tuple)):
            raise TypeError("Name must be of type list or tuple")
        for t in transitions:
            if not isinstance(t, Transition):
                raise TypeError("Name must be of type Transition")
        self._transitions = transitions

    @property
    def data_flows(self):
        """Property for the _data_flows field

        """
        return self._data_flows

    @data_flows.setter
    @Observable.observed
    def data_flows(self, data_flows):
        if not isinstance(data_flows, (list, tuple)):
            raise TypeError("Name must be of type list or tuple")
        for df in data_flows:
            if not isinstance(df, DataFlow):
                raise TypeError("Name must be of type DataFlow")
        self._data_flows = data_flows

    @property
    def scope(self):
        """Property for the _scope field

        """
        return self._scope

    @scope.setter
    @Observable.observed
    def scope(self, scope):
        if not isinstance(scope, (list, tuple)):
            raise TypeError("Name must be of type list or tuple")
        for s in scope:
            if not isinstance(s, ScopeVariable):
                raise TypeError("Name must be of type ScopeVariable")
        self._scope = scope

    @property
    def current_state(self):
        """Property for the _current_state field

        """
        return self._current_state

    @current_state.setter
    @Observable.observed
    def current_state(self, current_state):
        if not isinstance(current_state, State):
            raise TypeError("Name must be of type State")
        self._current_state = current_state

    @property
    def scoped_keys(self):
        """Property for the _scoped_keys field

        """
        return self._scoped_keys

    @scoped_keys.setter
    @Observable.observed
    def scoped_keys(self, scoped_keys):
        if not isinstance(scoped_keys, (list, tuple)):
            raise TypeError("Name must be of type list or tuple")
        for s in scoped_keys:
            if not isinstance(s, str):
                raise TypeError("Name must be of type str")
        self._scoped_keys = scoped_keys

    @property
    def validity_checker(self):
        """Property for the _validity_checker field

        """
        return self._validity_checker

    @validity_checker.setter
    @Observable.observed
    def validity_checker(self, validity_checker):
        if not isinstance(validity_checker, ValidityChecker):
            raise TypeError("Name must be of type ValidityChecker")
        self._validity_checker = validity_checker