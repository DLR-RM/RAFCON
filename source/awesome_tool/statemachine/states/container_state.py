"""
.. module:: container_state
   :platform: Unix, Windows
   :synopsis: A module to represent a generic container state in the state machine

.. moduleauthor:: Sebastian Brunner


"""

from utils import log
logger = log.get_logger(__name__)
from gtkmvc import Observable
from threading import Condition

from statemachine.states.state import State
from statemachine.transition import Transition
from statemachine.outcome import Outcome
from statemachine.data_flow import DataFlow
from statemachine.scope import ScopedVariable, ScopedResult
from statemachine.id_generator import *
from statemachine.config import *
from statemachine.validity_check.validity_checker import ValidityChecker


class ContainerState(State, Observable):

    """A class for representing a state in the statemachine

    :ivar _states: the child states of the container state of the state:
    :ivar _transitions: transitions between all child states:
    :ivar _data_flows: data flows between all child states:
    :ivar _start_state: the state to start with when the hierarchy state is executed
    :ivar _scope: common scope of container:
    :ivar _current_state: currently active state of the container:
    :ivar _scoped_keys: keys of all variables that can be accessed by each child state:
    :ivar _v_checker: reference to an object that checks the validity of this container state:

    """

    def __init__(self, name=None, state_id=None, input_keys=None, output_keys=None, outcomes=None, sm_status=None,
                 states=None, transitions=None, data_flows=None, start_state=None, scoped_variables=None,
                 v_checker=None, path=None, filename=None):

        State.__init__(self, name, state_id, input_keys, output_keys, outcomes, sm_status, path, filename)

        self._states = None
        self.states = states
        self._transitions = None
        self.transitions = transitions
        self._data_flows = None
        self.data_flows = data_flows
        self._start_state = start_state
        self._scoped_variables = None
        self.scoped_variables = scoped_variables
        self._scoped_results = {}
        self._v_checker = v_checker
        self._current_state = None
        self._transitions_cv = Condition()

    def run(self, *args, **kwargs):
        """Implementation of the abstract run() method of the :class:`threading.Thread`

        Should be filled with code, that should be executed for each container_state derivative. Right now only debug
        statements.
        """
        #logger.debug("Start container state with id %s", self._state_id)
        raise NotImplementedError("The ContainerState.run() function has to be implemented!")

    def enter(self):
        """Called on entering the container state

        Here initializations of scoped variables and modules that are supposed to be used by the children take place.
        This method calls the custom entry function provided by a python script.
        """
        logger.debug("Calling enter() script of container state with id %s", self._state_id)
        self.script.load_and_build_module()
        self.script.enter(self, self.scoped_variables)

    def exit(self, transition):
        """Called on exiting the container state

        Clean up code for the state and its variables is executed here. This method calls the custom exit function
        provided by a python script.

        :param transition: The exit transition of the state
        """
        if not isinstance(transition, Transition):
            raise TypeError("ID must be of type Transition")
        logger.debug("Calling exit() script of container state with id %s", self._state_id)
        self.script.load_and_build_module()
        self.script.exit(self, self.scoped_variables)

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
        result_transition = None
        for key, transition in self.transitions.iteritems():
            if transition.from_state is state.state_id and transition.from_outcome is outcome.outcome_id:
                result_transition = transition
        if result_transition is None:
            logger.debug("No transition found!")
            exit()
        return result_transition

    # Primary key is state_id, as one should be able to change the name of the state without updating all connections
    def create_state(self, name, state_id=None):
        """Creates a state for the container state.

        :param name: the name of the new state
        :param state_id: the optional state_id for the new state

        """
        if state_id is None:
            state_id = state_id_generator(STATE_ID_LENGTH)
        state = State(state_id, name)
        self._states[state_id] = state
        return state_id

    def add_state(self, state):
        """Adds a state to the container state.

        :param state: the state that is going to be added

        """
        self._states[state.state_id] = state
        return

    #Primary key is transition_id.
    def add_transition(self, from_state, from_outcome, to_state, to_outcome):
        """Adds a transition to the container state

        Note: Either the toState or the toOutcome needs to be "None"

        :param from_state: The source state of the transition
        :param from_outcome: The outcome of the source state to connect the transition to
        :param to_state: The target state of the transition
        :param to_outcome: The target outcome of a container state
        """
        transition_id = generate_transition_id()
        self._transitions[transition_id] = Transition(from_state, from_outcome, to_state, to_outcome)
        return transition_id

    #Primary key is data_flow_id.
    def add_data_flow(self, from_state, from_key, to_state, to_key):
        """Adds a data_flow to the container state

        :param from_state: The source state of the data_flow
        :param from_key: The output_key of the source state
        :param to_state: The target state of the data_flow
        :param to_key: The input_key of the target state

        """
        data_flow_id = generate_data_flow_id()
        self.data_flows[data_flow_id] = DataFlow(from_state, from_key, to_state, to_key)
        return data_flow_id

    #Primary key is the name of scoped_key.
    def add_scoped_key(self, name, value_type):
        """Adds a data_flow to the container state

        :param name: The name of the scoped key

        """
        self._scoped_variables[name] = ScopedVariable(name, value_type)

    def set_start_state(self, state_id):
        """Adds a data_flow to the container state

        :param state_id: The state_id of the state (that was already added to the container)
                        that will be the start state

        """
        self._start_state = self._states[state_id]

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
        if states is None:
            self._states = {}
        else:
            if not isinstance(states, dict):
                raise TypeError("states must be of type dict")
            for s in states:
                if not isinstance(s, State):
                    raise TypeError("element of states must be of type State")
            self._states = states

    @property
    def transitions(self):
        """Property for the _transitions field

        """
        return self._transitions

    @transitions.setter
    @Observable.observed
    def transitions(self, transitions):
        if transitions is None:
            self._transitions = {}
        else:
            if not isinstance(transitions, dict):
                raise TypeError("transitions must be of type dict")
            for t in transitions:
                if not isinstance(t, Transition):
                    raise TypeError("element of transitions must be of type Transition")
            self._transitions = transitions

    @property
    def data_flows(self):
        """Property for the _data_flows field

        """
        return self._data_flows

    @data_flows.setter
    @Observable.observed
    def data_flows(self, data_flows):
        if data_flows is None:
            self._data_flows = {}
        else:
            if not isinstance(data_flows, dict):
                raise TypeError("data_flows must be of type dict")
            for df in data_flows:
                if not isinstance(df, DataFlow):
                    raise TypeError("element of data_flows must be of type DataFlow")
            self._data_flows = data_flows

    @property
    def start_state(self):
        """Property for the _start_state field

        """
        return self._start_state

    @start_state.setter
    @Observable.observed
    def start_state(self, start_state):
        if not isinstance(start_state, State):
            raise TypeError("start_state must be of type list or State")
        self._start_state = start_state

    @property
    def scoped_variables(self):
        """Property for the _scoped_variables field

        """
        return self._scoped_variables

    @scoped_variables.setter
    @Observable.observed
    def scoped_variables(self, scoped_variables):
        if scoped_variables is None:
            self._scoped_variables = {}
        else:
            if not isinstance(scoped_variables, dict):
                raise TypeError("scope_variables must be of type dict")
            for s in scoped_variables:
                if not isinstance(s, ScopedVariable):
                    raise TypeError("element of scope must be of type ScopeVariable")
            self._scoped_variables = scoped_variables

    @property
    def scoped_results(self):
        """Property for the _scoped_results field

        """
        return self._scoped_results

    @scoped_results.setter
    @Observable.observed
    def scoped_results(self, scoped_results):
        if not isinstance(scoped_results, dict):
            raise TypeError("scoped_results must be of type dict")
        for s in scoped_results:
            if not isinstance(s, ScopedResult):
                raise TypeError("element of scoped_keys must be of type ScopedResult")
        self._scoped_results = scoped_results

    @property
    def current_state(self):
        """Property for the _current_state field

        """
        return self._current_state

    @current_state.setter
    @Observable.observed
    def current_state(self, current_state):
        if not isinstance(current_state, State):
            raise TypeError("current_state must be of type State")
        self._current_state = current_state

    @property
    def v_checker(self):
        """Property for the _v_checker field

        """
        return self._v_checker

    @v_checker.setter
    @Observable.observed
    def v_checker(self, v_checker):
        if not isinstance(v_checker, ValidityChecker):
            raise TypeError("validity_check must be of type ValidityChecker")
        self._v_checker = v_checker