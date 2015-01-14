"""
.. module:: container_state
   :platform: Unix, Windows
   :synopsis: A module to represent a generic container state in the state machine

.. moduleauthor:: Sebastian Brunner


"""
from gtkmvc import Observable
from threading import Condition
import copy

from utils import log
logger = log.get_logger(__name__)
from statemachine.states.state import State
from statemachine.transition import Transition
from statemachine.outcome import Outcome
from statemachine.data_flow import DataFlow
from statemachine.scope import ScopedData, ScopedVariable
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
    :ivar _scoped_variables: scoped variables that can be accessed by each child state:
    :ivar _scoped_data: dictionary that holds all values of the input_data, the scoped_variables and the outputs of
                        of the child states during runtime
    :ivar _v_checker: reference to an object that checks the validity of this container state:

    """

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 sm_status=None, states=None, transitions=None, data_flows=None, start_state=None,
                 scoped_variables=None, v_checker=None, path=None, filename=None, state_type=None):

        logger.debug("Creating new container state")
        State.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes, sm_status, path, filename,
                       state_type=state_type)

        self._states = None
        self.states = states
        self._transitions = None
        self.transitions = transitions
        self._data_flows = None
        self.data_flows = data_flows
        self._start_state = None
        self.start_state = start_state
        self._scoped_variables = None
        self.scoped_variables = scoped_variables
        self._scoped_data = {}
        self._v_checker = v_checker
        self._current_state = None
        self._transitions_cv = Condition()

    def __str__(self):
        return "state type: %s\n%s" % (self.state_type, State.__str__(self))

    def run(self, *args, **kwargs):
        """Implementation of the abstract run() method of the :class:`threading.Thread`

        Should be filled with code, that should be executed for each container_state derivative.
        """
        raise NotImplementedError("The ContainerState.run() function has to be implemented!")

    def enter(self):
        """Called on entering the container state

        Here initializations of scoped variables and modules that are supposed to be used by the children take place.
        This method calls the custom entry function provided by a python script.
        """
        logger.debug("Calling enter() script of container state with id %s", self._state_id)
        self.script.load_and_build_module()
        self.script.enter(self, self.scoped_variables)

    def exit(self):
        """Called on exiting the container state

        Clean up code for the state and its variables is executed here. This method calls the custom exit function
        provided by a python script.
        """
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
            raise TypeError("outcome must be of type Outcome")
        logger.debug("Return transition for a specific child state and its specific outcome")
        result_transition = None
        for key, transition in self.transitions.iteritems():
            #print "outcome: key: %s transition: %s" % (str(key), str(transition))
            if transition.from_state == state.state_id and transition.from_outcome == outcome.outcome_id:
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
        if state.state_id in self._states:
            print "This should never happen: For adding a new state to a container state, the new state must have" \
                  " a state_id that does not already exist in the container state!"
            exit()
        self._states[state.state_id] = state

    def remove_state(self, state_id):
        """Remove a state from the container state.

        :param state_id: the id of the state to remove

        """
        self._states.pop(state_id, None)

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

    def remove_transition(self, transition_id):
        """Removes a transition from the container state

        :param transition_id: the id of the transition to remove

        """
        self._transitions.pop(transition_id, None)

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

    def remove_data_flow(self, data_flow_id):
        """ Removes a data flow from the container state

        :param data_flow_id: the id of the data_flow to remove

        """
        self.data_flows.pop(data_flow_id, None)

    #Primary key is the name of scoped_key.
    def add_scoped_variable(self, name, value_type, default_value):
        """Adds a data_flow to the container state

        :param name: The name of the scoped key

        """
        self._scoped_variables[name] = ScopedVariable(name, value_type, self, default_value)

    def remove_scoped_variable(self, name):
        """Remove a scoped variable from the container state

        :param name: the name of the scoped variable to remove

        """
        del self._scoped_variables[name]

    def set_start_state(self, state_id):
        """Adds a data_flow to the container state

        :param state_id: The state_id (that was already added to the container) that will be the start state

        """
        self._start_state = state_id

    def get_start_state(self):
        """Get the start state of the container state

        """
        if self._sm_status.dependency_tree is None:
            return self.states[self.start_state]
        else:
            #TODO: do start with state provided by the dependency tree
            pass

    def get_inputs_for_state(self, state):
        """Get all input data of an state

        :param state: the state of which the input data is determined

        """
        result_dict = {}
        for input_port_key, value in state.input_data_ports.iteritems():
            #print "input_port_key: %s - value: %s" % (str(input_port_key), str(value))
            # at first load all default values
            result_dict[input_port_key] = value.default_value
            #print result_dict
            # for all input keys fetch the correct data_flow connection and read data into the result_dict
            for data_flow_key, data_flow in self.data_flows.iteritems():
                #print "data_flow_key: %s - data_flow: %s" % (str(data_flow_key), str(data_flow))
                if data_flow.to_key == input_port_key:
                    if data_flow.to_state == state.state_id:
                        # fetch data from the scoped_data list: the key is the data_port_key + the state_id
                        result_dict[input_port_key] =\
                            copy.deepcopy(self.scoped_data[data_flow.from_key+data_flow.from_state].value)
        return result_dict

    def get_outputs_for_state(self, state):
        """Return empty output dictionary for a state

        """
        result_dict = {}
        for key, value in state.output_data_ports.iteritems():
            result_dict[key] = None
        return result_dict

    def add_dict_to_scoped_data(self, dictionary, state=None):
        """Add a dictionary to the scope variables

        :param dictionary: The dictionary that is added to the scoped data

        """
        for key, value in dictionary.iteritems():
            if state is None:
                self.scoped_data[key+self.state_id] = ScopedData(key, value, type(value).__name__, self)
            else:
                self.scoped_data[key+state.state_id] = ScopedData(key, value, type(value).__name__, state)

    def add_scoped_variables_to_scoped_data(self):
        """Add the scoped variables default values to the scoped_data dictionary

        """
        for key, scoped_var in self.scoped_variables.iteritems():
            self.scoped_data[scoped_var.name+self.state_id] = ScopedData(scoped_var.name, scoped_var.default_value,
                                                                         scoped_var.value_type, self)

    def update_scoped_variables(self, dictionary, state):
        """Update the values of the scoped_variables that are stored in the scoped_data dictionary

        """
        for key, value in dictionary.iteritems():
            for data_flow_key, data_flow in self.data_flows.iteritems():
                if data_flow.to_state == self.state_id:
                    if data_flow.to_key in self.scoped_variables:
                        self.scoped_data[data_flow.to_key, self.state_id] =\
                            ScopedData(data_flow.to_key, value, type(value).__name__, state)

    def get_state_for_transition(self, transition):
        """Calculate the target state of a transition

        :param transition: The transition of which the target state is determined

        """
        if not isinstance(transition, Transition):
            raise TypeError("transition must be of type Transition")
        #the to_state is None when the transition connects an outcome of a child state to the outcome of a parent state
        if transition.to_state is None:
            return self
        else:
            return self.states[transition.to_state]

    def get_container_state_yaml_dict(data):
        dict_representation = {
            'name': data.name,
            'state_id': data.state_id,
            'state_type': str(data.state_type),
            'input_data_ports': data.input_data_ports,
            'output_data_ports': data.output_data_ports,
            'outcomes': data.outcomes,
            'path': data.script.path,
            'filename': data.script.filename,
            'transitions': data.transitions,
            'data_flows': data.data_flows,
            'start_state': data.start_state,
            'scoped_variables': data.scoped_variables
        }
        return dict_representation

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
            for key, value in transitions.iteritems():
                if not isinstance(value, Transition):
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
            for key, value in data_flows.iteritems():
                if not isinstance(value, DataFlow):
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
        if not start_state is None:
            if not isinstance(start_state, str):
                raise TypeError("start_state must be of type str")
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
    def scoped_data(self):
        """Property for the _scoped_data field

        """
        return self._scoped_data

    @scoped_data.setter
    @Observable.observed
    def scoped_data(self, scoped_data):
        if not isinstance(scoped_data, dict):
            raise TypeError("scoped_results must be of type dict")
        for s in scoped_data:
            if not isinstance(s, ScopedData):
                raise TypeError("element of scoped_data must be of type ScopedResult")
        self._scoped_data = scoped_data

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