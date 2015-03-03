"""
.. module:: container_state
   :platform: Unix, Windows
   :synopsis: A module to represent a generic container state in the state machine

.. moduleauthor:: Sebastian Brunner


"""
from threading import Condition
import copy
from gtkmvc import Observable
import os

from utils import log
logger = log.get_logger(__name__)
from statemachine.enums import StateType, DataPortType
from statemachine.states.state import State
from statemachine.transition import Transition
from statemachine.outcome import Outcome
from statemachine.data_flow import DataFlow
from statemachine.scope import ScopedData, ScopedVariable
from statemachine.id_generator import *
from statemachine.config import *
from statemachine.validity_check.validity_checker import ValidityChecker
import statemachine.singleton


class ContainerState(State):

    """A class for representing a state in the statemachine

    Only the variables are listed that are not already contained in the state base class

    :ivar states: the child states of the container state of the state:
    :ivar transitions: transitions between all child states:
    :ivar data_flows: data flows between all child states:
    :ivar start_state: the state to start with when the hierarchy state is executed
    :ivar scoped_variables: the scoped variables of the container:
    :ivar _v_checker: reference to an object that checks the validity of this container state:

    """

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 states=None, transitions=None, data_flows=None, start_state=None,
                 scoped_variables=None, v_checker=None, path=None, filename=None, state_type=None, check_path=True):

        State.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes, path, filename,
                       state_type=state_type, check_path=check_path)

        self._states = None
        self.states = states
        self._transitions = None
        self.transitions = transitions
        self._data_flows = None
        self.data_flows = data_flows
        self._start_state_id = None
        self.start_state = start_state
        self._scoped_variables = None
        self.scoped_variables = scoped_variables
        self.__scoped_variables_names = []
        self._scoped_data = {}
        self._v_checker = v_checker
        self._current_state = None
        #condition variable to wait for not connected states
        self._transitions_cv = Condition()
        logger.debug("Container state with id %s and name %s initialized" % (self._state_id, self.name))

    def __str__(self):
        return "%s\nnumber of substates: %s" % (State.__str__(self), len(self.states))

    def run(self, *args, **kwargs):
        """Implementation of the abstract run() method of the :class:`threading.Thread`

        Should be filled with code, that should be executed for each container_state derivative.
        """
        raise NotImplementedError("The ContainerState.run() function has to be implemented!")

    def enter(self, scoped_variables_dict):
        """Called on entering the container state

        Here initializations of scoped variables and modules that are supposed to be used by the children take place.
        This method calls the custom entry function provided by a python script.

        :param scoped_variables_dict: a dictionary of all scoped variables that are passed to the custom function
        """
        logger.debug("Calling enter() script of container state with name %s", self.name)
        self.script.load_and_build_module()
        self.script.enter(self, scoped_variables_dict)

    def exit(self, scoped_variables_dict):
        """Called on exiting the container state

        Clean up code for the state and its variables is executed here. This method calls the custom exit function
        provided by a python script.
        :param scoped_variables_dict: a dictionary of all scoped variables that are passed to the custom function
        """
        logger.debug("Calling exit() script of container state with name %s", self.name)
        self.script.load_and_build_module()
        self.script.exit(self, scoped_variables_dict)

    def handle_no_transition(self, state):
        """
        This function handles the case that there is no transition for a specific outcome of a substate. It waits on a
        condition variable to a new transition that will be connected by the programmer or GUI-user.
        :param state: The substate to find a transition for
        :return: The transition for the target state.
        """
        transition = None
        while not transition:

            # aborted case for child state
            if state.final_outcome.outcome_id == -1:
                if self.concurrency_queue:
                    self.concurrency_queue.put(self.state_id)
                self.final_outcome = Outcome(-1, "aborted")
                self.active = False
                logger.debug("Exit hierarchy state %s with outcome aborted, as the child state returned "
                             "aborted and no transition was added to the aborted outcome!" % self.name)
                return None

            # preempted case for child state
            elif state.final_outcome.outcome_id == -2:
                if self.concurrency_queue:
                    self.concurrency_queue.put(self.state_id)
                self.final_outcome = Outcome(-2, "preempted")
                self.active = False
                logger.debug("Exit hierarchy state %s with outcome preempted, as the child state returned "
                             "preempted and no transition was added to the preempted outcome!" % self.name)
                return None

            # preempted case
            if self.preempted:
                if self.concurrency_queue:
                    self.concurrency_queue.put(self.state_id)
                self.final_outcome = Outcome(-2, "preempted")
                self.active = False
                logger.debug("Exit hierarchy state %s with outcome preempted, as the state itself "
                             "was preempted!" % self.name)
                return None

            # depending on the execution mode pause execution
            execution_signal = statemachine.singleton.state_machine_execution_engine.handle_execution_mode(self)
            if execution_signal == "stop":
                # this will be caught at the end of the run method
                raise RuntimeError("state stopped")

            # wait until the user connects the outcome of the state with a transition
            self._transitions_cv.acquire()
            self._transitions_cv.wait(3.0)
            self._transitions_cv.release()
            transition = self.get_transition_for_outcome(state, state.final_outcome)

        return transition

    # ---------------------------------------------------------------------------------------------
    # -------------------------------------- state functions --------------------------------------
    # ---------------------------------------------------------------------------------------------

    @Observable.observed
    # Primary key is state_id, as one should be able to change the name of the state without updating all connections
    def create_state(self, name, state_id=None):
        """Creates a state for the container state.

        :param name: the name of the new state
        :param state_id: the optional state_id for the new state
        :return state_id: the state_id of the created state
        """
        if state_id is None:
            state_id = state_id_generator(STATE_ID_LENGTH)
        state = State(state_id, name)
        self._states[state_id] = state
        return state_id

    @Observable.observed
    def add_state(self, state):
        """Adds a state to the container state.

        :param state: the state that is going to be added

        """
        if state.state_id in self._states:
            raise AttributeError("State id %s already exists in the container state", state.state_id)
        else:
            state.parent = self
            self._states[state.state_id] = state

    @Observable.observed
    def remove_state(self, state_id):
        """Remove a state from the container state.

        :param state_id: the id of the state to remove

        """
        if not state_id in self.states:
            raise AttributeError("State_id %s does not exist" % state_id)

        # remove script folder
        statemachine.singleton.global_storage.remove_path(self.states[state_id].script.path)

        #first delete all transitions and data_flows in this state
        keys_to_delete = []
        for key, transition in self.transitions.iteritems():
            if transition.from_state == state_id or transition.to_state == state_id:
                keys_to_delete.append(key)
        for key in keys_to_delete:
            self.remove_transition(key)

        keys_to_delete = []
        for key, data_flow in self.data_flows.iteritems():
            if data_flow.from_state == state_id or data_flow.to_state == state_id:
                keys_to_delete.append(key)
        for key in keys_to_delete:
            self.remove_data_flow(key)

        # second delete all states in this state
        if isinstance(self.states[state_id], ContainerState):
            for child_state_id in self.states[state_id].states.keys():
                self.states[state_id].remove_state(child_state_id)
            for transition_id in self.states[state_id].transitions.keys():
                self.states[state_id].remove_transition(transition_id)
            for data_flow_id in self.states[state_id].data_flows.keys():
                self.states[state_id].remove_data_flow(data_flow_id)

        # final delete the state it self
        del self.states[state_id]

    @Observable.observed
    def set_start_state(self, state):
        """Sets the start state of a container state

        :param state: The state_id of a state or a direct reference ot he state (that was already added
        to the container) that will be the start state of this container state.

        """
        if isinstance(state, State):
            self._start_state_id = state.state_id
        else:
            self._start_state_id = state

    def get_start_state(self):
        """Get the start state of the container state

        """
        if self.start_state is not None:
            return self.states[self.start_state]
        else:
            logger.warning("The container state %s with name %s does not has start state! %s" % (self.state_id,
                                                                                                 self.name,
                                                                                                 self.get_path()))
            return None

        # If there is a value in the dependency tree for this state start with the this one
        #TODO: start execution with the state provided by the dependency tree

    # ---------------------------------------------------------------------------------------------
    # ---------------------------------- transition functions -------------------------------------
    # ---------------------------------------------------------------------------------------------

    @Observable.observed
    #Primary key is transition_id
    def add_transition(self, from_state_id, from_outcome, to_state_id=None, to_outcome=None, transition_id=None):
        """Adds a transition to the container state

        Note: Either the toState or the toOutcome needs to be "None"

        :param from_state_id: The source state of the transition
        :param from_outcome: The outcome of the source state to connect the transition to
        :param to_state_id: The target state of the transition
        :param to_outcome: The target outcome of a container state
        :param transition_id: An optional transition id for the new transition
        """
        if transition_id is None:
            transition_id = generate_transition_id()

        # check if states are existing
        if not (from_state_id in self.states or from_state_id == self.state_id):
            raise AttributeError("From_state_id %s does not exist in the container state" % from_state_id.state_id)

        if not to_state_id is None:
            if not (to_state_id in self.states or to_state_id == self.state_id):
                raise AttributeError("To_state %s does not exist in the container state" % to_state_id.state_id)

        # get correct states
        from_state = None
        if from_state_id == self.state_id:
            from_state = self
        else:
            from_state = self.states[from_state_id]

        to_state = None
        if not to_state_id is None:
            if to_state_id == self.state_id:
                to_state = self
            else:
                to_state = self.states[to_state_id]

        if to_state_id is None and to_outcome is None:
            raise AttributeError("Either to_state_id or to_outcome must not be None")

        # check if outcome of from state is not already connected
        for trans_key, transition in self.transitions.iteritems():
            if transition.from_state == from_state_id:
                if transition.from_outcome == from_outcome:
                    raise AttributeError("outcome %s of state %s is already connected" %
                                         (str(from_outcome), str(from_state_id)))

        # check if state is a concurrency state, in concurrency states only transitions to the parents are allowd
        if self.state_type is StateType.BARRIER_CONCURRENCY or self.state_type is StateType.PREEMPTION_CONCURRENCY:
            if not to_state_id is None:  # None means that the target state is the containing state
                raise AttributeError("In concurrency states the to_state must be the container state itself")

        # finally add transition
        if from_outcome in from_state.outcomes:
            if not to_outcome is None:
                if to_outcome in self.outcomes:  # if to_state is None then the to_outcome must be an outcome of self
                    self.transitions[transition_id] =\
                        Transition(from_state_id, from_outcome, to_state_id, to_outcome, transition_id)
                else:
                    raise AttributeError("to_state does not have outcome %s", to_outcome)
            else:  # to outcome is None but to_state is not None, so the transition is valid
                self.transitions[transition_id] =\
                    Transition(from_state_id, from_outcome, to_state_id, to_outcome, transition_id)
        else:
            raise AttributeError("from_state does not have outcome %s", from_state)

        # notify all states waiting for transition to be connected
        self._transitions_cv.acquire()
        self._transitions_cv.notify_all()
        self._transitions_cv.release()

        return transition_id

    def get_transition_for_outcome(self, state, outcome):
        """Determines the next transition of a state.

        :param state: The state for which the transition is determined
        :param outcome: The outcome of the state, that is given in the first parameter
        :return: the transition specified by the the state and the outcome
        """
        if not isinstance(state, State):
            raise TypeError("ID must be of type State")
        if not isinstance(outcome, Outcome):
            raise TypeError("outcome must be of type Outcome")
        logger.debug("Return transition for state %s and outcome %s" % (state.name, outcome))
        result_transition = None
        for key, transition in self.transitions.iteritems():
            if transition.from_state == state.state_id and transition.from_outcome == outcome.outcome_id:
                result_transition = transition
        if result_transition is None:
            logger.warn("No transition found for state with name %s!" % self.name)
        return result_transition

    @Observable.observed
    def remove_transition(self, transition_id):
        """Removes a transition from the container state

        :param transition_id: the id of the transition to remove

        """
        if transition_id == -1 or transition_id == -2:
            raise AttributeError("The transition_id must not be -1 (Aborted) or -2 (Preempted)")
        if transition_id not in self.transitions:
            raise AttributeError("The transition_id %s does not exist" % str(transition_id))
        self._transitions.pop(transition_id, None)


    @Observable.observed
    def remove_outcome(self, outcome_id):
        """Remove an outcome from the state

        :param outcome_id: the id of the outcome to remove

        """
        if not outcome_id in self._used_outcome_ids:
            raise AttributeError("There is no outcome_id %s" % str(outcome_id))

        if outcome_id == -1 or outcome_id == -2:
            raise AttributeError("You cannot remove the outcomes with id -1 or -2 as a state must always be able to"
                                 "return aborted or preempted")

        # delete all transitions connected to this outcome
        if not self.parent is None:
            # delete external -> should be only one
            for transition_id, transition in self.parent.transitions.iteritems():
                if transition.from_outcome == outcome_id:
                    self.parent.remove_transition(transition_id)
                    # del self.parent.transitions[transition_id]
                    break  # found the one outgoing transition

        # delete internal -> could be multiple
        transition_ids_to_remove = []
        for transition_id, transition in self.transitions.iteritems():
            if transition.to_outcome == outcome_id:
                transition_ids_to_remove.append(transition_id)

        for transition_id in transition_ids_to_remove:
            self.remove_transition(transition_id)

        # delete outcome it self
        self._used_outcome_ids.remove(outcome_id)
        self._outcomes.pop(outcome_id, None)

    # ---------------------------------------------------------------------------------------------
    # ----------------------------------- data-flow functions -------------------------------------
    # ---------------------------------------------------------------------------------------------

    @Observable.observed
    #Primary key is data_flow_id.
    def add_data_flow(self, from_state_id, from_data_port_id, to_state_id, to_data_port_id, data_flow_id=None):
        """Adds a data_flow to the container state

        :param from_state_id: The id source state of the data_flow
        :param from_data_port_id: The output_key of the source state
        :param to_state_id: The id target state of the data_flow
        :param to_data_port_id: The input_key of the target state
        :param data_flow_id: an optional id for the data flow

        """
        if data_flow_id is None:
            data_flow_id = generate_data_flow_id()

        if not (from_state_id in self.states or from_state_id == self.state_id):
            raise AttributeError("From_state_id %s does not exist in the container state" % from_state_id.state_id)
        if not (to_state_id in self.states or to_state_id == self.state_id):
            raise AttributeError("To_state %s does not exit in the container state" % to_state_id.state_id)

        from_state = None
        from_key_type = None
        if from_state_id == self.state_id:  # data_flow originates in container state
            from_state = self
            if from_data_port_id in from_state.scoped_variables:
                from_key_type = DataPortType.SCOPED
            elif from_data_port_id in from_state.input_data_ports:
                from_key_type = DataPortType.INPUT
            else:
                raise AttributeError("from_data_port_id not in scoped_variables or input_data_ports")
        else:  # data flow originates in child state
            from_state = self.states[from_state_id]
            if from_data_port_id in from_state.output_data_ports:
                from_key_type = DataPortType.OUTPUT
            else:
                raise AttributeError("from_data_port_id not in output_data_ports")

        to_state = None
        to_key_type = None
        if to_state_id == self.state_id:  # data_flow ends in container state
            to_state = self
            if to_data_port_id in self.scoped_variables:
                to_key_type = DataPortType.SCOPED
            elif to_data_port_id in self.output_data_ports:
                to_key_type = DataPortType.OUTPUT
            else:
                raise AttributeError("to_data_port_id not in scoped_variables or output_data_ports")
        else:  # data_flow ends in child state
            to_state = self.states[to_state_id]
            if to_data_port_id in to_state.input_data_ports:
                to_key_type = DataPortType.INPUT
            else:
                raise AttributeError("to_data_port_id not in input_data_ports")

        #check if to_dataport_id of to_state has already a data_flow
        for flow_id, data_flow in self.data_flows.iteritems():
            # scoped variables are allowed to have several data_flows connecte to them
            if data_flow.to_state == to_state_id and not data_flow.to_state == self.state_id:
                if data_flow.to_key == to_data_port_id:
                    raise AttributeError("port %s of state %s already has a connection" %
                                         (str(to_data_port_id), str(to_state_id)))

        self.data_flows[data_flow_id] = DataFlow(from_state_id, from_data_port_id, to_state_id, to_data_port_id, data_flow_id)
        return data_flow_id

    @Observable.observed
    def remove_data_flow(self, data_flow_id):
        """ Removes a data flow from the container state

        :param data_flow_id: the id of the data_flow to remove

        """
        self.data_flows.pop(data_flow_id, None)

    def remove_data_flows_with_data_port_id(self, data_port_id):
        """Remove an data ports whose from_key or to_key equals the passed data_port_id

        :param data_port_id: the id of a data_port of which all data_flows should be removed, the id can be a input or
                            output data port id

        """
        # delete all data flows in parent related to data_port_id and self.state_id
        if not self.parent is None:
            data_flow_ids_to_remove = []
            for data_flow_id, data_flow in self.parent.data_flows.iteritems():
                if data_flow.from_state == self.state_id and data_flow.from_key == data_port_id or \
                        data_flow.to_state == self.state_id and data_flow.to_key == data_port_id:
                    data_flow_ids_to_remove.append(data_flow_id)

            for data_flow_id in data_flow_ids_to_remove:
                self.parent.remove_data_flow(data_flow_id)
                # del self.parent.data_flows[data_flow_id]

        # delete all data flows in self related to data_port_id and self.state_id
        data_flow_ids_to_remove = []
        for data_flow_id, data_flow in self.data_flows.iteritems():
            if data_flow.from_state == self.state_id and data_flow.from_key == data_port_id or \
                    data_flow.to_state == self.state_id and data_flow.to_key == data_port_id:
                data_flow_ids_to_remove.append(data_flow_id)

        for data_flow_id in data_flow_ids_to_remove:
            self.remove_data_flow(data_flow_id)
            # del self.data_flows[data_flow_id]

    @Observable.observed
    def modify_data_flow_from_state(self, data_flow_id, from_state, from_key):
        """The function accepts consistent changes of from_state with respective from_key.

        :param from_state: string of this state- or one of its child-state-state_id
        :param from_key: the for respective from_state unique data_port_id
        :return:
        """

        #check if types are valid
        if from_state is not None and not type(from_state) == str:
            raise TypeError("from_state must be of type str")
        if from_key is not None and not type(from_key) == int:
            raise TypeError("from_key must be of type int")

        # consistency check
        if from_state == self.state_id:
            if not from_key in self.input_data_ports and not from_key in self.scoped_variables:
                raise AttributeError("from_key must be in list of output_data_ports or scoped_variables")
        else:  # child
            if not from_state in self.states:
                raise AttributeError("from_state must be in list of child-states")
            if not from_key in self.states[from_state].output_data_ports:
                raise AttributeError("from_key must be in list of child-state input_data_ports")
        # set properties
        self.data_flows[data_flow_id].from_state = from_state
        self.data_flows[data_flow_id].from_key = from_key

    @Observable.observed
    def modify_data_flow_from_key(self, data_flow_id, from_key):
        """The function accepts consistent change from_key.

        :param from_key: the for respective from_state unique data_port_id
        :return:
        """
        #check if type is valid
        if from_key is not None and not type(from_key) == int:
            raise TypeError("from_key must be of type int")

        # consistency check
        from_state = self.data_flows[data_flow_id].from_state
        if from_state == self.state_id:
            if not from_key in self.input_data_ports and not from_key in self.scoped_variables:
                raise AttributeError("from_key must be in list of output_data_ports or scoped_variables")
        else:  # child
            if not from_key in self.states[from_state].output_data_ports:
                raise AttributeError("from_key must be in list of child-state input_data_ports")

        # set property
        self.data_flows[data_flow_id].from_key = from_key

    @Observable.observed
    def modify_data_flow_to_state(self, data_flow_id, to_state, to_key):
        """The function accepts consistent changes of to_state with respective to_key.

        :param to_state: string of this state- or one of its child-state-state_id
        :param to_key: the for respective to_state unique data_port_id
        :return:
        """
        # check if types are valid
        if type(to_state) == str:
            raise TypeError("to_state must be of type str")
        if type(to_key) == int:
            raise TypeError("to_key must be of type int")

        # consistency check
        if to_state == self.state_id:
            if not to_key in self.input_data_ports and not to_key in self.scoped_variables:
                raise AttributeError("to_key must be in list of child-state input_data_ports")
        else:  # child
            if not to_state in self.states:
                raise AttributeError("to_state must be in list of child-states")
            if not to_key in self.states[to_state].output_data_ports:
                raise AttributeError("to_key must be in list of child-state input_data_ports")

        # set properties
        self.data_flows[data_flow_id].to_state = to_state
        self.data_flows[data_flow_id].to_key = to_key

    @Observable.observed
    def modify_data_flow_to_key(self, data_flow_id, to_key):
        """The function accepts consistent change to_key.

        :param to_key: the for respective to_state unique data_port_id
        :return:
        """
        # check if type is valid
        if type(to_key) == int:
            raise TypeError("from_key must be of type int")

        # consistency check
        to_state = self.data_flows[data_flow_id].to_state
        if to_state == self.state_id:
            if not to_key in self.input_data_ports and not to_key in self.scoped_variables:
                raise AttributeError("to_key must be in list of child-state input_data_ports")
        else:  # child
            if not to_key in self.states[to_state].output_data_ports:
                raise AttributeError("to_key must be in list of child-state input_data_ports")

        # set property
        self.data_flows[data_flow_id].to_key = to_key

    # ---------------------------------------------------------------------------------------------
    # ---------------------------- scoped variables functions --------.----------------------------
    # ---------------------------------------------------------------------------------------------

    def get_scoped_variable_from_name(self, name):
        """ Get the scoped variable for a unique name

        :param name: the unique name of the scoped variable
        :return: the scoped variable specified by the name
        """
        for scoped_variable_id, scoped_variable in self.scoped_variables.iteritems():
            if scoped_variable.name == name:
                return scoped_variable_id
        raise AttributeError("Name %s is not in scoped_variables", name)

    #Primary key is the name of scoped variable.str
    @Observable.observed
    def add_scoped_variable(self, name, data_type=None, default_value=None, scoped_variable_id=None):
        """ Adds a scoped variable to the container state

        :param name: The name of the scoped variable
        :param data_type: An optional data type of the scoped variable
        :param default_value: An optional default value of the scoped variable
        :param scoped_variable_id: An optional scoped variable id of the
        :return: the unique id of the added scoped variable
        """
        if scoped_variable_id is None:
            scoped_variable_id = generate_data_flow_id()
        if name in self.__scoped_variables_names:
            raise AttributeError("A scoped variable with name %s already exists", name)
        self.__scoped_variables_names.append(name)
        self._scoped_variables[scoped_variable_id] = ScopedVariable(name, data_type, default_value, scoped_variable_id)
        return scoped_variable_id

    @Observable.observed
    def remove_scoped_variable(self, scoped_variable_id):
        """Remove a scoped variable from the container state

        :param scoped_variable_id: the id of the scoped variable to remove
        """
        if not scoped_variable_id in self._scoped_variables:
            raise AttributeError("A scoped variable with id %s does not exist" % str(scoped_variable_id))

        # delete all data flows connected to scoped_variable
        self.remove_data_flows_with_data_port_id(self._scoped_variables[scoped_variable_id].data_port_id)

        # remove scoped variable name
        self.__scoped_variables_names.remove(self._scoped_variables[scoped_variable_id].name)
        # delete scoped variable
        del self._scoped_variables[scoped_variable_id]

    @Observable.observed
    def modify_scoped_variable_name(self, name, data_port_id):
        """ Changes the name of the scoped variable specified by data_port_id

        :param name: the new name of the scoped variable
        :param data_port_id: the unique id of the scoped variable
        :return:
        """
        self.scoped_variables[data_port_id].name = name

    @Observable.observed
    def modify_scoped_variable_data_type(self, data_type, data_port_id):
        """ Changes the name of the scoped variable specified by data_port_id

        :param data_type: the new data type of the scoped variable
        :param data_port_id: the unique id of the scoped variable
        :return:
        """
        self.scoped_variables[data_port_id].default_value = None
        self.scoped_variables[data_port_id].data_type = data_type

    @Observable.observed
    def modify_scoped_variable_default_value(self, default_value, data_port_id):
        """ Changes the name of the scoped variable specified by data_port_id

        :param default_value: the new default variable of the scoped variable
        :param data_port_id: the unique id of the scoped variable
        :return:
        """
        val = self.convert_string_to_type(default_value, self.scoped_variables[data_port_id].data_type)
        if not val is None:
            self.scoped_variables[data_port_id].default_value = val

    # ---------------------------------------------------------------------------------------------
    # ---------------------------- scoped variables functions end ---------------------------------
    # ---------------------------------------------------------------------------------------------

    def get_data_port_by_id(self, data_port_id):
        """ Returns the io-data_port or scoped_variable with a certain data_id

        :param data_port_id: the unique id of the target data port
        :return: the data port specified by the data port
        """
        if data_port_id in self.input_data_ports:
            return self.input_data_ports[data_port_id]
        elif data_port_id in self.output_data_ports:
            return self.output_data_ports[data_port_id]
        elif data_port_id in self.scoped_variables:
            return self.scoped_variables[data_port_id]
        else:
            raise AttributeError("Data_Port_id %s is not in input_data_ports, output_data_ports or scoped_variables", data_port_id)

    # ---------------------------------------------------------------------------------------------
    # ------------------------------- input / output data handling --------------------------------
    # ---------------------------------------------------------------------------------------------

    def get_inputs_for_state(self, state):
        """Get all input data of an state

        :param state: the state of which the input data is determined
        :return: the input data of the target state
        """
        result_dict = {}

        for input_port_key, value in state.input_data_ports.iteritems():
            # at first load all default values
            result_dict[value.name] = copy.copy(value.default_value)
            # for all input keys fetch the correct data_flow connection and read data into the result_dict
            for data_flow_key, data_flow in self.data_flows.iteritems():
                if data_flow.to_key == input_port_key:
                    if data_flow.to_state == state.state_id:
                        # fetch data from the scoped_data list: the key is the data_port_key + the state_id
                        key = str(data_flow.from_key)+data_flow.from_state
                        if key in self.scoped_data:
                            result_dict[value.name] = copy.deepcopy(self.scoped_data[key].value)
                        else:  # if there is not value for the data port specified, take the default value
                            result_dict[value.name] = value.default_value
        return result_dict

    def get_outputs_for_state(self, state):
        """Return empty output dictionary for a state

        :param state: the state of which the output data is determined
        :return: the output data of the target state
        """
        result_dict = {}
        for key, data_port in state.output_data_ports.iteritems():
            result_dict[data_port.name] = None
        return result_dict

    # ---------------------------------------------------------------------------------------------
    # ---------------------------- functions to modify the scoped data ----------------------------
    # ---------------------------------------------------------------------------------------------

    def add_input_data_to_scoped_data(self, dictionary, state):
        """Add a dictionary to the scoped data

        As the input_data dictionary maps names to values, the functions looks for the proper data_ports keys in the
        input_data_ports dictionary

        :param dictionary: The dictionary that is added to the scoped data
        :param state: The state to which the input_data was passed (should be self in most cases)

        """
        for dict_key, value in dictionary.iteritems():
            for input_data_port_key, data_port in state.input_data_ports.iteritems():
                if dict_key == data_port.name:
                    state.scoped_data[str(input_data_port_key)+self.state_id] =\
                        ScopedData(data_port.name, value, type(value).__name__, state.state_id, DataPortType.INPUT)

    def add_state_execution_output_to_scoped_data(self, dictionary, state):
        """Add a state execution output to the scoped data

        :param dictionary: The dictionary that is added to the scoped data
        :param state: The state that finished execution and provide the dictionary
        """
        for output_name, value in dictionary.iteritems():
            for output_data_port_key, data_port in state.output_data_ports.iteritems():
                if output_name == data_port.name:
                    self.scoped_data[str(output_data_port_key)+state.state_id] =\
                        ScopedData(data_port.name, value, type(value).__name__, state.state_id, DataPortType.OUTPUT)

    def add_default_values_of_scoped_variables_to_scoped_data(self):
        """Add the scoped variables default values to the scoped_data dictionary

        """
        for key, scoped_var in self.scoped_variables.iteritems():
            self.scoped_data[str(scoped_var.data_port_id)+self.state_id] =\
                ScopedData(scoped_var.name, scoped_var.default_value, scoped_var.data_type, self.state_id, DataPortType.SCOPED)

    def update_scoped_variables_with_output_dictionary(self, dictionary, state):
        """Update the values of the scoped variables with the passed dictionary

        :param: the dictionary to update the scoped variables with
        :param: the state the output dictionary belongs to

        """
        for key, value in dictionary.iteritems():
            for data_flow_key, data_flow in self.data_flows.iteritems():
                if data_flow.to_state == self.state_id:
                    if data_flow.to_key in self.scoped_variables:
                        current_scoped_variable = self.scoped_variables[data_flow.to_key]
                        self.scoped_data[str(data_flow.to_key) + self.state_id] =\
                            ScopedData(current_scoped_variable.name, value, type(value).__name__, state.state_id, DataPortType.SCOPED)

    # ---------------------------------------------------------------------------------------------
    # ------------------------ functions to modify the scoped data end ----------------------------
    # ---------------------------------------------------------------------------------------------

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

    def get_scoped_variables_as_dict(self, dict):
        """ Get the scoped variables of the state as dictionary

        :param dict: the dict that is filled with the scoped variables
        :return:
        """
        for key_svar, svar in self.scoped_variables.iteritems():
            for key_sdata, sdata in self.scoped_data.iteritems():
                if svar.name == sdata.name and sdata.from_state == self.state_id:
                    if sdata.data_port_type is DataPortType.SCOPED:
                        dict[svar.name] = sdata.value

    def write_output_data(self):
        """ Write the scoped data to output of the state. Called before exiting the container state.

        :return:
        """
        for output_name, value in self.output_data.iteritems():
            output_port_id = self.get_io_data_port_id_from_name_and_type(output_name, DataPortType.OUTPUT)
            for data_flow_id, data_flow in self.data_flows.iteritems():
                if data_flow.to_state == self.state_id:
                    if data_flow.to_key == output_port_id:
                        self.output_data[output_name] = \
                            copy.deepcopy(self.scoped_data[str(data_flow.from_key)+data_flow.from_state].value)

    def add_enter_exit_script_output_dict_to_scoped_data(self, output_dict):
        """ Copy the data of the enter/exit scripts to the scoped data

        :param output_dict: the output dictionary of the scripts
        :return:
        """
        for output_name, output_data in output_dict.iteritems():
            for key_sdata, sdata in self.scoped_data.iteritems():
                if sdata.data_port_type is DataPortType.SCOPED and output_name == sdata.name:
                    scoped_variable_key = self.get_scoped_variable_from_name(output_name)
                    tmp = ScopedData(output_name, output_data, type(output_data).__name__, self.state_id, DataPortType.SCOPED)
                    self.scoped_data[str(scoped_variable_key)+self.state_id] = tmp

    # yaml part
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
            for key, state in states.iteritems():
                if not isinstance(state, State):
                    raise TypeError("element of container_state.states must be of type State")
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
        return self._start_state_id

    @start_state.setter
    @Observable.observed
    def start_state(self, start_state):
        if not start_state is None:
            if not isinstance(start_state, str):
                raise TypeError("start_state must be of type str")
        self._start_state_id = start_state

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
            for key, svar in scoped_variables.iteritems():
                if not isinstance(svar, ScopedVariable):
                    raise TypeError("element of scope must be of type ScopedVariable")
            self._scoped_variables = scoped_variables

    @property
    def scoped_data(self):
        """Property for the _scoped_data field

        """
        return self._scoped_data

    @scoped_data.setter
    #@Observable.observed
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
    #@Observable.observed
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
    #@Observable.observed
    def v_checker(self, v_checker):
        if not isinstance(v_checker, ValidityChecker):
            raise TypeError("validity_check must be of type ValidityChecker")
        self._v_checker = v_checker
