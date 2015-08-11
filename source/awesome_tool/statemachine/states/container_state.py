"""
.. module:: container_state
   :platform: Unix, Windows
   :synopsis: A module to represent a generic container state in the state machine

.. moduleauthor:: Sebastian Brunner


"""
from threading import Condition
import copy
from gtkmvc import Observable

from awesome_tool.statemachine.enums import DataPortType, StateExecutionState
from awesome_tool.statemachine.script import Script, ScriptType
from awesome_tool.statemachine.states.state import State
from awesome_tool.statemachine.transition import Transition
from awesome_tool.statemachine.outcome import Outcome
from awesome_tool.statemachine.data_flow import DataFlow
from awesome_tool.statemachine.scope import ScopedData, ScopedVariable
from awesome_tool.statemachine.id_generator import *
from awesome_tool.statemachine.validity_check.validity_checker import ValidityChecker
from awesome_tool.statemachine.singleton import global_storage, state_machine_manager, state_machine_execution_engine
from awesome_tool.utils.type_helpers import type_inherits_of_type
from awesome_tool.utils import log
logger = log.get_logger(__name__)
from awesome_tool.statemachine.enums import StateMachineExecutionStatus


class ContainerState(State):

    """A class for representing a state in the statemachine

    Only the variables are listed that are not already contained in the state base class

    :ivar states: the child states of the container state of the state
    :ivar transitions: transitions between all child states
    :ivar data_flows: data flows between all child states
    :ivar start_state_id: the state to start with when the hierarchy state is executed
    :ivar scoped_variables: the scoped variables of the container

    """

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None, outcomes=None,
                 states=None, transitions=None, data_flows=None, start_state_id=None,
                 scoped_variables=None, v_checker=None, path=None, filename=None, check_path=True):

        State.__init__(self, name, state_id, input_data_ports, output_data_ports, outcomes)

        self.script = Script(path, filename, script_type=ScriptType.CONTAINER, check_path=check_path, state=self)

        self._states = {}
        self._transitions = {}
        self._data_flows = {}
        self._scoped_variables = {}
        self._scoped_data = {}
        self.__scoped_variables_names = []
        # reference to an object that checks the validity of this container state
        self._v_checker = v_checker
        self._current_state = None
        # condition variable to wait for not connected states
        self._transitions_cv = Condition()
        self._child_execution = False

        self.scoped_variables = scoped_variables
        self.states = states
        self.transitions = transitions
        self.data_flows = data_flows
        if start_state_id is not None:
            self.start_state_id = start_state_id
        logger.debug("Container state with id %s and name %s initialized" % (self._state_id, self.name))

    # ---------------------------------------------------------------------------------------------
    # ----------------------------------- execution functions -------------------------------------
    # ---------------------------------------------------------------------------------------------

    def run(self, *args, **kwargs):
        """Implementation of the abstract run() method of the :class:`threading.Thread`

        Should be filled with code, that should be executed for each container_state derivative.
        """
        raise NotImplementedError("The ContainerState.run() function has to be implemented!")

    def recursively_preempt_states(self):
        """ Preempt the state and all of it child states.
        """
        self.preempted = True
        for state_id, state in self.states.iteritems():
            state.recursively_preempt_states()

    def setup_run(self):
        """ Executes a generic set of actions that has to be called in the run methods of each derived state class.

        :return:
        """
        State.setup_run(self)
        # reset the scoped data
        self._scoped_data = {}
        self.add_default_values_of_scoped_variables_to_scoped_data()
        self.add_input_data_to_scoped_data(self.input_data)

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
                self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
                logger.debug("Exit hierarchy state %s with outcome aborted, as the child state returned "
                             "aborted and no transition was added to the aborted outcome!" % self.name)
                return None

            # preempted case for child state
            elif state.final_outcome.outcome_id == -2:
                if self.concurrency_queue:
                    self.concurrency_queue.put(self.state_id)
                self.final_outcome = Outcome(-2, "preempted")
                self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
                logger.debug("Exit hierarchy state %s with outcome preempted, as the child state returned "
                             "preempted and no transition was added to the preempted outcome!" % self.name)
                return None

            # preempted case
            if self.preempted:
                if self.concurrency_queue:
                    self.concurrency_queue.put(self.state_id)
                self.final_outcome = Outcome(-2, "preempted")
                self.state_execution_status = StateExecutionState.WAIT_FOR_NEXT_STATE
                logger.debug("Exit hierarchy state %s with outcome preempted, as the state itself "
                             "was preempted!" % self.name)
                return None

            # depending on the execution mode pause execution
            execution_signal = state_machine_execution_engine.handle_execution_mode(self)
            if execution_signal is StateMachineExecutionStatus.STOPPED:
                # this will be caught at the end of the run method
                raise RuntimeError("state stopped")

            # wait until the user connects the outcome of the state with a transition
            self._transitions_cv.acquire()
            self._transitions_cv.wait(3.0)
            self._transitions_cv.release()
            transition = self.get_transition_for_outcome(state, state.final_outcome)

        return transition

    def handle_no_start_state(self):
        """Handles the situation, when no start state exists during execution

        The method waits, until a transition is created. It then checks again for an existing start state and waits
        again, if this is not the case.
        """
        while self.get_start_state(set_final_outcome=True) is None:
            self._transitions_cv.acquire()
            self._transitions_cv.wait(3.0)
            self._transitions_cv.release()
        return self.get_start_state()

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
            state_id = state_id_generator()
        state = State(state_id, name)
        self._states[state_id] = state
        return state_id

    @Observable.observed
    def add_state(self, state, storage_load=False):
        """Adds a state to the container state.

        :param state: the state that is going to be added

        """
        assert isinstance(state, State)
        if not storage_load:
            # unmark path for removal: this is needed when a state with the same id is removed and added again in this state
            own_sm_id = state_machine_manager.get_sm_id_for_state(self)
            if own_sm_id is not None:
                global_storage.unmark_path_for_removal_for_sm_id(own_sm_id, state.script.path)

        # TODO: add validity checks for states and then remove this check
        if state.state_id in self._states.iterkeys():
            raise AttributeError("State id %s already exists in the container state", state.state_id)
        else:
            state.parent = self
            self._states[state.state_id] = state

    @Observable.observed
    def remove_state(self, state_id, recursive_deletion=True, force=True):
        """Remove a state from the container state.

        :param state_id: the id of the state to remove
        :param recursive_deletion: a flag to indicate a recursive deletion of all substates

        """
        if state_id not in self.states:
            raise AttributeError("State_id %s does not exist" % state_id)

        if state_id == self.start_state_id:
            self.set_start_state(None)

        # remove script folder
        own_sm_id = state_machine_manager.get_sm_id_for_state(self)
        if own_sm_id is None:
            logger.warn("Something is going wrong during state removal. State does not belong to "
                        "a state machine!")
        else:
            global_storage.mark_path_for_removal_for_sm_id(own_sm_id, self.states[state_id].script.path)

        #first delete all transitions and data_flows, which are connected to the state to be deleted
        keys_to_delete = []
        for key, transition in self.transitions.iteritems():
            if transition.from_state == state_id or transition.to_state == state_id:
                keys_to_delete.append(key)
        for key in keys_to_delete:
            self.remove_transition(key, True)

        keys_to_delete = []
        for key, data_flow in self.data_flows.iteritems():
            if data_flow.from_state == state_id or data_flow.to_state == state_id:
                keys_to_delete.append(key)
        for key in keys_to_delete:
            self.remove_data_flow(key)

        if recursive_deletion:
            # Recursively delete all transitions, data flows and states within the state to be deleted
            if isinstance(self.states[state_id], ContainerState):
                for child_state_id in self.states[state_id].states.keys():
                    self.states[state_id].remove_state(child_state_id, force=True)
                for transition_id in self.states[state_id].transitions.keys():
                    self.states[state_id].remove_transition(transition_id)
                for data_flow_id in self.states[state_id].data_flows.keys():
                    self.states[state_id].remove_data_flow(data_flow_id)

        # final delete the state it self
        del self.states[state_id]

    @Observable.observed
    def change_state_type(self, state_m, new_state_class):
        from awesome_tool.mvc.statemachine_helper import StateMachineHelper
        return StateMachineHelper.change_state_type(state_m, new_state_class)

    # @Observable.observed
    def set_start_state(self, state):
        """Sets the start state of a container state

        :param state: The state_id of a state or a direct reference ot he state (that was already added
                    to the container) that will be the start state of this container state.

        """
        if state is None:
            self.start_state_id = None
        elif isinstance(state, State):
            self.start_state_id = state.state_id
        else:
            self.start_state_id = state

    def get_start_state(self, set_final_outcome=False):
        """Get the start state of the container state

        """

        if self.get_path() in state_machine_execution_engine.start_state_paths:
            for state_id, state in self.states.iteritems():
                if state.get_path() in state_machine_execution_engine.start_state_paths:
                    logger.debug("Forward execution to state " + state.name)
                    return state

        if self.start_state_id is None:
            return None

        # It is possible to connect the income directly with an outcome
        if self.start_state_id == self.state_id:
            if set_final_outcome:
                for transition_id in self.transitions:
                    if self.transitions[transition_id].from_state is None:
                        to_outcome_id = self.transitions[transition_id].to_outcome
                        self.final_outcome = self.outcomes[to_outcome_id]
                        break
            return self

        return self.states[self.start_state_id]

    # ---------------------------------------------------------------------------------------------
    # ---------------------------------- transition functions -------------------------------------
    # ---------------------------------------------------------------------------------------------

    def check_transition_id(self, transition_id):
        """ Check the transition id and calculate a new one if its None

        :param transition_id: The transition-id to check
        :return: The new transition id
        """
        if transition_id is not None:
            if transition_id in self._transitions.iterkeys():
                raise AttributeError("The transition id %s already exists. Cannot add transition!", transition_id)
        else:
            transition_id = generate_transition_id()
            while transition_id in self._transitions.iterkeys():
                transition_id = generate_transition_id()
        return transition_id

    def basic_transition_checks(self, from_state_id, from_outcome, to_state_id, to_outcome, transition_id):
        pass

    def check_if_outcome_already_connected(self, from_state_id, from_outcome):
        """ check if outcome of from state is not already connected

        :param from_state_id: The source state of the transition
        :param from_outcome: The outcome of the source state to connect the transition to
        :return:
        """
        for trans_key, transition in self.transitions.iteritems():
            if transition.from_state == from_state_id:
                if transition.from_outcome == from_outcome:
                    raise AttributeError("Outcome %s of state %s is already connected" %
                                         (str(from_outcome), str(from_state_id)))

    def create_transition(self, from_state_id, from_outcome, to_state_id, to_outcome, transition_id):
        """ Creates a new transition.

        Lookout: Check the parameters first before creating a new transition

        :param from_state_id: The source state of the transition
        :param from_outcome: The outcome of the source state to connect the transition to
        :param to_state_id: The target state of the transition
        :param to_outcome: The target outcome of a container state
        :param transition_id: An optional transition id for the new transition
        :return:
        """

        # get correct states
        if from_state_id is not None:
            if from_state_id == self.state_id:
                from_state = self
            else:
                from_state = self.states[from_state_id]

        # finally add transition
        if from_outcome is not None:
            if from_outcome in from_state.outcomes:
                if to_outcome is not None:
                    if to_outcome in self.outcomes:  # if to_state is None then the to_outcome must be an outcome of self
                        self.transitions[transition_id] = \
                            Transition(from_state_id, from_outcome, to_state_id, to_outcome, transition_id, self)
                    else:
                        raise AttributeError("to_state does not have outcome %s", to_outcome)
                else:  # to outcome is None but to_state is not None, so the transition is valid
                    self.transitions[transition_id] = \
                        Transition(from_state_id, from_outcome, to_state_id, to_outcome, transition_id, self)
            else:
                raise AttributeError("from_state does not have outcome %s", from_state)
        else:
            self.transitions[transition_id] = \
                Transition(None, None, to_state_id, to_outcome, transition_id, self)

        # notify all states waiting for transition to be connected
        self._transitions_cv.acquire()
        self._transitions_cv.notify_all()
        self._transitions_cv.release()

        return transition_id

    @Observable.observed
    def add_transition(self, from_state_id, from_outcome, to_state_id, to_outcome, transition_id=None):
        """Adds a transition to the container state

        Note: Either the toState or the toOutcome needs to be "None"

        :param from_state_id: The source state of the transition
        :param from_outcome: The outcome id of the source state to connect the transition to
        :param to_state_id: The target state of the transition
        :param to_outcome: The target outcome id of a container state
        :param transition_id: An optional transition id for the new transition
        """

        transition_id = self.check_transition_id(transition_id)

        new_transition = Transition(from_state_id, from_outcome, to_state_id, to_outcome, transition_id, self)
        self.transitions[transition_id] = new_transition

        # notify all states waiting for transition to be connected
        self._transitions_cv.acquire()
        self._transitions_cv.notify_all()
        self._transitions_cv.release()
        # self.create_transition(from_state_id, from_outcome, to_state_id, to_outcome, transition_id)

        return transition_id

    def get_transition_for_outcome(self, state, outcome):
        """Determines the next transition of a state.

        :param state: The state for which the transition is determined
        :param outcome: The outcome of the state, that is given in the first parameter
        :return: the transition specified by the the state and the outcome
        """
        if not isinstance(state, State):
            raise TypeError("state must be of type State")
        if not isinstance(outcome, Outcome):
            raise TypeError("outcome must be of type Outcome")
        logger.debug("Return transition for state %s and outcome %s" % (state.name, outcome))
        result_transition = None
        for key, transition in self.transitions.iteritems():
            if transition.from_state == state.state_id and transition.from_outcome == outcome.outcome_id:
                result_transition = transition
        if result_transition is None and outcome.outcome_id >= 0:
            logger.warn("No transition found for state with name %s!" % self.name)
        return result_transition

    @Observable.observed
    def remove_transition(self, transition_id, force=False):
        """Removes a transition from the container state

        :param transition_id: the id of the transition to remove

        """
        if transition_id == -1 or transition_id == -2:
            raise AttributeError("The transition_id must not be -1 (Aborted) or -2 (Preempted)")
        if transition_id not in self._transitions:
            raise AttributeError("The transition_id %s does not exist" % str(transition_id))
        self._transitions.pop(transition_id, None)

    def is_valid_transition_id(self, transition_id):
        """Checks if transition_id valid type and points to element of state.

        :param int transition_id:
        :return:
        """
        #check if types are valid
        if not isinstance(transition_id, int):
            raise TypeError("transition_id must be of type int")
        # consistency check
        if transition_id not in self.transitions:
            raise AttributeError("transition_id %s has to be in container_state %s transitions-list" %
                                 (transition_id, self.state_id))

    def is_valid_data_flow_id(self, data_flow_id):
        """Checks if data_flow_id valid type and points to element of state.

        :param int data_flow_id:
        :return:
        """
        #check if types are valid
        if not isinstance(data_flow_id, int):
            raise TypeError("data_flow_id must be of type int")
        # consistency check
        if data_flow_id not in self.data_flows:
            raise AttributeError("data_flow_id %s has to be in container_state %s data_flows-list" %
                                 (data_flow_id, self.state_id))

    def is_valid_state_id(self, state_id):
        """Checks if state_id valid type and points to element of state.

        :param str state_id:
        :return:
        """
        #check if types are valid
        if not isinstance(state_id, str):
            raise TypeError("state_id must be of type str")
        # consistency check
        if state_id not in self.states:
            raise AttributeError("state_id %s has to be child of container_state %s" %
                                 (state_id, self.state_id))

    def modify_transition_from_state(self, transition_id, from_state, from_outcome):
        """The function accepts consistent transition changes of from_state with respective from_outcome.

        :param int transition_id: a valid transition_id of ContainerState.transitions
        :param str from_state: string of one of self.states-state_id's
        :param int from_outcome: the for respective from_state unique outcome_id
        """
        # validity checks
        self.is_valid_transition_id(transition_id)
        if from_state is not None:
            self.is_valid_state_id(from_state)
            self.states[from_state].is_valid_outcome_id(from_outcome)
        elif from_outcome is not None:
            raise AttributeError("from_outcome must be None id from_state is None")
        # set properties
        self.transitions[transition_id].modify_origin(from_state, from_outcome)

    def modify_transition_from_outcome(self, transition_id, from_outcome):
        """The function accepts consistent transition changes of from_outcome.

        :param int transition_id: a valid transition_id of ContainerState.transitions
        :param int from_outcome: the for respective from_state unique outcome_id
        """
        # validity checks
        self.is_valid_transition_id(transition_id)
        self.states[self.transitions[transition_id].from_state].is_valid_outcome_id(from_outcome)
        # set properties
        self.transitions[transition_id].from_outcome = from_outcome

    def modify_transition_to_state(self, transition_id, to_state):
        """The function accepts consistent transition changes of to_state.

        :param int transition_id: a valid transition_id of ContainerState.transitions
        :param str to_state: string of one of self.states-state_id's
        """
        # validity checks
        self.is_valid_transition_id(transition_id)
        self.is_valid_state_id(to_state)
        # set properties
        self.transitions[transition_id].to_state = to_state

    def modify_transition_to_outcome(self, transition_id, to_outcome):
        """The function accepts consistent transition changes of to_outcome.

        :param int transition_id: a valid transition_id of ContainerState.transitions
        :param int to_outcome: a in self existing outcome
        """
        # validity checks
        self.is_valid_transition_id(transition_id)
        self.is_valid_outcome_id(to_outcome)
        # set properties
        self.transitions[transition_id].to_outcome = to_outcome

    def remove_outcome_hook(self, outcome_id):
        """Removes internal transition going to the outcome
        """
        transition_ids_to_remove = []
        for transition_id, transition in self.transitions.iteritems():
            if transition.to_outcome == outcome_id and transition.to_state is None:
                transition_ids_to_remove.append(transition_id)

        for transition_id in transition_ids_to_remove:
            self.remove_transition(transition_id)

    # ---------------------------------------------------------------------------------------------
    # ----------------------------------- data-flow functions -------------------------------------
    # ---------------------------------------------------------------------------------------------
    # TODO input, output oder scope nicht auf sich selbst
    # TODO scope nicht auf andere scope
    # TODO output-in, input-in nur ein data flow
    # TODO data flows mit gleichen Attributen nur einmal

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
            while data_flow_id in self._data_flows.iterkeys():
                data_flow_id = generate_data_flow_id()

        self.data_flows[data_flow_id] = DataFlow(from_state_id, from_data_port_id, to_state_id, to_data_port_id,
                                                 data_flow_id, self)
        return data_flow_id

    @Observable.observed
    def remove_data_flow(self, data_flow_id):
        """ Removes a data flow from the container state

        :param int data_flow_id: the id of the data_flow to remove

        """
        self.is_valid_data_flow_id(data_flow_id)
        self.data_flows.pop(data_flow_id, None)

    def remove_data_flows_with_data_port_id(self, data_port_id):
        """Remove an data ports whose from_key or to_key equals the passed data_port_id

        :param int data_port_id: the id of a data_port of which all data_flows should be removed, the id can be a input or
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
        self._scoped_variables[scoped_variable_id] = ScopedVariable(name, data_type, default_value,
                                                                    scoped_variable_id, self)
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
        if self._scoped_variables[scoped_variable_id].name in self.__scoped_variables_names:
            self.__scoped_variables_names.remove(self._scoped_variables[scoped_variable_id].name)
        # delete scoped variable
        del self._scoped_variables[scoped_variable_id]

    def modify_scoped_variable_name(self, name, scoped_variable_id):
        if name in self.__scoped_variables_names:
            logger.warning("A scoped variable with name %s already exists", name)
            return
        self.__scoped_variables_names.remove(self._scoped_variables[scoped_variable_id].name)
        self._scoped_variables[scoped_variable_id].name = name
        self.__scoped_variables_names.append(name)

    # ---------------------------------------------------------------------------------------------
    # ---------------------------- scoped variables functions end ---------------------------------
    # ---------------------------------------------------------------------------------------------

    def get_outcome(self, state_id, outcome_id):
        if state_id == self.state_id:
            if outcome_id in self.outcomes:
                return self.outcomes[outcome_id]
        elif state_id in self.states:
            state = self.states[state_id]
            if outcome_id in state.outcomes:
                return state.outcomes[outcome_id]
        return None

    def get_data_port(self, state_id, port_id):
        """Searches for a data port

        The data port specified the the state id and data port id is searched in the state itself and in its children.

        :param str state_id: The id of the state the port is in
        :param int port_id:  The id of the port
        :return: The searched port or None if it is not found
        """
        if state_id == self.state_id:
            return self.get_data_port_by_id(port_id)
        for child_state_id, child_state in self.states.iteritems():
            if state_id != child_state_id:
                continue
            port = child_state.get_data_port_by_id(port_id)
            if port:
                return port
        return None

    def get_data_port_by_id(self, data_port_id):
        """Search for the given data port id in the data ports of the state

        The method tries to find a data port in the input and output data ports as well as in the scoped variables.
        :param data_port_id: the unique id of the data port
        :return: the data port with the searched id or None if not found
        """
        data_port = super(ContainerState, self).get_data_port_by_id(data_port_id)
        if data_port:
            return data_port
        if data_port_id in self.scoped_variables:
            return self.scoped_variables[data_port_id]
        return None

    # ---------------------------------------------------------------------------------------------
    # ---------------------------------- input data handling --------------------------------------
    # ---------------------------------------------------------------------------------------------

    def get_inputs_for_state(self, state):
        """Get all input data of an state

        :param state: the state of which the input data is determined
        :return: the input data of the target state
        """
        result_dict = {}

        tmp_dict = self.get_default_input_values_for_state(state)
        result_dict.update(tmp_dict)

        for input_port_key, value in state.input_data_ports.iteritems():
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

    # ---------------------------------------------------------------------------------------------
    # ---------------------------- functions to modify the scoped data ----------------------------
    # ---------------------------------------------------------------------------------------------

    def add_input_data_to_scoped_data(self, dictionary):
        """Add a dictionary to the scoped data

        As the input_data dictionary maps names to values, the functions looks for the proper data_ports keys in the
        input_data_ports dictionary

        :param dictionary: The dictionary that is added to the scoped data
        :param state: The state to which the input_data was passed (should be self in most cases)

        """
        for dict_key, value in dictionary.iteritems():
            for input_data_port_key, data_port in self.input_data_ports.iteritems():
                if dict_key == data_port.name:
                    self.scoped_data[str(input_data_port_key)+self.state_id] = \
                        ScopedData(data_port.name, value, type(value), self.state_id, DataPortType.INPUT)
                    # forward the data to scoped variables
                    for data_flow_key, data_flow in self.data_flows.iteritems():
                        if data_flow.from_key == input_data_port_key and data_flow.from_state == self.state_id:
                            if data_flow.to_state == self.state_id:
                                current_scoped_variable = self.scoped_variables[data_flow.to_key]
                                self.scoped_data[str(data_flow.to_key)+self.state_id] = \
                                    ScopedData(current_scoped_variable.name, value, type(value), self.state_id,
                                               DataPortType.SCOPED)

    def add_state_execution_output_to_scoped_data(self, dictionary, state):
        """Add a state execution output to the scoped data

        :param dictionary: The dictionary that is added to the scoped data
        :param state: The state that finished execution and provide the dictionary
        """
        for output_name, value in dictionary.iteritems():
            for output_data_port_key, data_port in state.output_data_ports.iteritems():
                if output_name == data_port.name:
                    self.scoped_data[str(output_data_port_key)+state.state_id] = \
                        ScopedData(data_port.name, value, type(value), state.state_id, DataPortType.OUTPUT)

    def add_default_values_of_scoped_variables_to_scoped_data(self):
        """Add the scoped variables default values to the scoped_data dictionary

        """
        for key, scoped_var in self.scoped_variables.iteritems():
            self.scoped_data[str(scoped_var.data_port_id)+self.state_id] = \
                ScopedData(scoped_var.name, scoped_var.default_value, scoped_var.data_type, self.state_id,
                           DataPortType.SCOPED)

    def update_scoped_variables_with_output_dictionary(self, dictionary, state):
        """Update the values of the scoped variables with the output dictionary of a specific state.

        :param: the dictionary to update the scoped variables with
        :param: the state the output dictionary belongs to

        """
        for key, value in dictionary.iteritems():
            output_data_port_key = None
            # search for the correct output data port key of the source state
            for o_key, o_port in state.output_data_ports.iteritems():
                if o_port.name == key:
                    output_data_port_key = o_key
                    break
            if output_data_port_key is None:
                if not key == "error":
                    logger.warning("Output variable %s was written during state execution, "
                                   "that has no data port connected to it.", str(key))
            for data_flow_key, data_flow in self.data_flows.iteritems():
                if data_flow.from_key == output_data_port_key and data_flow.from_state == state.state_id:
                    if data_flow.to_state == self.state_id:  # is target of data flow own state id?
                        if data_flow.to_key in self.scoped_variables.iterkeys():  # is target data port scoped?
                            current_scoped_variable = self.scoped_variables[data_flow.to_key]
                            self.scoped_data[str(data_flow.to_key) + self.state_id] = \
                                ScopedData(current_scoped_variable.name, value, type(value), state.state_id,
                                           DataPortType.SCOPED)

    # ---------------------------------------------------------------------------------------------
    # ------------------------ functions to modify the scoped data end ----------------------------
    # ---------------------------------------------------------------------------------------------

    def change_state_id(self, state_id=None):
        """
        Changes the id of the state to a new id. This functions replaces the old state_id with the new state_id in all
        data flows and transitions.
        :param state_id: The new state if of the state
        :return:
        """
        old_state_id = self.state_id
        State.change_state_id(self, state_id)
        while self.state_id == old_state_id:
            old_state_id = self.state_id
            State.change_state_id()

        # change id in all transitions
        for trans_id, transition in self.transitions.iteritems():
            if transition.from_state == old_state_id:
                transition.from_state = self.state_id
            if transition.to_state == old_state_id:
                transition.to_state = self.state_id

        # change id in all data_flows
        for df_id, data_flow in self.data_flows.iteritems():
            if data_flow.from_state == old_state_id:
                data_flow.from_state = self.state_id
            if data_flow.to_state == old_state_id:
                data_flow.to_state = self.state_id

    def state_id_exists(self, new_state_id):
        """
        Checks if a specific key already exists among the child states.
        :param new_state_id: the state id to check
        :return: True if the key already exists, False else.
        """
        for state_id in self.states.keys():
            if state_id == new_state_id:
                return True
        return False

    def get_state_for_transition(self, transition):
        """Calculate the target state of a transition

        :param transition: The transition of which the target state is determined

        """
        if not isinstance(transition, Transition):
            raise TypeError("transition must be of type Transition")
        # the to_state is None when the transition connects an outcome of a child state to the outcome of a parent state
        if transition.to_state == self.state_id or transition.to_state is None:
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
                        scoped_data_key = str(data_flow.from_key)+data_flow.from_state
                        if scoped_data_key in self.scoped_data.iterkeys():
                            self.output_data[output_name] = \
                                copy.deepcopy(self.scoped_data[scoped_data_key].value)
                        else:
                            if not self.backward_execution:
                                logger.error("Output data with name %s was not found in the scoped data. "
                                             "This normally means a statemachine design error", output_name)

    # yaml part
    def get_container_state_yaml_dict(data):
        dict_representation = {
            'name': data.name,
            'state_id': data.state_id,
            'description': data.description,
            'input_data_ports': data.input_data_ports,
            'output_data_ports': data.output_data_ports,
            'outcomes': data.outcomes,
            'path': data.script.path,
            'filename': data.script.filename,
            'transitions': data.transitions,
            'data_flows': data.data_flows,
            'scoped_variables': data.scoped_variables
        }
        return dict_representation

    def __str__(self):
        return "{0} [{1} child states]".format(State.__str__(self), len(self.states))

    # ---------------------------------------------------------------------------------------------
    # -------------------------------------- check methods ---------------------------------------
    # ---------------------------------------------------------------------------------------------

    def check_child_validity(self, child):
        """Check validity of passed child object

        The method is called by state child objects (transitions, data flows) when these are initialized or changed. The
        method checks the type of the child and then checks its validity in the context of the state.

        :param object child: The child of the state that is to be tested
        :return bool validity, str message: validity is True, when the child is valid, False else. message gives more
            information especially if the child is not valid
        """
        # First let the state do validity checks for outcomes and data ports
        valid, message = super(ContainerState, self).check_child_validity(child)
        if not valid and message != "no valid child type":
            return False, message
        # Continue with checks if previous ones did not fail
        # Check type of child and call appropriate validity test
        if isinstance(child, DataFlow):
            return self._check_data_flow_validity(child)
        if isinstance(child, Transition):
            return self._check_transition_validity(child)
        return valid, message

    def check_data_port_connection(self, check_data_port):
        """Checks the connection validity of a data port

        The method is called by a child state to check the validity of a data port in case it is connected with data
        flows. The data port does not belong to 'self', but to one of self.states.
        If the data port is connected to a data flow, the method checks, whether these connect consistent data types
        of ports.
        :param awesome_tool.statemachine.data_port.DataPort check_data_port: The port to check
        :return: valid, message
        """
        for data_flow in self.data_flows.itervalues():
            # Check whether the data flow connects the given port
            from_port = self.get_data_port(data_flow.from_state, data_flow.from_key)
            to_port = self.get_data_port(data_flow.to_state, data_flow.to_key)
            if check_data_port is from_port or check_data_port is to_port:
                if not type_inherits_of_type(from_port.data_type, to_port.data_type):
                    return False, "Connection of two non-compatible data types"
        return True, "valid"

    def _check_data_flow_validity(self, check_data_flow):
        """Checks the validity of a data flow

        Calls further checks to inspect the id, ports and data types.

        :param awesome_tool.statemachine.data_flow.DataFlow check_data_flow: The data flow to be checked
        :return bool validity, str message: validity is True, when the data flow is valid, False else. message gives
            more information especially if the data flow is not valid
        """
        valid, message = self._check_data_flow_id(check_data_flow)
        if not valid:
            return False, message

        valid, message = self._check_data_flow_ports(check_data_flow)
        if not valid:
            return False, message

        return self._check_data_flow_types(check_data_flow)

    def _check_data_flow_id(self, data_flow):
        """Checks the validity of a data flow id

        Checks whether the id of the given data flow is already by anther data flow used within the state.

        :param awesome_tool.statemachine.data_flow.DataFlow data_flow: The data flow to be checked
        :return bool validity, str message: validity is True, when the data flow is valid, False else. message gives
            more information especially if the outcome is not valid
        """
        data_flow_id = data_flow.data_flow_id
        if data_flow_id in self.data_flows and data_flow is not self.data_flows[data_flow_id]:
            return False, "data_flow_id already existing"
        return True, "valid"

    def _check_data_flow_ports(self, data_flow):
        """Checks the validity of the ports of a data flow

        Checks whether the ports of a data flow are existing and whether it is allowed to connect these ports.

        :param awesome_tool.statemachine.data_flow.DataFlow data_flow: The data flow to be checked
        :return bool validity, str message: validity is True, when the data flow is valid, False else. message gives
            more information especially if the data flow is not valid
        """
        from_state_id = data_flow.from_state
        to_state_id = data_flow.to_state
        from_data_port_id = data_flow.from_key
        to_data_port_id = data_flow.to_key

        # Check whether to and from port are existing
        from_data_port = self.get_data_port(from_state_id, from_data_port_id)
        if not from_data_port:
            return False, "Data flow origin not existing"
        to_data_port = self.get_data_port(to_state_id, to_data_port_id)
        if not to_data_port:
            return False, "Data flow target not existing"

        # Check, whether the origin of the data flow is valid
        if from_state_id == self.state_id:  # data_flow originates in container state
            if from_data_port_id not in self.input_data_ports and from_data_port_id not in self.scoped_variables:
                return False, "Data flow origin port must be an input port or scoped variable, when the data flow " \
                              "starts in the parent state"
        else:  # data flow originates in child state
            if from_data_port_id not in from_data_port.parent.output_data_ports:
                return False, "Data flow origin port must be an output port, when the data flow " \
                              "starts in the child state"

        # Check, whether the target of a data flow is valid
        if to_state_id == self.state_id:  # data_flow ends in container state
            if to_data_port_id not in self.output_data_ports and to_data_port_id not in self.scoped_variables:
                return False, "Data flow target port must be an output port or scoped variable, when the data flow " \
                              "goes to the parent state"
        else:  # data_flow ends in child state
            if to_data_port_id not in to_data_port.parent.input_data_ports:
                return False, "Data flow target port must be an input port, when the data flow goes to a child state"

        # Check, whether the target port is already connected
        for existing_data_flow in self.data_flows.itervalues():
            to_data_port_existing = self.get_data_port(existing_data_flow.from_state, existing_data_flow.from_key)
            if to_data_port is to_data_port_existing and data_flow is not existing_data_flow:
                # Scoped variables are an exception, they can be connected several times
                if not to_data_port.parent or to_data_port_id not in to_data_port.parent.scoped_variables:
                    return False, "Data flow target is already connected to another data flow"

        return True, "valid"

    def _check_data_flow_types(self, check_data_flow):
        """Checks the validity of the data flow connection

        Checks whether the ports of a data flow have matching data types.

        :param awesome_tool.statemachine.data_flow.DataFlow check_data_flow: The data flow to be checked
        :return bool validity, str message: validity is True, when the data flow is valid, False else. message gives
            more information especially if the data flow is not valid
        """
        # Check whether the data types or origin and target fit
        from_data_port = self.get_data_port(check_data_flow.from_state, check_data_flow.from_key)
        to_data_port = self.get_data_port(check_data_flow.to_state, check_data_flow.to_key)
        if not type_inherits_of_type(from_data_port.data_type, to_data_port.data_type):
            return False, "Data flow origin and target do not have matching data types"
        return True, "valid"

    def _check_transition_validity(self, check_transition):
        """Checks the validity of a transition

        Calls further checks to inspect the id, origin, target and connection of the transition.

        :param awesome_tool.statemachine.transition.Transition check_transition: The transition to be checked
        :return bool validity, str message: validity is True, when the transition is valid, False else. message gives
            more information especially if the transition is not valid
        """
        valid, message = self._check_transition_id(check_transition)
        if not valid:
            return False, message

        # Separate check for start transitions
        if check_transition.from_state is None:
            return self._check_start_transition(check_transition)

        valid, message = self._check_transition_origin(check_transition)
        if not valid:
            return False, message

        valid, message = self._check_transition_target(check_transition)
        if not valid:
            return False, message

        return self._check_transition_connection(check_transition)

    def _check_transition_id(self, transition):
        """Checks the validity of a transition id

        Checks whether the transition id is already used by another transition within the state

        :param awesome_tool.statemachine.transition.Transition transition: The transition to be checked
        :return bool validity, str message: validity is True, when the transition is valid, False else. message gives
            more information especially if the transition is not valid
        """
        transition_id = transition.transition_id
        if transition_id in self.transitions and transition is not self.transitions[transition_id]:
            return False, "transition_id already existing"
        return True, "valid"

    def _check_start_transition(self, start_transition):
        """Checks the validity of a start transition

        Checks whether the given transition is a start transition a whether it is the only one within the state.

        :param awesome_tool.statemachine.transition.Transition start_transition: The transition to be checked
        :return bool validity, str message: validity is True, when the transition is valid, False else. message gives
            more information especially if the transition is not valid
        """
        for transition in self.transitions.itervalues():
            if transition.from_state is None:
                if start_transition is not transition:
                    return False, "Only one start transition is allowed"

        if start_transition.from_outcome is not None:
            return False, "from_outcome must not be set in start transition"

        return self._check_transition_target(start_transition)

    def _check_transition_target(self, transition):
        """Checks the validity of a transition target

        Checks whether the transition target is valid.

        :param awesome_tool.statemachine.transition.Transition transition: The transition to be checked
        :return bool validity, str message: validity is True, when the transition is valid, False else. message gives
            more information especially if the transition is not valid
        """

        to_state_id = transition.to_state
        to_outcome_id = transition.to_outcome

        if to_state_id == self.state_id:
            if to_outcome_id not in self.outcomes:
                return False, "to_outcome is not existing"
        else:
            if to_state_id not in self.states:
                return False, "to_state is not existing"
            if to_outcome_id is not None:
                return False, "to_outcome must be None as transition goes to child state"

        return True, "valid"

    def _check_transition_origin(self, transition):
        """Checks the validity of a transition origin

        Checks whether the transition origin is valid.

        :param awesome_tool.statemachine.transition.Transition transition: The transition to be checked
        :return bool validity, str message: validity is True, when the transition is valid, False else. message gives
            more information especially if the transition is not valid
        """
        from_state_id = transition.from_state
        from_outcome_id = transition.from_outcome

        if from_state_id != self.state_id and from_state_id not in self.states:
            return False, "from_state not existing"

        from_outcome = self.get_outcome(from_state_id, from_outcome_id)
        if from_outcome is None:
            return False, "from_outcome not existing in from_state"

        return True, "valid"

    def _check_transition_connection(self, check_transition):
        """Checks the validity of a transition connection

        Checks whether the transition is allowed to connect the origin with the target.

        :param awesome_tool.statemachine.transition.Transition check_transition: The transition to be checked
        :return bool validity, str message: validity is True, when the transition is valid, False else. message gives
            more information especially if the transition is not valid
        """
        from_state_id = check_transition.from_state
        from_outcome_id = check_transition.from_outcome
        to_state_id = check_transition.to_state
        to_outcome_id = check_transition.to_outcome

        # check for connected origin
        for transition in self.transitions.itervalues():
            if transition.from_state == from_state_id:
                if transition.from_outcome == from_outcome_id:
                    if check_transition is not transition:
                        return False, "transition origin already connected to another transition"

        if from_state_id in self.states and to_state_id in self.states and to_outcome_id is not None:
            return False, "no transition from one outcome to another one on the same hierarchy allowed"

        return True, "valid"

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
        # First safely remove all existing states (recursively!), as they will be replaced
        state_ids = self.states.keys()
        for state_id in state_ids:
            self.remove_state(state_id)
        if states is not None:
            if not isinstance(states, dict):
                raise TypeError("states must be of type dict")
            for state in states.itervalues():
                self.add_state(state)
            #     if not isinstance(state, State):
            #         raise TypeError("element of container_state.states must be of type State")
            #     state.parent = self
            # self._states = states

    @property
    def transitions(self):
        """Property for the _transitions field

        """
        return self._transitions

    @transitions.setter
    @Observable.observed
    def transitions(self, transitions):
        # First safely remove all existing transitions, as they will be replaced
        transition_ids = self.transitions.keys()
        for transition_id in transition_ids:
            self.remove_transition(transition_id)
        if transitions is not None:
            if not isinstance(transitions, dict):
                raise TypeError("transitions must be of type dict")
            for transition in transitions.itervalues():
                if not isinstance(transition, Transition):
                    raise TypeError("element of transitions must be of type Transition")
                transition.parent = self
            self._transitions = transitions

    @property
    def data_flows(self):
        """Property for the _data_flows field

        """
        return self._data_flows

    @data_flows.setter
    @Observable.observed
    def data_flows(self, data_flows):
        # First safely remove all existing data flows, as they will be replaced
        data_flow_ids = self.data_flows.keys()
        for data_flow_id in data_flow_ids:
            self.remove_data_flow(data_flow_id)
        if data_flows is not None:
            if not isinstance(data_flows, dict):
                raise TypeError("data_flows must be of type dict")
            for data_flow in data_flows.itervalues():
                if not isinstance(data_flow, DataFlow):
                    raise TypeError("element of data_flows must be of type DataFlow")
                data_flow.parent = self
            self._data_flows = data_flows

    @property
    def start_state_id(self):
        """Returns the id of the state first executed within the container

        :return: The if of the start state
        """
        for transition_id in self.transitions:
            if self.transitions[transition_id].from_state is None:
                to_state = self.transitions[transition_id].to_state
                if to_state is not None:
                    return to_state
                else:
                    return self.state_id
        return None

    @start_state_id.setter
    # @Observable.observed
    def start_state_id(self, start_state_id, to_outcome=None):
        """Set the start state of the container state

        The start state is the state to which the first transition goes to. Therefore the method creates a unique
        first transition to the state with the given id. Existing first transitions are removed. If the given state
        id is None, the first transition is removed.

        :param start_state_id: The state id of the state which should be executed first in the Container state
        """
        if start_state_id is not None and start_state_id not in self.states:
            raise ValueError("start_state_id does not exist")

        if start_state_id is None and to_outcome is not None:
            if to_outcome not in self.outcomes:
                raise ValueError("to_outcome does not exist")
            if start_state_id != self.state_id:
                raise ValueError("to_outcome defined but start_state_id is not state_id")

        # First we remove the transition to the start state
        for transition_id in self.transitions:
            if self.transitions[transition_id].from_state is None:
                # If the current start state is the same as the old one, we don't have to do anything
                if self.transitions[transition_id].to_state == start_state_id:
                    return
                self.remove_transition(transition_id)
                break
        if start_state_id is not None:
            self.add_transition(None, None, start_state_id, to_outcome)

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
                raise TypeError("scoped_variables must be of type dict")
            for scoped_variable in scoped_variables.itervalues():
                if not isinstance(scoped_variable, ScopedVariable):
                    raise TypeError("element of scope must be of type ScopedVariable")
                scoped_variable.parent = self
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
        for key, s in scoped_data.iteritems():
            if not isinstance(s, ScopedData):
                raise TypeError("element of scoped_data must be of type ScopedData")
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

    @property
    def child_execution(self):
        """Property for the _child_execution field

        """
        if self.state_execution_status is StateExecutionState.EXECUTE_CHILDREN:
            return True
        else:
            return False

            # @child_execution.setter
            # @Observable.observed
            # def child_execution(self, child_execution):
            #     if child_execution is not None:
            #         if not isinstance(child_execution, bool):
            #             raise TypeError("child_execution must be of type str")
            #     self._child_execution = child_execution
