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

from awesome_tool.utils import log
logger = log.get_logger(__name__)
from awesome_tool.statemachine.enums import DataPortType, StateExecutionState
from awesome_tool.statemachine.script import Script, ScriptType
from awesome_tool.statemachine.states.state import State
from awesome_tool.statemachine.transition import Transition
from awesome_tool.statemachine.outcome import Outcome
from awesome_tool.statemachine.data_flow import DataFlow
from awesome_tool.statemachine.scope import ScopedData, ScopedVariable
from awesome_tool.statemachine.id_generator import *
from awesome_tool.statemachine.config import *
from awesome_tool.statemachine.validity_check.validity_checker import ValidityChecker
import awesome_tool.statemachine.singleton


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

        self._states = None
        self.states = states
        self._transitions = None
        self.transitions = transitions
        self._data_flows = None
        self.data_flows = data_flows
        if start_state_id is not None:
            self.start_state_id = start_state_id
        self._scoped_variables = None
        self.scoped_variables = scoped_variables
        self.__scoped_variables_names = []
        self._scoped_data = {}
        # reference to an object that checks the validity of this container state
        self._v_checker = v_checker
        self._current_state = None
        # condition variable to wait for not connected states
        self._transitions_cv = Condition()
        self._child_execution = False
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
        # print "---------------------- scoped data 1-----------------------"
        # for key, value in self.scoped_data.iteritems():
        #     print key, value
        self.add_default_values_of_scoped_variables_to_scoped_data()
        self.add_input_data_to_scoped_data(self.input_data)

    def enter(self, scoped_variables_dict, backward_execution=False):
        """Called on entering the container state

        Here initializations of scoped variables and modules that are supposed to be used by the children take place.
        This method calls the custom entry function provided by a python script.

        :param scoped_variables_dict: a dictionary of all scoped variables that are passed to the custom function
        """
        self.state_execution_status = StateExecutionState.ENTER
        logger.debug("Calling enter() script of container state with name %s (backward: %s)",
                     self.name, backward_execution)
        self.script.load_and_build_module()
        self.script.enter(self, scoped_variables_dict, backward_execution)

    def exit(self, scoped_variables_dict, backward_execution=False):
        """Called on exiting the container state

        Clean up code for the state and its variables is executed here. This method calls the custom exit function
        provided by a python script.
        :param scoped_variables_dict: a dictionary of all scoped variables that are passed to the custom function
        """
        self.state_execution_status = StateExecutionState.EXIT
        logger.debug("Calling exit() script of container state with name %s (backward: %s)",
                     self.name, backward_execution)
        self.script.load_and_build_module()
        self.script.exit(self, scoped_variables_dict, backward_execution)

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
            execution_signal = awesome_tool.statemachine.singleton.state_machine_execution_engine.handle_execution_mode(self)
            if execution_signal == "stop":
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
    def add_state(self, state):
        """Adds a state to the container state.

        :param state: the state that is going to be added

        """

        # unmark path for removal: this is needed when a state with the same id is removed and added again in this state
        own_sm_id = awesome_tool.statemachine.singleton.state_machine_manager.get_sm_id_for_state(self)
        if own_sm_id is not None:
            awesome_tool.statemachine.singleton.global_storage.unmark_path_for_removal_for_sm_id(
                own_sm_id, state.script.path)

        if state.state_id in self._states.iterkeys():
            raise AttributeError("State id %s already exists in the container state", state.state_id)
        else:
            state.parent = self
            self._states[state.state_id] = state

    @Observable.observed
    def remove_state(self, state_id, recursive_deletion=True):
        """Remove a state from the container state.

        :param state_id: the id of the state to remove

        """
        if state_id not in self.states:
            raise AttributeError("State_id %s does not exist" % state_id)

        if state_id == self.start_state_id:
            self.set_start_state(None)

        # remove script folder
        own_sm_id = awesome_tool.statemachine.singleton.state_machine_manager.get_sm_id_for_state(self)
        if own_sm_id is None:
            logger.warn("Something is going wrong during state removal. State does not belong to "
                               "a state machine!")
        else:
            awesome_tool.statemachine.singleton.global_storage.mark_path_for_removal_for_sm_id(own_sm_id,
                                                                                  self.states[state_id].script.path)

        #first delete all transitions and data_flows, which are connected to the state to be deleted
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

        if recursive_deletion:
            # Recursively delete all transitions, data flows and states within the state to be deleted
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
    def change_state_type(self, state_m, new_state_class):
        from awesome_tool.mvc.statemachine_helper import StateMachineHelper
        return StateMachineHelper.change_state_type(state_m, new_state_class)

    # @Observable.observed
    def set_start_state(self, state):
        """Sets the start state of a container state

        :param state: The state_id of a state or a direct reference ot he state (that was already added
                    to the container) that will be the start state of this container state.

        """
        if isinstance(state, State):
            self.start_state_id = state.state_id
        else:
            self.start_state_id = state

    def get_start_state(self, set_final_outcome=False):
        """Get the start state of the container state

        """
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
        if transition_id is not None:
            if transition_id in self._transitions.iterkeys():
                raise AttributeError("The transition id %s already exists. Cannot add transition!", transition_id)
        else:
            transition_id = generate_transition_id()
            while transition_id in self._transitions.iterkeys():
                transition_id = generate_transition_id()

        # Check if transition is starting transition and the start state is already defined
        if from_state_id is None and self.start_state_id is not None:
            raise AttributeError("The start state is already defined: {0}".format(self.get_start_state().name))

        # check if states are existing
        if from_state_id is not None and not (from_state_id in self.states or from_state_id == self.state_id):
            raise AttributeError("From_state_id {0} does not exist in the container state".format(from_state_id))

        if to_state_id is not None:
            if not (to_state_id in self.states or to_state_id == self.state_id):
                raise AttributeError("To_state {0} does not exist in the container state".format(to_state_id))

        if to_state_id is None and to_outcome is None:
            raise AttributeError("Either the to_state_id or the to_outcome must be None")

        # get correct states
        if from_state_id is not None:
            if from_state_id == self.state_id:
                from_state = self
            else:
                from_state = self.states[from_state_id]

        if to_state_id is None and to_outcome is None:
            raise AttributeError("Either to_state_id or to_outcome must not be None")

        # check if outcome of from state is not already connected
        for trans_key, transition in self.transitions.iteritems():
            if transition.from_state == from_state_id:
                if transition.from_outcome == from_outcome:
                    raise AttributeError("outcome %s of state %s is already connected" %
                                         (str(from_outcome), str(from_state_id)))

        from awesome_tool.statemachine.states.concurrency_state import ConcurrencyState
        # check if state is a concurrency state, in concurrency states only transitions to the parents are allowed
        if isinstance(self, ConcurrencyState):
            if to_state_id is not None:  # None means that the target state is the containing state
                raise AttributeError("In concurrency states the to_state must be the container state itself")

        # finally add transition
        if from_outcome is not None:
            if from_outcome in from_state.outcomes:
                if to_outcome is not None:
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
        else:
            self.transitions[transition_id] =\
                Transition(None, None, to_state_id, to_outcome, transition_id)

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
            raise TypeError("state must be of type State")
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
        if data_flow_id is not None:
            if data_flow_id in self._data_flows.iterkeys():
                raise AttributeError("The data flow id %s already exists. Cannot add data flow!", data_flow_id)
        else:
            data_flow_id = generate_data_flow_id()
            while data_flow_id in self._data_flows.iterkeys():
                data_flow_id = generate_data_flow_id()

        if not (from_state_id in self.states or from_state_id == self.state_id):
            raise AttributeError("From_state_id %s does not exist in the container state" % from_state_id.state_id)
        if not (to_state_id in self.states or to_state_id == self.state_id):
            raise AttributeError("To_state %s does not exit in the container state" % to_state_id.state_id)

        if from_state_id == self.state_id:  # data_flow originates in container state
            from_state = self
            if from_data_port_id in from_state.scoped_variables:
                from_data_port = from_state.scoped_variables[from_data_port_id]
            elif from_data_port_id in from_state.input_data_ports:
                from_data_port = from_state.input_data_ports[from_data_port_id]
            else:
                raise AttributeError("from_data_port_id not in scoped_variables or input_data_ports")
        else:  # data flow originates in child state
            from_state = self.states[from_state_id]
            if from_data_port_id in from_state.output_data_ports:
                from_data_port = from_state.output_data_ports[from_data_port_id]
            else:
                raise AttributeError("from_data_port_id not in output_data_ports")

        if to_state_id == self.state_id:  # data_flow ends in container state
            to_state = self
            if to_data_port_id in to_state.scoped_variables:
                to_data_port = to_state.scoped_variables[to_data_port_id]
            elif to_data_port_id in to_state.output_data_ports:
                to_data_port = to_state.output_data_ports[to_data_port_id]
            else:
                raise AttributeError("to_data_port_id not in scoped_variables or output_data_ports")
        else:  # data_flow ends in child state
            to_state = self.states[to_state_id]
            if to_data_port_id in to_state.input_data_ports:
                to_data_port = to_state.input_data_ports[to_data_port_id]
            else:
                raise AttributeError("to_data_port_id not in input_data_ports")

        # check if to_dataport_id of to_state has already a data_flow
        for flow_id, data_flow in self.data_flows.iteritems():
            # scoped variables are allowed to have several data_flows connecte to them
            if data_flow.to_state == to_state_id and not data_flow.to_state == self.state_id:
                if data_flow.to_key == to_data_port_id:
                    raise AttributeError("port %s of state %s already has a connection" %
                                         (str(to_data_port_id), str(to_state_id)))

        # check if the data types of the tow ports are the same
        if not from_data_port.data_type == to_data_port.data_type:
            raise AttributeError("The from data port and the to data port do not have the same data type (%s and %s)" %
                                 (str(from_data_port.data_type), str(to_data_port.data_type)))

        self.data_flows[data_flow_id] = DataFlow(from_state_id, from_data_port_id, to_state_id, to_data_port_id, data_flow_id)
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
            # del self.data_flows[data_flow_id]

    def modify_data_flow_from_state(self, data_flow_id, from_state, from_key):
        """The function accepts consistent data_flow changes of from_state with respective from_key.

        :param int data_flow_id: a valid data_flow_id of ContainerState.data_flows
        :param str from_state: string of this state- or one of its child-state-state_id
        :param int from_key: the for respective from_state unique data_port_id
        """
        #check if types are valid
        self.is_valid_data_flow_id(data_flow_id)
        if from_state is not None and not type(from_state) == str:
            raise TypeError("from_state must be of type str")
        if from_key is not None and not type(from_key) == int:
            raise TypeError("from_key must be of type int")

        # consistency check
        if from_state == self.state_id:
            if not (from_key in self.input_data_ports or from_key in self.scoped_variables):
                raise AttributeError("from_key must be in list of output_data_ports or scoped_variables")
        else:  # child
            if not from_state in self.states:
                raise AttributeError("from_state must be in list of child-states")
            if not from_key in self.states[from_state].output_data_ports:
                raise AttributeError("from_key must be in list of child-state output_data_ports")
        # set properties
        self.data_flows[data_flow_id].modify_origin(from_state, from_key)

    def modify_data_flow_from_key(self, data_flow_id, from_key):
        """The function accepts consistent data_flow change of from_key.

        :param int data_flow_id: a valid data_flow_id of ContainerState.data_flows
        :param int from_key: the for respective from_state unique data_port_id
        """
        #check if type is valid
        self.is_valid_data_flow_id(data_flow_id)
        if from_key is not None and not type(from_key) == int:
            raise TypeError("from_key must be of type int")

        # consistency check
        from_state = self.data_flows[data_flow_id].from_state
        if from_state == self.state_id:
            if not (from_key in self.input_data_ports or from_key in self.scoped_variables):
                raise AttributeError("from_key must be in list of input_data_ports or scoped_variables")
        else:  # child
            if not from_key in self.states[from_state].output_data_ports:
                raise AttributeError("from_key must be in list of child-state input_data_ports")

        # set property
        self.data_flows[data_flow_id].from_key = from_key

    def modify_data_flow_to_state(self, data_flow_id, to_state, to_key):
        """The function accepts consistent data_flow changes of to_state with respective to_key.

        :param int data_flow_id: a valid data_flow_id of ContainerState.data_flows
        :param str to_state: string of this state- or one of its child-state-state_id
        :param int to_key: the for respective to_state unique data_port_id
        """
        # check if types are valid
        self.is_valid_data_flow_id(data_flow_id)
        if not type(to_state) == str:
            raise TypeError("to_state must be of type str")
        if not type(to_key) == int:
            raise TypeError("to_key must be of type int")

        # consistency check
        if to_state == self.state_id:
            if not (to_key in self.scoped_variables or to_key in self.output_data_ports):
                print to_key, self.scoped_variables.keys(), self.input_data_ports.keys()
                raise AttributeError("to_key must be in list of child-state input_data_ports or own scoped_variables")
        else:  # child
            if not to_state in self.states:
                raise AttributeError("to_state must be in list of child-states")
            if not to_key in self.states[to_state].input_data_ports:
                raise AttributeError("to_key must be in list of child-state input_data_ports")

        # set properties
        self.data_flows[data_flow_id].modify_target(to_state, to_key)

    def modify_data_flow_to_key(self, data_flow_id, to_key):
        """The function accepts consistent data_flow change of to_key.

        :param int data_flow_id: a valid data_flow_id of ContainerState.data_flows
        :param int to_key: the for respective to_state unique data_port_id
        """
        # check if type is valid
        self.is_valid_data_flow_id(data_flow_id)
        if not type(to_key) == int:
            raise TypeError("from_key must be of type int")

        # consistency check
        to_state = self.data_flows[data_flow_id].to_state
        if to_state == self.state_id:
            if not (to_key in self.output_data_ports or to_key in self.scoped_variables):
                raise AttributeError("to_key must be in list of child-state output_data_ports or scoped_variables")
        else:  # child
            if not to_key in self.states[to_state].input_data_ports:
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
        if self._scoped_variables[scoped_variable_id].name in self.__scoped_variables_names:
            self.__scoped_variables_names.remove(self._scoped_variables[scoped_variable_id].name)
        # delete scoped variable
        del self._scoped_variables[scoped_variable_id]

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
                    self.scoped_data[str(input_data_port_key)+self.state_id] =\
                        ScopedData(data_port.name, value, type(value).__name__, self.state_id, DataPortType.INPUT)
                    # forward the data to scoped variables
                    for data_flow_key, data_flow in self.data_flows.iteritems():
                        if data_flow.from_key == input_data_port_key and data_flow.from_state == self.state_id:
                            if data_flow.to_state == self.state_id:
                                current_scoped_variable = self.scoped_variables[data_flow.to_key]
                                self.scoped_data[str(data_flow.to_key)+self.state_id] = \
                                    ScopedData(current_scoped_variable.name, value, type(value).__name__, self.state_id,
                                               DataPortType.SCOPED)

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
                                ScopedData(current_scoped_variable.name, value, type(value).__name__, state.state_id,
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
                        scoped_data_key = str(data_flow.from_key)+data_flow.from_state
                        if scoped_data_key in self.scoped_data.iterkeys():
                            self.output_data[output_name] = \
                                copy.deepcopy(self.scoped_data[scoped_data_key].value)
                        else:
                            logger.error("Output data with name %s was not found in the scoped data. "
                                         "This normally means a statemachine design error", output_name)

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
            raise AttributeError("start_state_id does not exist")

        if start_state_id is None and to_outcome is not None:
            if to_outcome not in self.outcomes:
                raise AttributeError("to_outcome does not exist")

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
