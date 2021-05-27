# Copyright (C) 2014-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Michael Vilzmann <michael.vilzmann@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: container_state
   :synopsis: A module to represent a generic container state in the state machine

"""
from weakref import ref
from builtins import str
from copy import copy, deepcopy
from threading import Condition
from collections import OrderedDict

from gtkmvc3.observable import Observable

from rafcon.core.custom_exceptions import RecoveryModeException
from rafcon.core.decorators import lock_state_machine
from rafcon.core.execution.execution_status import StateMachineExecutionStatus
from rafcon.core.id_generator import *
from rafcon.core.singleton import state_machine_execution_engine
from rafcon.core.state_elements.data_flow import DataFlow
from rafcon.core.state_elements.logical_port import Outcome
from rafcon.core.state_elements.scope import ScopedData, ScopedVariable
from rafcon.core.state_elements.data_port import InputDataPort, OutputDataPort
from rafcon.core.state_elements.state_element import StateElement
from rafcon.core.state_elements.transition import Transition
from rafcon.core.states.library_state import LibraryState
from rafcon.core.states.state import State
from rafcon.core.states.state import StateExecutionStatus
from rafcon.core.config import global_config
from rafcon.utils.type_helpers import type_inherits_of_type
from rafcon.utils import log

logger = log.get_logger(__name__)


class ContainerState(State):
    """A class for representing a state in the state machine

    Only the variables are listed that are not already contained in the state base class

    :ivar dict ContainerState.states: the child states of the container state of the state
    :ivar dict ContainerState.transitions: transitions between all child states
    :ivar dict ContainerState.data_flows: data flows between all child states
    :ivar str ContainerState.start_state_id: the state to start with when the hierarchy state is executed
    :ivar dict ContainerState.scoped_variables: the scoped variables of the container

    """

    _state_element_attrs = State.state_element_attrs + ['scoped_variables', 'states', 'transitions', 'data_flows']

    def __init__(self, name=None, state_id=None, input_data_ports=None, output_data_ports=None,
                 income=None, outcomes=None,
                 states=None, transitions=None, data_flows=None, start_state_id=None,
                 scoped_variables=None, safe_init=True):

        self._states = OrderedDict()
        self._transitions = {}
        self._data_flows = {}
        self._scoped_variables = {}
        self._scoped_data = {}
        self._current_state = None
        # condition variable to wait for not connected states
        self._transitions_cv = Condition()
        self._child_execution = False
        self._start_state_modified = False

        State.__init__(self, name, state_id, input_data_ports, output_data_ports, income, outcomes, safe_init=safe_init)

        if start_state_id is not None:
            self.start_state_id = start_state_id

        if safe_init:
            ContainerState._safe_init(self, scoped_variables=scoped_variables, states=states, transitions=transitions,
                                      data_flows=data_flows)
        else:
            ContainerState._unsafe_init(self, scoped_variables=scoped_variables, states=states, transitions=transitions,
                                        data_flows=data_flows)

    def _safe_init(self, scoped_variables, states, transitions, data_flows):
        self.scoped_variables = scoped_variables if scoped_variables is not None else {}
        self.states = states if states is not None else {}
        self.transitions = transitions if transitions is not None else {}
        self.data_flows = data_flows if data_flows is not None else {}

    def _unsafe_init(self, scoped_variables, states, transitions, data_flows):
        self._scoped_variables = scoped_variables if scoped_variables is not None else {}
        # set parents manually
        for _, scoped_variable in self._scoped_variables.items():
            scoped_variable._parent = ref(self)
        self._states = states if states is not None else {}
        for _, state in self._states.items():
            state._parent = ref(self)
        self._transitions = transitions if transitions is not None else {}
        for _, transition in self._transitions.items():
            transition._parent = ref(self)
        self._data_flows = data_flows if data_flows is not None else {}
        for _, data_flow in self._data_flows.items():
            data_flow._parent = ref(self)

    # ---------------------------------------------------------------------------------------------
    # ----------------------------------- generic methods -----------------------------------------
    # ---------------------------------------------------------------------------------------------

    @lock_state_machine
    def update_hash(self, obj_hash):
        super(ContainerState, self).update_hash(obj_hash)
        for state_element in sorted(self.states.values()) + sorted(list(self.transitions.values()) +
                                                                   list(self.data_flows.values()) +
                                                                   list(self.scoped_variables.values())):
            self.update_hash_from_dict(obj_hash, state_element)

    @staticmethod
    def state_to_dict(state):
        dict_representation = {
            'name': state.name,
            'state_id': state.state_id,
            'description': state.description,
            'input_data_ports': state.input_data_ports,
            'output_data_ports': state.output_data_ports,
            'income': state.income,
            'outcomes': state.outcomes,
            'transitions': state.transitions,
            'data_flows': state.data_flows,
            'scoped_variables': state.scoped_variables
        }
        return dict_representation

    @classmethod
    def from_dict(cls, dictionary):
        states = None if 'states' not in dictionary else dictionary['states']
        transitions = dictionary['transitions']
        data_flows = dictionary['data_flows']
        safe_init = global_config.get_config_value("LOAD_SM_WITH_CHECKS", True)
        state = cls(name=dictionary['name'],
                    state_id=dictionary['state_id'],
                    input_data_ports=dictionary['input_data_ports'],
                    output_data_ports=dictionary['output_data_ports'],
                    income=dictionary.get('income', None),  # older state machine versions don't have this set
                    outcomes=dictionary['outcomes'],
                    states=None,
                    transitions=transitions if states else None,
                    data_flows=data_flows if states else None,
                    scoped_variables=dictionary['scoped_variables'],
                    safe_init=safe_init)
        try:
            state.description = dictionary['description']
        except (TypeError, KeyError):  # (Very) old state machines do not have a description field
            import traceback
            formatted_lines = traceback.format_exc().splitlines()
            logger.warning("Erroneous description for state '{1}': {0}".format(formatted_lines[-1], dictionary['name']))

        if states:
            return state
        else:
            return state, dictionary['transitions'], dictionary['data_flows']

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node, deep=True)
        state, transitions, data_flows = cls.from_dict(dict_representation)
        return state, transitions, data_flows

    def __str__(self):
        return "{0} [{1} child states]".format(super(ContainerState, self).__str__(), len(self.states))

    def __hash__(self):
        return id(self)

    def __eq__(self, other):
        # logger.info("compare method \n\t\t\t{0} \n\t\t\t{1}".format(self, other))
        if not isinstance(other, self.__class__):
            return False
        try:
            diff_states = [self.states[state_id] == state for state_id, state in list(other.states.items())]
            diff_states.append(len(self.states) == len(other.states))
        except KeyError:
            return False
        return all(diff_states) and str(self) == str(other)

    def __copy__(self):
        input_data_ports = {key: copy(self._input_data_ports[key]) for key in self._input_data_ports.keys()}
        output_data_ports = {key: copy(self._output_data_ports[key]) for key in self._output_data_ports.keys()}
        income = copy(self._income)
        outcomes = {key: copy(self._outcomes[key]) for key in self._outcomes.keys()}
        states = {key: copy(self._states[key]) for key in self._states.keys()}
        scoped_variables = {key: copy(self._scoped_variables[key]) for key in self._scoped_variables.keys()}
        data_flows = {key: copy(self._data_flows[key]) for key in self._data_flows.keys()}
        transitions = {key: copy(self._transitions[key]) for key in self._transitions.keys()}

        state = self.__class__(self.name, self.state_id, input_data_ports, output_data_ports, income, outcomes, states,
                               transitions, data_flows, None, scoped_variables, safe_init=False)
        state._description = deepcopy(self.description)
        state._semantic_data = deepcopy(self.semantic_data)
        state._file_system_path = self.file_system_path
        return state

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    def __contains__(self, item):
        """Checks whether `item` is an element of the container state

        Following child items are checked: outcomes, input data ports, output data ports, scoped variables, states,
        transitions, data flows

        :param item: State or state element
        :return: Whether item is a direct child of this state
        :rtype: bool
        """
        if not isinstance(item, (State, StateElement)):
            return False
        return super(ContainerState, self).__contains__(item) or item in self.states.values() \
               or item in self.transitions.values() or item in self.data_flows.values() \
               or item in self.scoped_variables.values()

    # ---------------------------------------------------------------------------------------------
    # ----------------------------------- execution functions -------------------------------------
    # ---------------------------------------------------------------------------------------------

    def run(self, *args, **kwargs):
        """Implementation of the abstract run() method of the :class:`threading.Thread`

        Should be filled with code, that should be executed for each container_state derivative.
        :raises exceptions.NotImplementedError: in every case
        """
        raise NotImplementedError("The ContainerState.run() function has to be implemented!")

    def recursively_preempt_states(self):
        """ Preempt the state and all of it child states.
        """
        super(ContainerState, self).recursively_preempt_states()
        # notify the transition condition variable to let the state instantaneously stop
        with self._transitions_cv:
            self._transitions_cv.notify_all()
        for state in self.states.values():
            state.recursively_preempt_states()

    def recursively_pause_states(self):
        """ Pause the state and all of it child states.
        """
        super(ContainerState, self).recursively_pause_states()
        for state in self.states.values():
            state.recursively_pause_states()

    def recursively_resume_states(self):
        """ Resume the state and all of it child states.
        """
        super(ContainerState, self).recursively_resume_states()
        for state in self.states.values():
            state.recursively_resume_states()

    def setup_run(self):
        """ Executes a generic set of actions that has to be called in the run methods of each derived state class.

        :return:
        """
        super(ContainerState, self).setup_run()
        # reset the scoped data
        self._scoped_data = {}
        self._start_state_modified = False
        self.add_default_values_of_scoped_variables_to_scoped_data()
        self.add_input_data_to_scoped_data(self.input_data)

    def handle_no_transition(self, state):
        """ This function handles the case that there is no transition for a specific outcome of a sub-state.

        The method waits on a condition variable to a new transition that will be connected by the programmer or
        GUI-user.

        :param state: The sub-state to find a transition for
        :return: The transition for the target state.
        :raises exceptions.RuntimeError: if the execution engine is stopped
                                        (this will be caught at the end of the run method)
        """
        transition = None
        while not transition:

            # (child) state preempted or aborted
            if self.preempted or state.final_outcome.outcome_id in [-2, -1]:
                if self.concurrency_queue:
                    self.concurrency_queue.put(self.state_id)
                self.state_execution_status = StateExecutionStatus.WAIT_FOR_NEXT_STATE

                if state.final_outcome.outcome_id == -1:
                    self.final_outcome = Outcome(-1, "aborted")
                else:
                    self.final_outcome = Outcome(-2, "preempted")

                logger.debug("{0} of {1} not connected, using default transition to parental {2}".format(
                    state.final_outcome, state, self.final_outcome))
                return None

            # depending on the execution mode pause execution
            execution_signal = state_machine_execution_engine.handle_execution_mode(self)
            if execution_signal is StateMachineExecutionStatus.STOPPED:
                # this will be caught at the end of the run method
                self.last_child.state_execution_status = StateExecutionStatus.INACTIVE
                logger.warning("State machine was stopped, while state {} waited for the next transition.".format(
                    state.name
                ))
                return None

            # wait until the user connects the outcome of the state with a transition
            logger.warning("Waiting for new transition at {1} of {0} ".format(state, state.final_outcome))
            with self._transitions_cv:
                self._transitions_cv.wait(3.0)

            transition = self.get_transition_for_outcome(state, state.final_outcome)

        return transition

    def handle_no_start_state(self):
        """Handles the situation, when no start state exists during execution

        The method waits, until a transition is created. It then checks again for an existing start state and waits
        again, if this is not the case. It returns the None state if the the state machine was stopped.
        """
        start_state = self.get_start_state(set_final_outcome=True)
        while not start_state:
            # depending on the execution mode pause execution
            execution_signal = state_machine_execution_engine.handle_execution_mode(self)
            if execution_signal is StateMachineExecutionStatus.STOPPED:
                # this will be caught at the end of the run method
                return None

            with self._transitions_cv:
                self._transitions_cv.wait(3.0)
            start_state = self.get_start_state(set_final_outcome=True)
        return start_state

    # ---------------------------------------------------------------------------------------------
    # -------------------------------------- state functions --------------------------------------
    # ---------------------------------------------------------------------------------------------

    @lock_state_machine
    @Observable.observed
    def group_states(self, state_ids, scoped_variable_ids=None):
        """ Group states and scoped variables into a new hierarchy state and remain internal connections.
            Interconnecting transitions and data flows to parent and other child states are removed, at the moment.

        :param state_ids: state_id's of all states that are to be grouped.
        :param scoped_variable_ids: data_port_id's of all scoped variables that are to be grouped, too.
        :return:
        """
        # TODO remember changed state or state element ids and provide them for the model functionalities
        assert all([state_id in self.states.keys() for state_id in state_ids])
        if scoped_variable_ids is None:
            scoped_variable_ids = []
        assert all([p_id in self.scoped_variables.keys() for p_id in scoped_variable_ids])
        from rafcon.core.states.barrier_concurrency_state import DeciderState
        if any(isinstance(self.states[child_state_id], DeciderState) for child_state_id in state_ids):
            raise ValueError("State of type DeciderState can not be grouped.")

        def create_name(name_str, used_names):
            number_str = ""
            number_of_str = 0
            while name_str + number_str in used_names:
                number_str = "_{}".format(number_of_str)
                number_of_str += 1
            return name_str + number_str

        [related_transitions, related_data_flows] = self.get_connections_for_state_and_scoped_variables(state_ids,
                                                                                                        scoped_variable_ids)

        def assign_ingoing_outgoing(df, going_data_linkage_for_port, ingoing=True):
            internal = 'internal' if ingoing else 'external'
            external = 'external' if ingoing else 'internal'
            if (df.to_state, df.to_key) in going_data_linkage_for_port['to']:
                going_data_linkage_for_port['to'][(df.to_state, df.to_key)][external].append(df)
            else:
                going_data_linkage_for_port['to'][(df.to_state, df.to_key)] = {external: [df], internal: [df]}
            if (df.from_state, df.from_key) in going_data_linkage_for_port['from']:
                going_data_linkage_for_port['from'][(df.from_state, df.from_key)][internal].append(df)
            else:
                going_data_linkage_for_port['from'][(df.from_state, df.from_key)] = {external: [df], internal: [df]}

        def print_df_from_and_to(going_data_linkage_for_port):
            logger.verbose('data linkage FROM: ')
            for port, port_dfs in going_data_linkage_for_port['from'].items():
                logger.verbose("\tport: {0} {1}".format(port, '' if 'args' not in port_dfs else port_dfs['args']))
                logger.verbose("\t\texternal: \n\t\t\t" + "\n\t\t\t".join([str(df) for df in port_dfs['external']]))
                logger.verbose("\t\tinternal: \n\t\t\t" + "\n\t\t\t".join([str(df) for df in port_dfs['internal']]))
            logger.verbose('data linkage TO: ')
            for port, port_dfs in going_data_linkage_for_port['to'].items():
                logger.verbose("\tport: {0} {1}".format(port, '' if 'args' not in port_dfs else port_dfs['args']))
                logger.verbose("\t\texternal: \n\t\t\t" + "\n\t\t\t".join([str(df) for df in port_dfs['external']]))
                logger.verbose("\t\tinternal: \n\t\t\t" + "\n\t\t\t".join([str(df) for df in port_dfs['internal']]))

        def reduce_dfs(port_data_linkages, df_id):
            for port_key in list(port_data_linkages.keys()):
                port_df = port_data_linkages[port_key]
                port_df['internal'] = [df for df in port_df['internal'] if df.data_flow_id != df_id]
                port_df['external'] = [df for df in port_df['external'] if df.data_flow_id != df_id]
                if not port_df['internal'] and not port_df['external']:
                    del port_data_linkages[port_key]

        def do_prior_in_out_going(going_data_linkage_for_port, prior_port_key, prior_locate_key):
            # logger.info("PRIOR IN OUT {0}, {1}".format(prior_port_key, prior_locate_key))
            minor_port_key = 'from' if prior_port_key == 'to' else 'to'
            minor_locate_key = 'external' if prior_locate_key == 'internal' else 'internal'
            for port_key in list(going_data_linkage_for_port[prior_port_key].keys()):
                port_dfs = going_data_linkage_for_port[prior_port_key][port_key]
                # print(prior_port_key, ": check: ", port_key, " length: ", len(port_dfs[prior_locate_key]))
                if len(port_dfs[prior_locate_key]) > 1:
                    for df in port_dfs[prior_locate_key]:
                        # print("remove: ", df.data_flow_id, prior_locate_key, minor_port_key)
                        reduce_dfs(going_data_linkage_for_port[minor_port_key], df.data_flow_id)
            for port_key in list(going_data_linkage_for_port[minor_port_key].keys()):
                port_dfs = going_data_linkage_for_port[minor_port_key][port_key]
                # print(minor_port_key, ": check: ", port_key, " length: ", len(port_dfs[minor_locate_key]))
                if len(port_dfs[minor_locate_key]) > 1:
                    for df in port_dfs[minor_locate_key]:
                        # print("remove: ", df.data_flow_id, minor_locate_key, prior_port_key)
                        reduce_dfs(going_data_linkage_for_port[prior_port_key], df.data_flow_id)
            for port_key in list(going_data_linkage_for_port[prior_port_key].keys()):
                port_dfs = going_data_linkage_for_port[prior_port_key][port_key]
                # print(prior_port_key, ": check: ", port_key, " length: ", len(port_dfs[prior_locate_key]))
                if len(port_dfs[prior_locate_key]) == 1:
                    for df in port_dfs[prior_locate_key]:
                        # print("remove: ", df.data_flow_id, prior_locate_key, minor_port_key)
                        reduce_dfs(going_data_linkage_for_port[minor_port_key], df.data_flow_id)

        def create_data_port_args(going_data_linkage_for_port):
            names = []
            for goal, data_port_linkage in going_data_linkage_for_port['to'].items():
                state = self.states[goal[0]] if goal[0] != self.state_id else self
                port = state.get_data_port_by_id(goal[1])
                data_port_linkage['args'] = port.state_element_to_dict(port)
                data_port_linkage['args']['name'] = create_name(data_port_linkage['args']['name'], names)
                names.append(data_port_linkage['args']['name'])
            for goal, data_port_linkage in going_data_linkage_for_port['from'].items():
                state = self.states[goal[0]] if goal[0] != self.state_id else self
                port = state.get_data_port_by_id(goal[1])
                data_port_linkage['args'] = port.state_element_to_dict(port)
                data_port_linkage['args']['name'] = create_name(data_port_linkage['args']['name'], names)
                names.append(data_port_linkage['args']['name'])

        ################## IDENTIFY/PRE-PROCESS INGOING DATA FLOWS ###################
        # ingoing data linkage to rebuild -> overview of all with duplicates
        ingoing_data_linkage_for_port = {'from': {}, 'to': {}}
        for df in related_data_flows['ingoing']:
            assign_ingoing_outgoing(df, ingoing_data_linkage_for_port)

        # logger.info("GROUP DATA INGOING BEFORE")
        # print_df_from_and_to(ingoing_data_linkage_for_port)

        # prior to-linkage-merge for ingoing over from-linkage-merge -> less internal ingoing data flows
        do_prior_in_out_going(ingoing_data_linkage_for_port, prior_port_key='to', prior_locate_key='external')

        # logger.info("GROUPED INGOING DATA AFTER")
        # print_df_from_and_to(ingoing_data_linkage_for_port)

        # get name and args for ports
        create_data_port_args(ingoing_data_linkage_for_port)
        # logger.info("GROUPED INGOING DATA PORTS")
        # print_df_from_and_to(ingoing_data_linkage_for_port)

        ################## IDENTIFY/PRE-PROCESS OUTGOING DATA FLOWS ###################
        # outgoing data linkage to rebuild -> overview of all with duplicates
        outgoing_data_linkage_for_port = {'from': {}, 'to': {}}
        for df in related_data_flows['outgoing']:
            assign_ingoing_outgoing(df, outgoing_data_linkage_for_port, ingoing=False)

        # logger.info("GROUP DATA OUTGOING BEFORE")
        # print_df_from_and_to(outgoing_data_linkage_for_port)

        # prior from-linkage-merge for outgoing over to-linkage-merge -> less outgoing data flows
        do_prior_in_out_going(outgoing_data_linkage_for_port, prior_port_key='from', prior_locate_key='external')

        # logger.info("GROUPED OUTGOING DATA AFTER")
        # print_df_from_and_to(outgoing_data_linkage_for_port)

        # get name and args for ports
        create_data_port_args(outgoing_data_linkage_for_port)
        # logger.info("GROUPED OUTGOING DATA PORTS")
        # print_df_from_and_to(outgoing_data_linkage_for_port)

        ############################# CREATE NEW STATE #############################
        # all internal transitions
        transitions_internal = {t.transition_id: self.remove_transition(t.transition_id, destroy=False)
                                for t in related_transitions['enclosed']}
        # all internal data flows
        data_flows_internal = {df.data_flow_id: self.remove_data_flow(df.data_flow_id, destroy=False)
                               for df in related_data_flows['enclosed']}
        # TODO add warning if a data-flow is connected to scoped variables which has ingoing and outgoing data flows
        # TODO linked with selected states -> group would change behavior and scoped would need to be selected or
        # TODO                                local scoped variable need to be introduced in new hierarchy state
        # all internal scoped variables
        scoped_variables_to_group = {dp_id: self.remove_scoped_variable(dp_id, destroy=False)
                                     for dp_id in scoped_variable_ids}
        # all states
        states_to_group = {state_id: self.remove_state(state_id, recursive=False, destroy=False)
                           for state_id in state_ids}
        from rafcon.core.states.hierarchy_state import HierarchyState
        # secure state id conflicts for the taken transitions
        from rafcon.core.id_generator import state_id_generator
        state_id = state_id_generator(used_state_ids=state_ids + [self.state_id])
        # if scoped variables are used all data flows have to be checked if those link to those and correct the state_id
        if scoped_variable_ids:
            for data_flow in data_flows_internal.values():
                if data_flow.from_state == self.state_id:
                    data_flow.from_state = state_id
                if data_flow.to_state == self.state_id:
                    data_flow.to_state = state_id
        s = HierarchyState(states=states_to_group, transitions=transitions_internal, data_flows=data_flows_internal,
                           scoped_variables=scoped_variables_to_group, state_id=state_id)
        state_id = self.add_state(s)

        def find_logical_destinations_of_transitions(transitions):
            destinations = {}
            for t in transitions:
                if (t.to_state, t.to_outcome) in destinations:
                    destinations[(t.to_state, t.to_outcome)].append(t)
                else:
                    destinations[(t.to_state, t.to_outcome)] = [t]
            return destinations

        ################## IDENTIFY TRANSITIONS #####################
        # transition from ingoing transition
        ingoing_logical_destinations = find_logical_destinations_of_transitions(related_transitions['ingoing'])
        if len(ingoing_logical_destinations) > 1:
            logger.warning("There is only one ingoing transition on a state possible. \n"
                           "The following transitions are removed by 'group_states': \n{}"
                           "".format('\n'.join([str(destination) for destination in ingoing_logical_destinations.items()[1:]])))
        ingoing_transitions = None
        if len(ingoing_logical_destinations) > 0:
            ingoing_transitions = list(ingoing_logical_destinations.items())[0][1]
        # transitions from outgoing transitions
        transitions_outgoing = {t.transition_id: t for t in related_transitions['outgoing']}
        outgoing_logical_destinations = find_logical_destinations_of_transitions(related_transitions['outgoing'])

        ################## DO INGOING TRANSITIONS ###################
        if ingoing_transitions:
            t = ingoing_transitions[0]
            s.add_transition(None, None, t.to_state, t.to_outcome)
            for t in ingoing_transitions:
                self.add_transition(t.from_state, t.from_outcome, s.state_id, None)

        ################## DO OUTGOING OUTCOMES ###################
        # outcomes from outgoing transitions
        outcomes_outgoing_transitions = {}
        new_outcome_ids = {}
        state_outcomes_by_name = {oc.name: oc_id for oc_id, oc in s.outcomes.items()}
        for goal, transitions in list(outgoing_logical_destinations.items()):
            t = transitions[0]
            name = s.states[t.from_state].outcomes[t.from_outcome].name
            # print((t.to_state, t.to_outcome))
            # print(outcomes_outgoing_transitions)
            if goal in outcomes_outgoing_transitions:
                # logger.info("old outcome {}".format((t.to_state, t.to_outcome)))
                name = outcomes_outgoing_transitions[goal]
            else:
                name = create_name(name, list(new_outcome_ids.keys()))
                outcomes_outgoing_transitions[goal] = name
            # print(outcomes_outgoing_transitions, "\n", new_outcome_ids)
            if name not in new_outcome_ids:
                if name in state_outcomes_by_name:
                    new_outcome_ids[name] = state_outcomes_by_name[name]
                    # logger.info("old outcome_id {0}\n{1}".format(state_outcomes_by_name[name], new_outcome_ids))
                else:
                    new_outcome_ids[name] = s.add_outcome(name=name)
                    # logger.info("new outcome_id {0}\n{1}".format(new_outcome_ids[name], new_outcome_ids))
            # else:
            #     logger.info("name {0} in {1} -> {2}".format(name, new_outcome_ids, outcomes_outgoing_transitions))

        ################## DO OUTGOING TRANSITIONS ###################
        # external outgoing transitions
        # print("external transitions to create", outcomes_outgoing_transitions)
        for goal, name in outcomes_outgoing_transitions.items():
            try:
                # avoid to use a outcome twice
                if any([t for t in s.parent.transitions.values()
                        if t.from_state == s.state_id and t.from_outcome == new_outcome_ids[name]]):
                    continue
                # add the transition for the outcome
                self.add_transition(s.state_id, new_outcome_ids[name], goal[0], goal[1])
            except ValueError:
                logger.exception("Error while recreation of logical linkage.")
        # internal outgoing transitions
        # print("internal transitions to create", transitions_outgoing)
        for t_id, t in transitions_outgoing.items():
            name = outcomes_outgoing_transitions[(t.to_state, t.to_outcome)]
            s.add_transition(t.from_state, t.from_outcome, s.state_id, new_outcome_ids[name], t_id)

        new_state_ids = {self.state_id: s.state_id}
        ############## REBUILD INGOING DATA LINKAGE ################
        full_linkage = copy(ingoing_data_linkage_for_port['from'])
        full_linkage.update(ingoing_data_linkage_for_port['to'])
        for port, data_port_linkage in full_linkage.items():
            # input data ports from ingoing data flows
            args = data_port_linkage['args']
            args['data_port_id'] = None
            args['data_port_id'] = s.add_input_data_port(**args)
            # internal data flows from ingoing data flows
            # print("ingoing internal data flows")
            for df in data_port_linkage['internal']:
                # print(df)
                s.add_data_flow(from_state_id=s.state_id, from_data_port_id=args['data_port_id'],
                                to_state_id=new_state_ids.get(df.to_state, df.to_state),
                                to_data_port_id=df.to_key, data_flow_id=df.data_flow_id)
            # external data flows from ingoing data flows
            # print("ingoing external data flows")
            for df in data_port_linkage['external']:
                # print(df)
                self.add_data_flow(from_state_id=df.from_state,
                                   from_data_port_id=df.from_key, to_state_id=s.state_id,
                                   to_data_port_id=args['data_port_id'], data_flow_id=df.data_flow_id)

        ############## REBUILD OUTGOING DATA LINKAGE ################
        full_linkage = copy(outgoing_data_linkage_for_port['from'])
        full_linkage.update(outgoing_data_linkage_for_port['to'])
        for port, data_port_linkage in full_linkage.items():
            # output data ports from outgoing data flows
            args = data_port_linkage['args']
            args['data_port_id'] = None
            args['data_port_id'] = s.add_output_data_port(**args)
            # internal data flows from outgoing data flows
            # print("outgoing internal data flows")
            for df in data_port_linkage['internal']:
                # print(df)
                s.add_data_flow(from_state_id=new_state_ids.get(df.from_state, df.from_state),
                                from_data_port_id=df.from_key, to_state_id=s.state_id,
                                to_data_port_id=args['data_port_id'], data_flow_id=df.data_flow_id)
            # external data flows from outgoing data flows
            # print("outgoing external data flows")
            for df in data_port_linkage['external']:
                # print(df)
                self.add_data_flow(from_state_id=s.state_id, from_data_port_id=args['data_port_id'],
                                   to_state_id=df.to_state, to_data_port_id=df.to_key, data_flow_id=df.data_flow_id)

        return self.states[state_id]

    @lock_state_machine
    @Observable.observed
    def ungroup_state(self, state_id):
        """ Ungroup state with state id state_id into its parent and remain internal linkage in parent.
            Interconnecting transitions and data flows to parent and other child states are preserved except:
            - a transition that is going from income to outcome directly and
            - a data-flow that is linking input and output directly.

        :param state_id: State that is to be ungrouped.
        :return:
        """
        state = self.states[state_id]
        assert isinstance(state, ContainerState)
        from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState, UNIQUE_DECIDER_STATE_ID
        if isinstance(state, BarrierConcurrencyState):
            state.remove_state(state_id=UNIQUE_DECIDER_STATE_ID, force=True)
        [related_transitions, related_data_flows] = self.get_connections_for_state(state_id)

        # ingoing logical linkage to rebuild -> related_transitions['external']['ingoing']
        # outgoing logical linkage to rebuild -> related_transitions['external']['outgoing']
        # ingoing data linkage to rebuild
        ingoing_data_linkage_for_port = {}
        for df in related_data_flows['internal']['ingoing']:
            if (df.from_state, df.from_key) in ingoing_data_linkage_for_port:
                ingoing_data_linkage_for_port[(df.from_state, df.from_key)]['internal'].append(df)
            else:
                ingoing_data_linkage_for_port[(df.from_state, df.from_key)] = {'external': [], 'internal': [df]}
            if not ingoing_data_linkage_for_port[(df.from_state, df.from_key)]['external']:
                for ext_df in self.data_flows.values():
                    if (ext_df.to_state, ext_df.to_key) == (df.from_state, df.from_key):
                        ingoing_data_linkage_for_port[(df.from_state, df.from_key)]['external'].append(ext_df)
        # outgoing data linkage to rebuild
        outgoing_data_linkage_for_port = {}
        for df in related_data_flows['internal']['outgoing']:
            if (df.to_state, df.to_key) in outgoing_data_linkage_for_port:
                outgoing_data_linkage_for_port[(df.to_state, df.to_key)]['internal'].append(df)
            else:
                outgoing_data_linkage_for_port[(df.to_state, df.to_key)] = {'external': [], 'internal': [df]}
            if not outgoing_data_linkage_for_port[(df.to_state, df.to_key)]['external']:
                for ext_df in self.data_flows.values():
                    if (ext_df.from_state, ext_df.from_key) == (df.to_state, df.to_key):
                        outgoing_data_linkage_for_port[(df.to_state, df.to_key)]['external'].append(ext_df)
        # hold states and scoped variables to rebuild
        child_states = [state.remove_state(s_id, recursive=False, destroy=False) for s_id in list(state.states.keys())]
        child_scoped_variables = [sv for sv_id, sv in list(state.scoped_variables.items())]

        # remove state that should be ungrouped
        old_state = self.remove_state(state_id, recursive=False, destroy=False)

        # fill elements into parent state and remember id mapping from child to parent state to map other properties
        state_id_dict = {}
        sv_id_dict = {}
        enclosed_df_id_dict = {}
        enclosed_t_id_dict = {}

        # re-create states
        old_state_ids = [state.state_id for state in child_states]
        for child_state in child_states:
            old_state_id = child_state.state_id
            # needed to change state id here because not handled in add state and to avoid old state ids
            new_id = None
            if child_state.state_id in list(self.states.keys()):
                new_id = state_id_generator(used_state_ids=list(self.states.keys()) + old_state_ids + [self.state_id])
                child_state.change_state_id(new_id)
            new_state_id = self.add_state(child_state)
            if new_id is not None and not new_id == new_state_id:
                logger.error("In ungroup state the changed state id should not be changed again by add_state because it"
                             " could become a old_state_id again and screw data flows and transitions.")
            # remember new and old state id relations
            state_id_dict[old_state_id] = new_state_id
        # re-create scoped variables
        for sv in child_scoped_variables:
            name = sv.name
            if name in [parent_sv.name for parent_sv in self.scoped_variables.values()]:
                name = state_id + name
            new_sv_id = self.add_scoped_variable(name, sv.data_type, sv.default_value)
            sv_id_dict[sv.data_port_id] = new_sv_id

        # re-create transitions
        for t in related_transitions['internal']['enclosed']:
            new_t_id = self.add_transition(state_id_dict[t.from_state], t.from_outcome,
                                           state_id_dict[t.to_state], t.to_outcome)
            enclosed_t_id_dict[t.transition_id] = new_t_id
        assert len(related_transitions['internal']['ingoing']) <= 1
        if related_transitions['internal']['ingoing']:
            ingoing_t = related_transitions['internal']['ingoing'][0]
            for t in related_transitions['external']['ingoing']:
                self.add_transition(t.from_state, t.from_outcome, state_id_dict[ingoing_t.to_state],
                                    ingoing_t.to_outcome)
        for ext_t in related_transitions['external']['outgoing']:
            for t in related_transitions['internal']['outgoing']:
                if (t.to_state, t.to_outcome) == (ext_t.from_state, ext_t.from_outcome):
                    try:
                        self.add_transition(state_id_dict[t.from_state], t.from_outcome,
                                            ext_t.to_state, ext_t.to_outcome)
                    except ValueError:
                        from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState
                        if not isinstance(self, BarrierConcurrencyState):
                            logger.exception("Error while recreation of logical linkage.")

        # re-create data flow linkage
        for df in related_data_flows['internal']['enclosed']:
            # print("enclosed: ", df)
            new_df_id = self.add_data_flow(self.state_id if state_id == df.from_state else state_id_dict[df.from_state],
                                           sv_id_dict[df.from_key] if state_id == df.from_state else df.from_key,
                                           self.state_id if state_id == df.to_state else state_id_dict[df.to_state],
                                           sv_id_dict[df.to_key] if state_id == df.to_state else df.to_key)
            enclosed_df_id_dict[df.data_flow_id] = new_df_id
        for data_port_linkage in ingoing_data_linkage_for_port.values():
            for ext_df in data_port_linkage['external']:
                for df in data_port_linkage['internal']:
                    # print("ingoing: ", ext_df, df)
                    if df.to_state not in state_id_dict and df.to_state == state_id:
                        self.add_data_flow(ext_df.from_state, ext_df.from_key, self.state_id, sv_id_dict[df.to_key])
                    else:
                        self.add_data_flow(ext_df.from_state, ext_df.from_key, state_id_dict[df.to_state], df.to_key)
        for data_port_linkage in outgoing_data_linkage_for_port.values():
            for ext_df in data_port_linkage['external']:
                for df in data_port_linkage['internal']:
                    # print("outgoing: ", ext_df, df)
                    if df.from_state not in state_id_dict and df.from_state == state_id:
                        self.add_data_flow(self.state_id, sv_id_dict[df.from_key], ext_df.to_state, ext_df.to_key)
                    else:
                        self.add_data_flow(state_id_dict[df.from_state], df.from_key, ext_df.to_state, ext_df.to_key)

        self.ungroup_state.__func__.state_id_dict = state_id_dict
        self.ungroup_state.__func__.sv_id_dict = sv_id_dict
        self.ungroup_state.__func__.enclosed_df_id_dict = enclosed_df_id_dict
        self.ungroup_state.__func__.enclosed_t_id_dict = enclosed_t_id_dict

        old_state.destroy(recursive=True)
        return old_state

    @lock_state_machine
    @Observable.observed
    def add_state(self, state, storage_load=False):
        """Adds a state to the container state.

        :param state: the state that is going to be added
        :param storage_load: True if the state was directly loaded from filesystem
        :return: the state_id of the new state
        :raises exceptions.AttributeError: if state.state_id already exist
        """
        assert isinstance(state, State)
        # logger.info("add state {}".format(state))

        # handle the case that the child state id is the same as the container state id or future sibling state id
        while state.state_id == self.state_id or state.state_id in self.states:
            state.change_state_id()

        # TODO: add validity checks for states and then remove this check => to discuss
        if state.state_id in self._states.keys():
            raise AttributeError("State id %s already exists in the container state", state.state_id)
        else:
            state.parent = self
            self._states[state.state_id] = state

        return state.state_id

    @lock_state_machine
    @Observable.observed
    def remove_state(self, state_id, recursive=True, force=False, destroy=True):
        """Remove a state from the container state.

        :param state_id: the id of the state to remove
        :param recursive: a flag to indicate a recursive disassembling of all substates
        :param force: a flag to indicate forcefully deletion of all states (important for the decider state in the
                barrier concurrency state)
        :param destroy: a flag which indicates if the state should not only be disconnected from the state but also
                destroyed, including all its state elements
        :raises exceptions.AttributeError: if state.state_id does not
        """
        if state_id not in self.states:
            raise AttributeError("State_id %s does not exist" % state_id)

        if state_id == self.start_state_id:
            self.set_start_state(None)

        # first delete all transitions and data_flows, which are connected to the state to be deleted
        keys_to_delete = []
        for key, transition in self.transitions.items():
            if transition.from_state == state_id or transition.to_state == state_id:
                keys_to_delete.append(key)
        for key in keys_to_delete:
            self.remove_transition(key, True)

        keys_to_delete = []
        for key, data_flow in self.data_flows.items():
            if data_flow.from_state == state_id or data_flow.to_state == state_id:
                keys_to_delete.append(key)
        for key in keys_to_delete:
            self.remove_data_flow(key)

        if recursive and not destroy:
            raise AttributeError("The recursive flag requires the destroy flag to be set, too.")

        if destroy:
            # Recursively delete all transitions, data flows and states within the state to be deleted
            self.states[state_id].destroy(recursive)

        # final delete the state it self
        self.states[state_id].parent = None
        return self.states.pop(state_id)

    # do not observe
    def destroy(self, recursive):
        """ Removes all the state elements.

        :param recursive: Flag whether to destroy all state elements which are removed
        """
        for transition_id in list(self.transitions.keys()):
            self.remove_transition(transition_id, destroy=recursive)
        for data_flow_id in list(self.data_flows.keys()):
            self.remove_data_flow(data_flow_id, destroy=recursive)
        for scoped_variable_id in list(self.scoped_variables.keys()):
            self.remove_scoped_variable(scoped_variable_id, destroy=recursive)
        for state_id in list(self.states.keys()):
            if recursive:
                self.remove_state(state_id, recursive, force=True, destroy=recursive)
            else:
                del self.states[state_id]
        super(ContainerState, self).destroy(recursive)

    def get_connections_for_state(self, state_id):
        """The method generates two dictionaries with related transitions and data flows for the given state_id
        
        The method creates dictionaries for all 'internal' and 'external' (first dict-key) connections of the state.
        Both dictionaries contain sub dicts with 3 (external)/4 (internal) fields 'enclosed', 'ingoing', 'outgoing' and
        'self'.
         - 'enclosed' means the handed state.states cover origin and target of those linkage
         - 'ingoing' means the handed state is target of those linkage
         - 'outgoing' means the handed state is origin of those linkage
         - 'self' (corner case) single state that has linkage with it self and is thereby also origin and target at 
           the same time
        
        :param state_id: State taken into account.
        :rtype tuple
        :return: related_transitions, related_data_flows
        """
        related_transitions = {'external': {'ingoing': [], 'outgoing': [], 'self': []},
                               'internal': {'enclosed': [], 'ingoing': [], 'outgoing': [], 'self': []}}
        related_data_flows = {'external': {'ingoing': [], 'outgoing': [], 'self': []},
                              'internal': {'enclosed': [], 'ingoing': [], 'outgoing': [], 'self': []}}

        # self logical linkage
        related_transitions['external']['self'] = [t for t in self.transitions.values()
                                                   if t.from_state == state_id and t.to_state == state_id]
        # ingoing logical linkage
        related_transitions['external']['ingoing'] = [t for t in self.transitions.values()
                                                      if t.from_state != state_id and t.to_state == state_id]
        # outgoing logical linkage
        related_transitions['external']['outgoing'] = [t for t in self.transitions.values()
                                                       if t.from_state == state_id and t.to_state != state_id]

        # self data linkage
        related_data_flows['external']['self'] = [df for df in self.data_flows.values()
                                                  if df.from_state == state_id and df.to_state == state_id]
        # ingoing data linkage
        related_data_flows['external']['ingoing'] = [df for df in self.data_flows.values()
                                                     if df.from_state != state_id and df.to_state == state_id]
        # outgoing outgoing linkage
        related_data_flows['external']['outgoing'] = [df for df in self.data_flows.values()
                                                      if df.from_state == state_id and df.to_state != state_id]

        state = self.states[state_id]
        if not isinstance(state, ContainerState):
            return related_transitions, related_data_flows

        for t_id, t in state.transitions.items():
            # check if internal of new hierarchy state
            if state_id == t.from_state and state_id == t.to_state:  # most likely never happens but possible
                related_transitions['internal']['self'].append(t)
            elif t.from_state in state.states and t.to_state in state.states:
                related_transitions['internal']['enclosed'].append(t)
            elif t.to_state in state.states:
                related_transitions['internal']['ingoing'].append(t)
            elif t.from_state in state.states:
                related_transitions['internal']['outgoing'].append(t)
            else:
                raise AttributeError("All transition have to be ingoing, outgoing or internal.")

        for df_id, df in state.data_flows.items():
            # check if internal of hierarchy state
            if state_id == df.from_state and state_id == df.to_state:  # most likely never happens but possible
                related_data_flows['internal']['self'].append(df)
            elif df.from_state in state.states and df.to_state in state.states or \
                    df.from_state in state.states and state.state_id == df.to_state and df.to_key in state.scoped_variables or \
                    state.state_id == df.from_state and df.from_key in state.scoped_variables and df.to_state in state.states:
                related_data_flows['internal']['enclosed'].append(df)
            elif df.to_state in state.states or \
                    state.state_id == df.to_state and df.to_key in state.scoped_variables or \
                    df.to_state == df.from_state and df.from_key in state.input_data_ports:
                related_data_flows['internal']['ingoing'].append(df)
            elif df.from_state in state.states or \
                    state.state_id == df.from_state and df.from_key in state.scoped_variables or \
                    df.to_state == df.from_state and df.to_key in state.output_data_ports:
                related_data_flows['internal']['outgoing'].append(df)
            else:
                raise AttributeError("All data flow have to be ingoing, outgoing or internal.")

        return related_transitions, related_data_flows

    def get_connections_for_state_and_scoped_variables(self, state_ids, scoped_variables):
        """The method generates two dictionaries with transitions and data flows for the given state ids and scoped vars
        
        The method creates dictionaries with connections for a set of states and scoped variables.
        Both dictionaries have 3 fields (as first dict-key), 'enclosed', 'ingoing' and 'outgoing'
         - 'enclosed' means the given sets cover origin and target of those linkage
         - 'ingoing' means the given sets is target of those linkage
         - 'ingoing' means the given sets is origin of those linkage
        
        :param state_ids: List of states taken into account. 
        :param scoped_variables: List of scoped variables taken into account
        :rtype tuple
        :return: related_transitions, related_data_flows
        """
        # find all related transitions
        related_transitions = {'enclosed': [], 'ingoing': [], 'outgoing': []}
        for t in self.transitions.values():
            # check if internal of new hierarchy state
            if t.from_state in state_ids and t.to_state in state_ids:
                related_transitions['enclosed'].append(t)
            elif t.to_state in state_ids:
                related_transitions['ingoing'].append(t)
            elif t.from_state in state_ids:
                related_transitions['outgoing'].append(t)

        # find all related data flows
        related_data_flows = {'enclosed': [], 'ingoing': [], 'outgoing': []}
        for df in self.data_flows.values():
            # check if internal of new hierarchy state
            if df.from_state in state_ids and df.to_state in state_ids or \
                    df.from_state in state_ids and self.state_id == df.to_state and df.to_key in scoped_variables or \
                    self.state_id == df.from_state and df.from_key in scoped_variables and df.to_state in state_ids:
                related_data_flows['enclosed'].append(df)
            elif df.to_state in state_ids or \
                    self.state_id == df.to_state and df.to_key in scoped_variables:
                related_data_flows['ingoing'].append(df)
            elif df.from_state in state_ids or \
                    self.state_id == df.from_state and df.from_key in scoped_variables:
                related_data_flows['outgoing'].append(df)

        return related_transitions, related_data_flows

    @lock_state_machine
    @Observable.observed
    def substitute_state(self, state_id, state):

        if state_id not in self.states:
            raise ValueError("The state_id {0} to be substituted has to be in the states list of "
                             "respective parent state {1}.".format(state_id, self.get_path()))
        from rafcon.core.states.barrier_concurrency_state import DeciderState
        if isinstance(self.states[state_id], DeciderState):
            raise ValueError("State of type DeciderState can not be substituted.")

        while state.state_id in self.states:
            logger.info("Rename state_id of state to substitute.")
            state.change_state_id()

        [related_transitions, related_data_flows] = self.get_connections_for_state(state_id)

        readjust_parent_of_ports = True if state.state_id != list(state.outcomes.items())[0][1].parent.state_id else False

        old_outcome_names = {oc_id: oc.name for oc_id, oc in self.states[state_id].outcomes.items()}
        old_input_data_ports = copy(self.states[state_id].input_data_ports)
        old_output_data_ports = copy(self.states[state_id].output_data_ports)
        old_state_was_library = False
        if isinstance(self.states[state_id], LibraryState):
            old_input_data_port_runtime_values = self.states[state_id].input_data_port_runtime_values
            old_output_data_port_runtime_values = self.states[state_id].output_data_port_runtime_values
            old_use_runtime_value_input_data_ports = self.states[state_id].use_runtime_value_input_data_ports
            old_use_runtime_value_output_data_ports = self.states[state_id].use_runtime_value_output_data_ports
            old_state_was_library = True

        self.remove_state(state_id)
        old_state_id = state_id
        state_id = self.add_state(state)

        re_create_io_going_t_ids = []
        re_create_io_going_df_ids = []

        act_outcome_ids_by_name = {oc.name: oc_id for oc_id, oc in state.outcomes.items()}
        act_input_data_port_by_name = {ip.name: ip for ip in state.input_data_ports.values()}
        act_output_data_port_by_name = {op.name: op for op in state.output_data_ports.values()}

        for t in related_transitions['external']['self']:
            new_t_id = self.add_transition(state_id, t.from_outcome, state_id, t.to_outcome, t.transition_id)
            re_create_io_going_t_ids.append(new_t_id)
            assert new_t_id == t.transition_id

        for t in related_transitions['external']['ingoing']:
            new_t_id = self.add_transition(t.from_state, t.from_outcome, state_id, t.to_outcome, t.transition_id)
            re_create_io_going_t_ids.append(new_t_id)
            assert new_t_id == t.transition_id

        for t in related_transitions['external']['outgoing']:
            from_outcome = act_outcome_ids_by_name.get(old_outcome_names[t.from_outcome], None)
            if from_outcome is not None:
                new_t_id = self.add_transition(state_id, from_outcome, t.to_state, t.to_outcome, t.transition_id)
                re_create_io_going_t_ids.append(new_t_id)
                assert new_t_id == t.transition_id

        for old_ip in old_input_data_ports.values():
            ip = act_input_data_port_by_name.get(old_input_data_ports[old_ip.data_port_id].name, None)
            if ip is not None and ip.data_type == old_input_data_ports[old_ip.data_port_id].data_type:
                if isinstance(state, LibraryState) and old_state_was_library:
                    state.input_data_port_runtime_values[ip.data_port_id] = old_input_data_port_runtime_values[old_ip.data_port_id]
                    state.use_runtime_value_input_data_ports[ip.data_port_id] = old_use_runtime_value_input_data_ports[old_ip.data_port_id]
                elif isinstance(state, LibraryState) and not old_state_was_library:
                    state.input_data_port_runtime_values[ip.data_port_id] = old_input_data_ports[old_ip.data_port_id].default_value
                    state.use_runtime_value_input_data_ports[ip.data_port_id] = True
                elif not isinstance(state, LibraryState) and old_state_was_library:
                    if old_use_runtime_value_input_data_ports[old_ip.data_port_id]:
                        ip.default_value = old_input_data_port_runtime_values[old_ip.data_port_id]
                    else:
                        ip.default_value = old_input_data_ports[old_ip.data_port_id].default_value
                else:
                    ip.default_value = old_input_data_ports[old_ip.data_port_id].default_value
        for df in related_data_flows['external']['ingoing']:
            ip = act_input_data_port_by_name.get(old_input_data_ports[df.to_key].name, None)
            if ip is not None and ip.data_type == old_input_data_ports[df.to_key].data_type:
                new_df_id = self.add_data_flow(df.from_state, df.from_key, state_id, ip.data_port_id, df.data_flow_id)
                re_create_io_going_df_ids.append(new_df_id)
                assert new_df_id == df.data_flow_id

        for old_op in old_output_data_ports.values():
            op = act_output_data_port_by_name.get(old_output_data_ports[old_op.data_port_id].name, None)
            if op is not None and op.data_type == old_output_data_ports[old_op.data_port_id].data_type:
                if isinstance(state, LibraryState) and old_state_was_library:
                    state.output_data_port_runtime_values[op.data_port_id] = old_output_data_port_runtime_values[old_op.data_port_id]
                    state.use_runtime_value_output_data_ports[op.data_port_id] = old_use_runtime_value_output_data_ports[old_op.data_port_id]
                elif isinstance(state, LibraryState) and not old_state_was_library:
                    state.output_data_port_runtime_values[op.data_port_id] = old_output_data_ports[old_op.data_port_id].default_value
                    state.use_runtime_value_output_data_ports[op.data_port_id] = True
                elif not isinstance(state, LibraryState) and old_state_was_library:
                    if old_use_runtime_value_output_data_ports[old_op.data_port_id]:
                        op.default_value = old_output_data_port_runtime_values[old_op.data_port_id]
                    else:
                        op.default_value = old_output_data_ports[old_op.data_port_id].default_value
                else:
                    op.default_value = old_output_data_ports[old_op.data_port_id].default_value
        for df in related_data_flows['external']['outgoing']:
            op = act_output_data_port_by_name.get(old_output_data_ports[df.from_key].name, None)
            if op is not None and op.data_type == old_output_data_ports[df.from_key].data_type:
                new_df_id = self.add_data_flow(state_id, op.data_port_id, df.to_state, df.to_key, df.data_flow_id)
                re_create_io_going_df_ids.append(new_df_id)
                assert new_df_id == df.data_flow_id

        if readjust_parent_of_ports:
            state = self.states[state_id]
            state.input_data_ports = state.input_data_ports
            state.output_data_ports = state.output_data_ports
            state.outcomes = state.outcomes
        self.substitute_state.__func__.re_create_io_going_t_ids = re_create_io_going_t_ids
        self.substitute_state.__func__.re_create_io_going_df_ids = re_create_io_going_df_ids
        logger.info("substitute finished")
        return self.states[state_id]

    @lock_state_machine
    @Observable.observed
    def change_state_type(self, state, new_state_class):
        """ Changes the type of the state to another type

        :param state: the state to be changed
        :param new_state_class: the new type of the state
        :return: the new state having the new state type
        :rtype: :py:class:`rafcon.core.states.state.State`
        :raises exceptions.ValueError: if the state does not exist in the container state
        """
        from rafcon.gui.helpers.state import create_new_state_from_state_with_type

        state_id = state.state_id

        if state_id not in self.states:
            raise ValueError("State '{0}' with id '{1}' does not exist".format(state.name, state_id))

        new_state = create_new_state_from_state_with_type(state, new_state_class)
        new_state.parent = self

        assert new_state.state_id == state_id

        self.states[state_id] = new_state

        return new_state

    @lock_state_machine
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

        :param set_final_outcome: if the final_outcome of the state should be set if the income directly connects to
                                    an outcome
        :return: the start state
        """

        # overwrite the start state in the case that a specific start state is specific e.g. by start_from_state
        if self.get_path() in state_machine_execution_engine.start_state_paths:
            for state_id, state in self.states.items():
                if state.get_path() in state_machine_execution_engine.start_state_paths:
                    state_machine_execution_engine.start_state_paths.remove(self.get_path())
                    self._start_state_modified = True
                    return state

        if self.start_state_id is None:
            return None

        # It is possible to connect the income directly with an outcome
        if self.start_state_id == self.state_id:
            if set_final_outcome:
                for transition_id in self.transitions:
                    # the transition of which the from state is None is the transition that directly connects the income
                    if self.transitions[transition_id].from_state is None:
                        to_outcome_id = self.transitions[transition_id].to_outcome
                        self.final_outcome = self.outcomes[to_outcome_id]
                        break
            return self

        return self.states[self.start_state_id]

    @lock_state_machine
    def remove(self, state_element, recursive=True, force=False, destroy=True):
        """Remove item from state

        :param StateElement state_element: State or state element to be removed
        :param bool recursive: Only applies to removal of state and decides whether the removal should be called
            recursively on all child states
        :param bool force: if the removal should be forced without checking constraints
        :param bool destroy: a flag that signals that the state element will be fully removed and disassembled
        """
        if isinstance(state_element, State):
            return self.remove_state(state_element.state_id, recursive=recursive, force=force, destroy=destroy)
        elif isinstance(state_element, Transition):
            return self.remove_transition(state_element.transition_id, destroy=destroy)
        elif isinstance(state_element, DataFlow):
            return self.remove_data_flow(state_element.data_flow_id, destroy=destroy)
        elif isinstance(state_element, ScopedVariable):
            return self.remove_scoped_variable(state_element.data_port_id, destroy=destroy)
        else:
            super(ContainerState, self).remove(state_element, force=force, destroy=destroy)

    # ---------------------------------------------------------------------------------------------
    # ---------------------------------- transition functions -------------------------------------
    # ---------------------------------------------------------------------------------------------

    def check_transition_id(self, transition_id):
        """ Check the transition id and calculate a new one if its None

        :param transition_id: The transition-id to check
        :return: The new transition id
        :raises exceptions.AttributeError: if transition.transition_id already exists
        """
        if transition_id is not None:
            if transition_id in self._transitions.keys():
                raise AttributeError("The transition id %s already exists. Cannot add transition!", transition_id)
        else:
            transition_id = generate_transition_id()
            while transition_id in self._transitions.keys():
                transition_id = generate_transition_id()
        return transition_id

    def basic_transition_checks(self, from_state_id, from_outcome, to_state_id, to_outcome, transition_id):
        pass

    def check_if_outcome_already_connected(self, from_state_id, from_outcome):
        """ check if outcome of from state is not already connected

        :param from_state_id: The source state of the transition
        :param from_outcome: The outcome of the source state to connect the transition to
        :raises exceptions.AttributeError: if the outcome of the state with the state_id==from_state_id
                                            is already connected
        """
        for trans_key, transition in self.transitions.items():
            if transition.from_state == from_state_id:
                if transition.from_outcome == from_outcome:
                    raise AttributeError("Outcome %s of state %s is already connected" %
                                         (str(from_outcome), str(from_state_id)))

    @lock_state_machine
    def create_transition(self, from_state_id, from_outcome, to_state_id, to_outcome, transition_id):
        """ Creates a new transition.

        Lookout: Check the parameters first before creating a new transition

        :param from_state_id: The source state of the transition
        :param from_outcome: The outcome of the source state to connect the transition to
        :param to_state_id: The target state of the transition
        :param to_outcome: The target outcome of a container state
        :param transition_id: An optional transition id for the new transition
        :raises exceptions.AttributeError: if the from or to state is incorrect
        :return: the id of the new transition
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
        with self._transitions_cv:
            self._transitions_cv.notify_all()

        return transition_id

    @lock_state_machine
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

        # Set from_state_id to None for start transitions, as from_state_id and from_outcome should both be None for
        # these transitions
        if from_state_id == self.state_id and from_outcome is None:
            from_state_id = None

        new_transition = Transition(from_state_id, from_outcome, to_state_id, to_outcome, transition_id, self)
        self.transitions[transition_id] = new_transition

        # notify all states waiting for transition to be connected
        with self._transitions_cv:
            self._transitions_cv.notify_all()
        # self.create_transition(from_state_id, from_outcome, to_state_id, to_outcome, transition_id)
        return transition_id

    def get_transition_for_outcome(self, state, outcome):
        """Determines the next transition of a state.

        :param state: The state for which the transition is determined
        :param outcome: The outcome of the state, that is given in the first parameter
        :return: the transition specified by the the state and the outcome
        :raises exceptions.TypeError: if the types of the passed parameters are incorrect
        """
        if not isinstance(state, State):
            raise TypeError("state must be of type State")
        if not isinstance(outcome, Outcome):
            raise TypeError("outcome must be of type Outcome")
        result_transition = None
        for key, transition in self.transitions.items():
            if transition.from_state == state.state_id and transition.from_outcome == outcome.outcome_id:
                result_transition = transition
        return result_transition

    @lock_state_machine
    @Observable.observed
    def remove_transition(self, transition_id, destroy=True):
        """Removes a transition from the container state

        :param transition_id: the id of the transition to remove
        :raises exceptions.AttributeError: if the transition_id is already used
        """
        if transition_id == -1 or transition_id == -2:
            raise AttributeError("The transition_id must not be -1 (Aborted) or -2 (Preempted)")
        if transition_id not in self._transitions:
            raise AttributeError("The transition_id %s does not exist" % str(transition_id))

        self.transitions[transition_id].parent = None
        return self.transitions.pop(transition_id)

    @lock_state_machine
    def remove_outcome_hook(self, outcome_id):
        """Removes internal transition going to the outcome
        """
        for transition_id in list(self.transitions.keys()):
            transition = self.transitions[transition_id]
            if transition.to_outcome == outcome_id and transition.to_state == self.state_id:
                self.remove_transition(transition_id)

    # ---------------------------------------------------------------------------------------------
    # ----------------------------------- data-flow functions -------------------------------------
    # ---------------------------------------------------------------------------------------------
    # TODO input, output oder scope nicht auf sich selbst
    # TODO scope nicht auf andere scope
    # TODO output-in, input-in nur ein data flow
    # TODO data flows mit gleichen Attributen nur einmal

    def check_data_flow_id(self, data_flow_id):
        """ Check the data flow id and calculate a new one if its None

        :param data_flow_id: The data flow id to check
        :return: The new data flow id
        :raises exceptions.AttributeError: if data_flow.data_flow_id already exists
        """
        if data_flow_id is not None:
            if data_flow_id in self._data_flows.keys():
                raise AttributeError("The data_flow id %s already exists. Cannot add data_flow!", data_flow_id)
        else:
            data_flow_id = generate_data_flow_id()
            while data_flow_id in self._data_flows.keys():
                data_flow_id = generate_data_flow_id()
        return data_flow_id

    @lock_state_machine
    @Observable.observed
    # Primary key is data_flow_id.
    def add_data_flow(self, from_state_id, from_data_port_id, to_state_id, to_data_port_id, data_flow_id=None):
        """Adds a data_flow to the container state

        :param from_state_id: The id source state of the data_flow
        :param from_data_port_id: The output_key of the source state
        :param to_state_id: The id target state of the data_flow
        :param to_data_port_id: The input_key of the target state
        :param data_flow_id: an optional id for the data flow
        """
        data_flow_id = self.check_data_flow_id(data_flow_id)

        self.data_flows[data_flow_id] = DataFlow(from_state_id, from_data_port_id, to_state_id, to_data_port_id,
                                                 data_flow_id, self)
        return data_flow_id

    @lock_state_machine
    @Observable.observed
    def remove_data_flow(self, data_flow_id, destroy=True):
        """ Removes a data flow from the container state

        :param int data_flow_id: the id of the data_flow to remove
        :raises exceptions.AttributeError: if the data_flow_id does not exist
        """
        if data_flow_id not in self._data_flows:
            raise AttributeError("The data_flow_id %s does not exist" % str(data_flow_id))

        self._data_flows[data_flow_id].parent = None
        return self._data_flows.pop(data_flow_id)

    @lock_state_machine
    def remove_data_flows_with_data_port_id(self, data_port_id):
        """Remove an data ports whose from_key or to_key equals the passed data_port_id

        :param int data_port_id: the id of a data_port of which all data_flows should be removed, the id can be a input or
                            output data port id

        """
        # delete all data flows in parent related to data_port_id and self.state_id = external data flows
        # checking is_root_state_of_library is only necessary in case of scoped variables, as the scoped variables
        # they are not destroyed by the library state, as the library state does not have a reference to the scoped vars
        if not self.is_root_state and not self.is_root_state_of_library:
            data_flow_ids_to_remove = []
            for data_flow_id, data_flow in self.parent.data_flows.items():
                if data_flow.from_state == self.state_id and data_flow.from_key == data_port_id or \
                                        data_flow.to_state == self.state_id and data_flow.to_key == data_port_id:
                    data_flow_ids_to_remove.append(data_flow_id)

            for data_flow_id in data_flow_ids_to_remove:
                self.parent.remove_data_flow(data_flow_id)

        # delete all data flows in self related to data_port_id and self.state_id = internal data flows
        data_flow_ids_to_remove = []
        for data_flow_id, data_flow in self.data_flows.items():
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
        :raises exceptions.AttributeError: if the name is not in the the scoped_variables dictionary
        """
        for scoped_variable_id, scoped_variable in self.scoped_variables.items():
            if scoped_variable.name == name:
                return scoped_variable_id
        raise AttributeError("Name %s is not in scoped_variables dictionary", name)

    @lock_state_machine
    @Observable.observed
    def add_scoped_variable(self, name, data_type=None, default_value=None, scoped_variable_id=None):
        """ Adds a scoped variable to the container state

        :param name: The name of the scoped variable
        :param data_type: An optional data type of the scoped variable
        :param default_value: An optional default value of the scoped variable
        :param scoped_variable_id: An optional scoped variable id of the
        :return: the unique id of the added scoped variable
        :raises exceptions.ValueError: if the scoped variable is not valid
        """
        if scoped_variable_id is None:
            # All data port ids have to passed to the id generation as the data port id has to be unique inside a state
            scoped_variable_id = generate_data_port_id(self.get_data_port_ids())
        self._scoped_variables[scoped_variable_id] = ScopedVariable(name, data_type, default_value,
                                                                    scoped_variable_id, self)

        # Check for name uniqueness
        valid, message = self._check_data_port_name(self._scoped_variables[scoped_variable_id])
        if not valid:
            self._scoped_variables[scoped_variable_id].parent = None
            del self._scoped_variables[scoped_variable_id]
            raise ValueError(message)

        return scoped_variable_id

    @lock_state_machine
    @Observable.observed
    def remove_scoped_variable(self, scoped_variable_id, destroy=True):
        """Remove a scoped variable from the container state

        :param scoped_variable_id: the id of the scoped variable to remove
        :raises exceptions.AttributeError: if the id of the scoped variable already exists
        """
        if scoped_variable_id not in self._scoped_variables:
            raise AttributeError("A scoped variable with id %s does not exist" % str(scoped_variable_id))

        # delete all data flows connected to scoped_variable
        if destroy:
            self.remove_data_flows_with_data_port_id(scoped_variable_id)

        # delete scoped variable
        self._scoped_variables[scoped_variable_id].parent = None
        return self._scoped_variables.pop(scoped_variable_id)

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

        The data port specified by the state id and data port id is searched in the state itself and in its children.

        :param str state_id: The id of the state the port is in
        :param int port_id:  The id of the port
        :return: The searched port or None if it is not found
        """
        if state_id == self.state_id:
            return self.get_data_port_by_id(port_id)
        for child_state_id, child_state in self.states.items():
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

    def get_data_port_ids(self):
        return list(self._scoped_variables.keys()) + list(self._input_data_ports.keys()) + list(self._output_data_ports.keys())

    # ---------------------------------------------------------------------------------------------
    # ---------------------------------- input data handling --------------------------------------
    # ---------------------------------------------------------------------------------------------

    def get_inputs_for_state(self, state):
        """Retrieves all input data of a state. If several data flows are connected to an input port the
        most current data is used for the specific input port.

        :param state: the state of which the input data is determined
        :return: the input data of the target state
        """
        result_dict = {}

        tmp_dict = self.get_default_input_values_for_state(state)
        result_dict.update(tmp_dict)

        for input_port_key, value in state.input_data_ports.items():
            # for all input keys fetch the correct data_flow connection and read data into the result_dict
            actual_value = None
            actual_value_time = 0
            for data_flow_key, data_flow in self.data_flows.items():

                if data_flow.to_key == input_port_key:
                    if data_flow.to_state == state.state_id:
                        # fetch data from the scoped_data list: the key is the data_port_key + the state_id
                        key = str(data_flow.from_key) + data_flow.from_state
                        if key in self.scoped_data:
                            if actual_value is None or actual_value_time < self.scoped_data[key].timestamp:
                                actual_value = deepcopy(self.scoped_data[key].value)
                                actual_value_time = self.scoped_data[key].timestamp

            if actual_value is not None:
                result_dict[value.name] = actual_value

        return result_dict

    # ---------------------------------------------------------------------------------------------
    # ---------------------------- functions to modify the scoped data ----------------------------
    # ---------------------------------------------------------------------------------------------

    @lock_state_machine
    def add_input_data_to_scoped_data(self, dictionary):
        """Add a dictionary to the scoped data

        As the input_data dictionary maps names to values, the functions looks for the proper data_ports keys in the
        input_data_ports dictionary

        :param dictionary: The dictionary that is added to the scoped data
        :param state: The state to which the input_data was passed (should be self in most cases)
        """
        for dict_key, value in dictionary.items():
            for input_data_port_key, data_port in list(self.input_data_ports.items()):
                if dict_key == data_port.name:
                    self.scoped_data[str(input_data_port_key) + self.state_id] = \
                        ScopedData(data_port.name, value, type(value), self.state_id, ScopedVariable, parent=self)
                    # forward the data to scoped variables
                    for data_flow_key, data_flow in self.data_flows.items():
                        if data_flow.from_key == input_data_port_key and data_flow.from_state == self.state_id:
                            if data_flow.to_state == self.state_id and data_flow.to_key in self.scoped_variables:
                                current_scoped_variable = self.scoped_variables[data_flow.to_key]
                                self.scoped_data[str(data_flow.to_key) + self.state_id] = \
                                    ScopedData(current_scoped_variable.name, value, type(value), self.state_id,
                                               ScopedVariable, parent=self)

    @lock_state_machine
    def add_state_execution_output_to_scoped_data(self, dictionary, state):
        """Add a state execution output to the scoped data

        :param dictionary: The dictionary that is added to the scoped data
        :param state: The state that finished execution and provide the dictionary
        """
        for output_name, value in dictionary.items():
            for output_data_port_key, data_port in list(state.output_data_ports.items()):
                if output_name == data_port.name:
                    if not isinstance(value, data_port.data_type):
                        if (not ((type(value) is float or type(value) is int) and
                                     (data_port.data_type is float or data_port.data_type is int)) and
                                not (isinstance(value, type(None)))):
                            logger.error("The data type of output port {0} should be of type {1}, but is of type {2}".
                                         format(output_name, data_port.data_type, type(value)))
                    self.scoped_data[str(output_data_port_key) + state.state_id] = \
                        ScopedData(data_port.name, value, type(value), state.state_id, OutputDataPort, parent=self)

    @lock_state_machine
    def add_default_values_of_scoped_variables_to_scoped_data(self):
        """Add the scoped variables default values to the scoped_data dictionary

        """
        for key, scoped_var in self.scoped_variables.items():
            self.scoped_data[str(scoped_var.data_port_id) + self.state_id] = \
                ScopedData(scoped_var.name, scoped_var.default_value, scoped_var.data_type, self.state_id,
                           ScopedVariable, parent=self)

    @lock_state_machine
    def update_scoped_variables_with_output_dictionary(self, dictionary, state):
        """Update the values of the scoped variables with the output dictionary of a specific state.

        :param: the dictionary to update the scoped variables with
        :param: the state the output dictionary belongs to
        """
        for key, value in dictionary.items():
            output_data_port_key = None
            # search for the correct output data port key of the source state
            for o_key, o_port in state.output_data_ports.items():
                if o_port.name == key:
                    output_data_port_key = o_key
                    break
            if output_data_port_key is None:
                if not key == "error":
                    logger.warning("Output variable %s was written during state execution, "
                                   "that has no data port connected to it.", str(key))
            for data_flow_key, data_flow in self.data_flows.items():
                if data_flow.from_key == output_data_port_key and data_flow.from_state == state.state_id:
                    if data_flow.to_state == self.state_id:  # is target of data flow own state id?
                        if data_flow.to_key in self.scoped_variables.keys():  # is target data port scoped?
                            current_scoped_variable = self.scoped_variables[data_flow.to_key]
                            self.scoped_data[str(data_flow.to_key) + self.state_id] = \
                                ScopedData(current_scoped_variable.name, value, type(value), state.state_id,
                                           ScopedVariable, parent=self)

    # ---------------------------------------------------------------------------------------------
    # ------------------------ functions to modify the scoped data end ----------------------------
    # ---------------------------------------------------------------------------------------------

    @lock_state_machine
    def change_state_id(self, state_id=None):
        """
        Changes the id of the state to a new id. This functions replaces the old state_id with the new state_id in all
        data flows and transitions.

        :param state_id: The new state if of the state
        """
        old_state_id = self.state_id
        super(ContainerState, self).change_state_id(state_id)
        # Use private variables to change ids to prevent validity checks
        # change id in all transitions
        for transition in self.transitions.values():
            if transition.from_state == old_state_id:
                transition._from_state = self.state_id
            if transition.to_state == old_state_id:
                transition._to_state = self.state_id

        # change id in all data_flows
        for data_flow in self.data_flows.values():
            if data_flow.from_state == old_state_id:
                data_flow._from_state = self.state_id
            if data_flow.to_state == old_state_id:
                data_flow._to_state = self.state_id

    def get_state_for_transition(self, transition):
        """Calculate the target state of a transition

        :param transition: The transition of which the target state is determined
        :return: the to-state of the transition
        :raises exceptions.TypeError: if the transition parameter is of wrong type
        """
        if not isinstance(transition, Transition):
            raise TypeError("transition must be of type Transition")
        # the to_state is None when the transition connects an outcome of a child state to the outcome of a parent state
        if transition.to_state == self.state_id or transition.to_state is None:
            return self
        else:
            return self.states[transition.to_state]

    def write_output_data(self, specific_output_dictionary=None):
        """ Write the scoped data to the output of the state. Called before exiting the container state.

        :param specific_output_dictionary: an optional dictionary to write the output data in
        :return:
        """
        if isinstance(specific_output_dictionary, dict):
            output_dict = specific_output_dictionary
        else:
            output_dict = self.output_data

        for output_name, value in self.output_data.items():
            output_port_id = self.get_io_data_port_id_from_name_and_type(output_name, OutputDataPort)
            actual_value = None
            actual_value_was_written = False
            actual_value_time = 0
            for data_flow_id, data_flow in self.data_flows.items():
                if data_flow.to_state == self.state_id:
                    if data_flow.to_key == output_port_id:
                        scoped_data_key = str(data_flow.from_key) + data_flow.from_state
                        if scoped_data_key in self.scoped_data:
                            # if self.scoped_data[scoped_data_key].timestamp > actual_value_time is True
                            # the data of a previous execution of the same state is overwritten
                            if actual_value is None or self.scoped_data[scoped_data_key].timestamp > actual_value_time:
                                actual_value = deepcopy(self.scoped_data[scoped_data_key].value)
                                actual_value_time = self.scoped_data[scoped_data_key].timestamp
                                actual_value_was_written = True
                        else:
                            if not self.backward_execution:
                                logger.debug(
                                    "Output data with name {0} of state {1} was not found in the scoped data "
                                    "of state {2}. Thus the state did not write onto this output. "
                                    "This can mean a state machine design error.".format(
                                        str(output_name), str(self.states[data_flow.from_state].get_path()),
                                        self.get_path()))
            if actual_value_was_written:
                output_dict[output_name] = actual_value

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
        if not valid and not message.startswith("Invalid state element"):
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

        :param rafcon.core.data_port.DataPort check_data_port: The port to check
        :return: valid, message
        """
        for data_flow in self.data_flows.values():
            # Check whether the data flow connects the given port
            from_port = self.get_data_port(data_flow.from_state, data_flow.from_key)
            to_port = self.get_data_port(data_flow.to_state, data_flow.to_key)
            if check_data_port is from_port or check_data_port is to_port:
                # check if one of the data_types if type 'object'; in this case the data flow is always valid
                if not (from_port.data_type is object or to_port.data_type is object):
                    if not type_inherits_of_type(from_port.data_type, to_port.data_type):
                        return False, "Connection of two non-compatible data types"
        return True, "valid"

    def _check_data_port_id(self, data_port):
        """Checks the validity of a data port id

        Checks whether the id of the given data port is already used by anther data port (input, output, scoped vars)
        within the state.

        :param rafcon.core.data_port.DataPort data_port: The data port to be checked
        :return bool validity, str message: validity is True, when the data port is valid, False else. message gives
            more information especially if the data port is not valid
        """
        # First check inputs and outputs
        valid, message = super(ContainerState, self)._check_data_port_id(data_port)
        if not valid:
            return False, message
        # Container state also has scoped variables
        for scoped_variable_id, scoped_variable in self.scoped_variables.items():
            if data_port.data_port_id == scoped_variable_id and data_port is not scoped_variable:
                return False, "data port id already existing in state"
        return True, message

    def _check_data_port_name(self, data_port):
        """Checks the validity of a data port name

        Checks whether the name of the given data port is already used by anther data port within the state. Names
        must be unique with input data ports, output data ports and scoped variables.

        :param rafcon.core.data_port.DataPort data_port: The data port to be checked
        :return bool validity, str message: validity is True, when the data port is valid, False else. message gives
            more information especially if the data port is not valid
        """
        # First check inputs and outputs
        valid, message = super(ContainerState, self)._check_data_port_name(data_port)
        if not valid:
            return False, message

        if data_port.data_port_id in self.scoped_variables:
            for scoped_variable in self.scoped_variables.values():
                if data_port.name == scoped_variable.name and data_port is not scoped_variable:
                    return False, "scoped variable name already existing in state's scoped variables"

        return True, message

    def _check_data_flow_validity(self, check_data_flow):
        """Checks the validity of a data flow

        Calls further checks to inspect the id, ports and data types.

        :param rafcon.core.data_flow.DataFlow check_data_flow: The data flow to be checked
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

        :param rafcon.core.data_flow.DataFlow data_flow: The data flow to be checked
        :return bool validity, str message: validity is True, when the data flow is valid, False else. message gives
            more information especially if the data flow is not valid
        """
        data_flow_id = data_flow.data_flow_id
        if data_flow_id in self.data_flows and data_flow is not self.data_flows[data_flow_id]:
            return False, "data_flow_id already existing"
        return True, "valid"

    def _check_data_flow_ports(self, data_flow):
        """Checks the validity of the ports of a data flow

        Checks whether the ports of a data flow are existing and whether it is allowed to connect these ports.

        :param rafcon.core.data_flow.DataFlow data_flow: The data flow to be checked
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
            return False, "Data flow origin not existing -> {0}".format(data_flow)
        to_data_port = self.get_data_port(to_state_id, to_data_port_id)
        if not to_data_port:
            return False, "Data flow target not existing -> {0}".format(data_flow)

        # Data_ports without parents are not allowed to be connected twice
        if not from_data_port.parent:
            return False, "Source data port does not have a parent -> {0}".format(data_flow)
        if not to_data_port.parent:
            return False, "Target data port does not have a parent -> {0}".format(data_flow)

        # Check if data ports are identical
        if from_data_port is to_data_port:
            return False, "Source and target data ports of data flow must not be identical -> {}".format(data_flow)

        # Check, whether the origin of the data flow is valid
        if from_state_id == self.state_id:  # data_flow originates in container state
            if from_data_port_id not in self.input_data_ports and from_data_port_id not in self.scoped_variables:
                return False, "Data flow origin port must be an input port or scoped variable, when the data flow " \
                              "starts in the parent state -> {0}".format(data_flow)
        else:  # data flow originates in child state
            if from_data_port_id not in from_data_port.parent.output_data_ports:
                return False, "Data flow origin port must be an output port, when the data flow " \
                              "starts in the child state -> {0}".format(data_flow)

        # Check, whether the target of a data flow is valid
        if to_state_id == self.state_id:  # data_flow ends in container state
            if to_data_port_id not in self.output_data_ports and to_data_port_id not in self.scoped_variables:
                return False, "Data flow target port must be an output port or scoped variable, when the data flow " \
                              "goes to the parent state -> {0}".format(data_flow)
        else:  # data_flow ends in child state
            if to_data_port_id not in to_data_port.parent.input_data_ports:
                return False, "Data flow target port must be an input port, when the data flow goes to a child state" \
                              " -> {0}".format(data_flow)

        # Check if data flow connects two scoped variables
        if isinstance(from_data_port, ScopedVariable) and isinstance(to_data_port, ScopedVariable):
            return False, "Data flows must not connect two scoped variables -> {}".format(data_flow)

        # Check, whether the target port is already connected
        for existing_data_flow in self.data_flows.values():
            to_data_port_existing = self.get_data_port(existing_data_flow.to_state, existing_data_flow.to_key)
            from_data_port_existing = self.get_data_port(existing_data_flow.from_state, existing_data_flow.from_key)
            if to_data_port is to_data_port_existing and data_flow is not existing_data_flow:
                if from_data_port is from_data_port_existing:
                    return False, "Exactly the same data flow is already existing -> {0}".format(data_flow)

        return True, "valid"

    def _check_data_flow_types(self, check_data_flow):
        """Checks the validity of the data flow connection

        Checks whether the ports of a data flow have matching data types.

        :param rafcon.core.data_flow.DataFlow check_data_flow: The data flow to be checked
        :return bool validity, str message: validity is True, when the data flow is valid, False else. message gives
            more information especially if the data flow is not valid
        """
        # Check whether the data types or origin and target fit
        from_data_port = self.get_data_port(check_data_flow.from_state, check_data_flow.from_key)
        to_data_port = self.get_data_port(check_data_flow.to_state, check_data_flow.to_key)
        # Connections from the data port type "object" are always allowed
        if from_data_port.data_type is object:
            return True, "valid"
        if not type_inherits_of_type(from_data_port.data_type, to_data_port.data_type):
            return False, "Data flow (id: {0}) with origin state \"{1}\" (from data port name: {2}) " \
                          "and target state \"{3}\" (to data port name: {4}) " \
                          "do not have matching data types (from '{5}' to '{6}')".format(
                              check_data_flow.data_flow_id,
                              from_data_port.parent.name,
                              from_data_port.name,
                              to_data_port.parent.name,
                              to_data_port.name,
                              from_data_port.data_type,
                              to_data_port.data_type)
        return True, "valid"

    def _check_transition_validity(self, check_transition):
        """Checks the validity of a transition

        Calls further checks to inspect the id, origin, target and connection of the transition.

        :param rafcon.core.transition.Transition check_transition: The transition to be checked
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

        :param rafcon.core.transition.Transition transition: The transition to be checked
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

        :param rafcon.core.transition.Transition start_transition: The transition to be checked
        :return bool validity, str message: validity is True, when the transition is valid, False else. message gives
            more information especially if the transition is not valid
        """
        for transition in self.transitions.values():
            if transition.from_state is None:
                if start_transition is not transition:
                    return False, "Only one start transition is allowed"

        if start_transition.from_outcome is not None:
            return False, "from_outcome must not be set in start transition"

        return self._check_transition_target(start_transition)

    def _check_transition_target(self, transition):
        """Checks the validity of a transition target

        Checks whether the transition target is valid.

        :param rafcon.core.transition.Transition transition: The transition to be checked
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

        :param rafcon.core.transition.Transition transition: The transition to be checked
        :return bool validity, str message: validity is True, when the transition is valid, False else. message gives
            more information especially if the transition is not valid
        """
        from_state_id = transition.from_state
        from_outcome_id = transition.from_outcome

        if from_state_id == self.state_id:
            return False, "from_state_id of transition must not be the container state itself." \
                          " In the case of a start transition both the from state and the from_outcome are None."

        if from_state_id != self.state_id and from_state_id not in self.states:
            return False, "from_state not existing"

        from_outcome = self.get_outcome(from_state_id, from_outcome_id)
        if from_outcome is None:
            return False, "from_outcome not existing in from_state"

        return True, "valid"

    def _check_transition_connection(self, check_transition):
        """Checks the validity of a transition connection

        Checks whether the transition is allowed to connect the origin with the target.

        :param rafcon.core.transition.Transition check_transition: The transition to be checked
        :return bool validity, str message: validity is True, when the transition is valid, False else. message gives
            more information especially if the transition is not valid
        """
        from_state_id = check_transition.from_state
        from_outcome_id = check_transition.from_outcome
        to_state_id = check_transition.to_state
        to_outcome_id = check_transition.to_outcome

        # check for connected origin
        for transition in self.transitions.values():
            if transition.from_state == from_state_id:
                if transition.from_outcome == from_outcome_id:
                    if check_transition is not transition:
                        return False, "transition origin already connected to another transition"

        if from_state_id in self.states and to_state_id in self.states and to_outcome_id is not None:
            return False, "no transition from one outcome to another one on the same hierarchy allowed"

        return True, "valid"

    # ---------------------------------------------------------------------------------------------
    # ----------------------------------------- misc ----------------------------------------------
    # ---------------------------------------------------------------------------------------------

    def get_states_statistics(self, hierarchy_level):
        """
        Returns the numer of child states
        :return:
        """
        number_of_all_child_states = 0
        max_child_hierarchy_level = 0
        for s in self.states.values():
            child_hierarchy_level = 0
            number_of_child_states, child_hierarchy_level = s.get_states_statistics(child_hierarchy_level)
            number_of_all_child_states += number_of_child_states
            if child_hierarchy_level > max_child_hierarchy_level:
                max_child_hierarchy_level = child_hierarchy_level

        return number_of_all_child_states + 1, hierarchy_level + max_child_hierarchy_level + 1

    def get_number_of_transitions(self):
        """
        Returns the number of transitions
        :return:
        """
        number_of_all_transitions = 0
        for s in self.states.values():
            number_of_all_transitions += s.get_number_of_transitions()
        return number_of_all_transitions + len(self.transitions)

    def get_number_of_data_flows(self):
        """
        Returns the number of data flows
        :return:
        """
        number_of_all_data_flows = 0
        for s in self.states.values():
            number_of_all_data_flows += s.get_number_of_data_flows()
        return number_of_all_data_flows + len(self.data_flows)

    # ---------------------------------------------------------------------------------------------
    # ------------ Properties for all class fields that must be observed by gtkmvc3 ----------------
    # ---------------------------------------------------------------------------------------------

    @property
    def states(self):
        """Property for the _states field

        The setter-method substitute ContainerState.states which is a dict. The method checks if the elements are
        of the right type  or will cancel the operation and recover old outcomes. The method does check validity of
        the elements by calling the parent-setter.

        """
        return self._states

    @states.setter
    @lock_state_machine
    @Observable.observed
    def states(self, states):
        """ Setter for _states field

        See property

        :param states: Dictionary of States
        :raises exceptions.TypeError: if the states parameter is of wrong type
        :raises exceptions.AttributeError: if the keys of the dictionary and the state_ids in the dictionary do not match
        """
        if not isinstance(states, dict):
            raise TypeError("states must be of type dict")
        if [state_id for state_id, state in states.items() if not isinstance(state, State)]:
            raise TypeError("element of container_state.states must be of type State")
        if [state_id for state_id, state in states.items() if not state_id == state.state_id]:
            raise AttributeError("The key of the state dictionary and the id of the state do not match")

        old_states = self._states
        self._states = states
        for state_id, state in states.items():
            try:
                state.parent = self
            except ValueError:
                self._states = old_states
                raise

        # check that all old_states are no more referencing self as their parent
        for old_state in old_states.values():
            if old_state not in self._states.values() and old_state.parent is self:
                old_state.parent = None

    @property
    def transitions(self):
        """Property for the _transitions field

        The setter-method substitute ContainerState._transitions with a handed dictionary. The method checks if the
        elements are of the right type and the keys consistent (Transition.transition_id==key). The method does check
        validity of the elements by calling the parent-setter and in case of failure cancel the operation and
        recover old _transitions dictionary.

        :return: Dictionary transitions[transition_id] of :class:`rafcon.core.transition.Transition`
        :rtype: dict
        """
        return self._transitions

    @transitions.setter
    @lock_state_machine
    @Observable.observed
    def transitions(self, transitions):
        """ Setter for _transitions field

        See property

        :param: transitions: Dictionary transitions[transition_id] of :class:`rafcon.core.transition.Transition`
        :raises exceptions.TypeError: if the transitions parameter has the wrong type
        :raises exceptions.AttributeError: if the keys of the transitions dictionary and the transition_ids of the
                                            transitions in the dictionary do not match
        """
        if not isinstance(transitions, dict):
            raise TypeError("transitions must be of type dict")
        if [t_id for t_id, transition in transitions.items() if not isinstance(transition, Transition)]:
            raise TypeError("element of transitions must be of type Transition")
        if [t_id for t_id, transition in transitions.items() if not t_id == transition.transition_id]:
            raise AttributeError("The key of the transition dictionary and the id of the transition do not match")

        old_transitions = self._transitions
        self._transitions = transitions
        transition_ids_to_delete = []
        for transition_id, transition in transitions.items():
            try:
                transition.parent = self
            except (ValueError, RecoveryModeException) as e:
                if type(e) is RecoveryModeException:
                    logger.exception("Recovery error:")
                    if e.do_delete_item:
                        transition_ids_to_delete.append(transition.transition_id)
                else:
                    self._transitions = old_transitions
                    raise

        self._transitions = dict((transition_id, t) for (transition_id, t) in self._transitions.items()
                                 if transition_id not in transition_ids_to_delete)

        # check that all old_transitions are no more referencing self as there parent
        for old_transition in old_transitions.values():
            if old_transition not in self._transitions.values() and old_transition.parent is self:
                old_transition.parent = None

    @property
    def data_flows(self):
        """Property for the _data_flows field

        The setter-method substitute ContainerState._data_flows with handed dictionary. The method checks if the
        elements are of the right type and the keys consistent (DataFlow.data_flow_id==key). The method does check
        validity of the elements by calling the parent-setter and in case of failure cancel the operation and
        recover old _data_flows dictionary.

        :return: Dictionary data_flows[data_flow_id] of :class:`rafcon.core.data_flow.DataFlow`
        :rtype: dict
        """
        return self._data_flows

    @data_flows.setter
    @lock_state_machine
    @Observable.observed
    def data_flows(self, data_flows):
        """ Setter for _data_flows field

        See property

        :param dict data_flows: Dictionary data_flows[data_flow_id] of :class:`rafcon.core.data_flow.DataFlow`
        :raises exceptions.TypeError: if the data_flows parameter has the wrong type
        :raises exceptions.AttributeError: if the keys of the data_flows dictionary and the data_flow_ids of the
                                            data flows in the dictionary do not match
        """
        if not isinstance(data_flows, dict):
            raise TypeError("data_flows must be of type dict")
        if [df_id for df_id, data_flow in data_flows.items() if not isinstance(data_flow, DataFlow)]:
            raise TypeError("element of data_flows must be of type DataFlow")
        if [df_id for df_id, data_flow in data_flows.items() if not df_id == data_flow.data_flow_id]:
            raise AttributeError("The key of the data flow dictionary and the id of the data flow do not match")

        old_data_flows = self._data_flows
        self._data_flows = data_flows
        data_flow_ids_to_delete = []
        for data_flow_id, data_flow in data_flows.items():
            try:
                data_flow.parent = self
            except (ValueError, RecoveryModeException) as e:
                if type(e) is RecoveryModeException:
                    logger.error("Recovery error:")
                    if e.do_delete_item:
                        data_flow_ids_to_delete.append(data_flow.data_flow_id)
                else:
                    self._data_flows = old_data_flows
                    raise

        self._data_flows = dict((data_flow_id, d) for (data_flow_id, d) in self._data_flows.items()
                                if data_flow_id not in data_flow_ids_to_delete)

        # check that all old_data_flows are no more referencing self as there parent
        for old_data_flow in old_data_flows.values():
            if old_data_flow not in self._data_flows.values() and old_data_flow.parent is self:
                old_data_flow.parent = None

    @property
    def start_state_id(self):
        """ The start state is the state to which the first transition goes to.

        The setter-method creates a unique first transition to the state with the given id.
        Existing first transitions are removed. If the given state id is None, the first transition is removed.

        :return: The id of the start state
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
    @lock_state_machine
    # @Observable.observed
    def start_state_id(self, start_state_id, to_outcome=None):
        """Set the start state of the container state

        See property

        :param start_state_id: The state id of the state which should be executed first in the Container state
        :raises exceptions.ValueError: if the start_state_id does not exist in
                                    :py:attr:`rafcon.core.states.container_state.ContainerState.states`
        """
        if start_state_id is not None and start_state_id not in self.states:
            raise ValueError("start_state_id does not exist")

        if start_state_id is None and to_outcome is not None:  # this is the case if the start state is the state itself
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

        The setter-method ContainerState._scoped_variables with a handed dictionary. The method checks if the elements
        are of the right type and the keys consistent (Transition.transition_id==key). The method does check validity
        of the elements by calling the parent-setter and in case of failure cancel the operation and recover old
        _scoped_variables dictionary.

        :return: Dictionary scoped_variables[data_port_id] of :class:`rafcon.core.scope.ScopedVariable`
        :rtype: dict
        """
        return self._scoped_variables

    @scoped_variables.setter
    @lock_state_machine
    @Observable.observed
    def scoped_variables(self, scoped_variables):
        """ Setter for _scoped_variables field

        See property

        :param dict scoped_variables: Dictionary scoped_variables[data_port_id] of :class:`rafcon.core.scope.ScopedVariable`
        :raises exceptions.TypeError: if the scoped_variables parameter has the wrong type
        :raises exceptions.AttributeError: if the keys of the scoped_variables dictionary and the ids
                                            of the scoped variables in the dictionary do not match
        """
        if not isinstance(scoped_variables, dict):
            raise TypeError("scoped_variables must be of type dict")
        if [sv_id for sv_id, sv in scoped_variables.items() if not isinstance(sv, ScopedVariable)]:
            raise TypeError("element of scope variable must be of type ScopedVariable")
        if [sv_id for sv_id, sv in scoped_variables.items() if not sv_id == sv.data_port_id]:
            raise AttributeError("The key of the scope variable dictionary and "
                                 "the id of the scope variable do not match")

        old_scoped_variables = self._scoped_variables
        self._scoped_variables = scoped_variables
        for port_id, scoped_variable in scoped_variables.items():
            try:
                scoped_variable.parent = self
            except ValueError:
                self._scoped_variables = old_scoped_variables
                raise

        # check that all old_scoped_variables are no more referencing self as there parent
        for old_scoped_variable in old_scoped_variables.values():
            if old_scoped_variable not in self._scoped_variables.values() and old_scoped_variable.parent is self:
                old_scoped_variable.parent = None

    @property
    def scoped_data(self):
        """Property for the _scoped_data field

        """
        return self._scoped_data

    @scoped_data.setter
    @lock_state_machine
    # @Observable.observed
    def scoped_data(self, scoped_data):
        if not isinstance(scoped_data, dict):
            raise TypeError("scoped_results must be of type dict")
        for key, s in scoped_data.items():
            if not isinstance(s, ScopedData):
                raise TypeError("element of scoped_data must be of type ScopedData")
        self._scoped_data = scoped_data

    @property
    def child_execution(self):
        """Property for the _child_execution field
        """
        if self.state_execution_status is StateExecutionStatus.EXECUTE_CHILDREN:
            return True
        else:
            return False
