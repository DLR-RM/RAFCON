# Copyright (C) 2015-2018 DLR
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
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""Action class for history

The Action-Class provides a general redo or undo functionality for any action, as long as the the class object was
initialized with consistent arguments.

This general Action (one procedure for all possible edition) procedure is expansive and complex, therefore it is aimed
to define specific _-Action-Classes for simple/specific edit actions.
"""
from future.utils import string_types
from builtins import object
from builtins import str
import copy
import json
import difflib

from gtkmvc3.model_mt import ModelMT
from jsonconversion.decoder import JSONObjectDecoder
from jsonconversion.encoder import JSONObjectEncoder

from rafcon.core.constants import UNIQUE_DECIDER_STATE_ID
from rafcon.core.global_variable_manager import GlobalVariableManager
from rafcon.core.library_manager import LibraryManager
from rafcon.core.script import Script
from rafcon.core.state_elements.data_flow import DataFlow
from rafcon.core.state_elements.data_port import DataPort
from rafcon.core.state_elements.data_port import InputDataPort, OutputDataPort
from rafcon.core.state_elements.logical_port import Income, Outcome
from rafcon.core.state_elements.scope import ScopedData, ScopedVariable
from rafcon.core.state_elements.transition import Transition
from rafcon.core.state_machine import StateMachine
from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState, DeciderState
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState, ContainerState
from rafcon.core.states.library_state import LibraryState
from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from rafcon.core.states.state import State
from rafcon.gui.models import ContainerStateModel, LibraryStateModel
from rafcon.gui.models.signals import MetaSignalMsg, ActionSignalMsg
from rafcon.gui.utils.notification_overview import NotificationOverview

from rafcon.utils import log
from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE, BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS
from rafcon.utils.storage_utils import substitute_modules

logger = log.get_logger(__name__)

core_object_list = [Transition, DataFlow, Outcome, InputDataPort, OutputDataPort, ScopedData, ScopedVariable, Script,
                    GlobalVariableManager, LibraryManager, StateMachine,
                    ExecutionState, HierarchyState, BarrierConcurrencyState, PreemptiveConcurrencyState, LibraryState,
                    DeciderState]

DEBUG_META_REFERENCES = False
HISTORY_DEBUG_LOG_FILE = RAFCON_TEMP_PATH_BASE + '/test_file.txt'

STATE_TUPLE_JSON_STR_INDEX = 0
STATE_TUPLE_CHILD_STATES_INDEX = 1
STATE_TUPLE_META_DICT_INDEX = 2
STATE_TUPLE_PATH_INDEX = 3
STATE_TUPLE_SCRIPT_TEXT_INDEX = 4
STATE_TUPLE_FILE_SYSTEM_PATH_INDEX = 5
STATE_TUPLE_SEMANTIC_DATA_INDEX = 6


def get_state_tuple(state, state_m=None):
    """ Generates a tuple that holds the state as yaml-strings and its meta data in a dictionary.
    The tuple consists of:
    [0] json_str for state,
    [1] dict of child_state tuples,
    [2] dict of model_meta-data of self and elements
    [3] path of state in state machine
    [4] script_text
    [5] file system path
    [6] semantic data
    #   states-meta - [state-, transitions-, data_flows-, outcomes-, inputs-, outputs-, scopes, states-meta]

    :param rafcon.core.states.state.State state: The state that should be stored
    :return: state_tuple tuple
    """
    state_str = json.dumps(state, cls=JSONObjectEncoder,
                           indent=4, check_circular=False, sort_keys=True)

    state_tuples_dict = {}
    if isinstance(state, ContainerState):
        # print(state.states, "\n")
        for child_state_id, child_state in state.states.items():
            # print("child_state: %s" % child_state_id, child_state, "\n")
            state_tuples_dict[child_state_id] = get_state_tuple(child_state)

    state_meta_dict = {} if state_m is None else get_state_element_meta(state_m)

    script_content = state.script.script if isinstance(state, ExecutionState) else None

    state_tuple = (state_str, state_tuples_dict, state_meta_dict, state.get_path(), script_content,
                   state.file_system_path, copy.deepcopy(state.semantic_data))

    return state_tuple


def get_state_from_state_tuple(state_tuple):
    # Transitions and data flows are not added, as also states are not added
    # We have to wait until the child states are loaded, before adding transitions and data flows, as otherwise the
    # validity checks for transitions and data flows would fail
    state_info = json.loads(state_tuple[STATE_TUPLE_JSON_STR_INDEX],
                            cls=JSONObjectDecoder, substitute_modules=substitute_modules)
    if not isinstance(state_info, tuple):
        state = state_info
    else:
        state = state_info[0]
        transitions = state_info[1]
        data_flows = state_info[2]

    state._file_system_path = state_tuple[STATE_TUPLE_FILE_SYSTEM_PATH_INDEX]
    # print("got state tuple with sd", state_tuple[STATE_TUPLE_SEMANTIC_DATA_INDEX], state_tuple[STATE_TUPLE_FILE_SYSTEM_PATH_INDEX])
    state.semantic_data = state_tuple[STATE_TUPLE_SEMANTIC_DATA_INDEX]

    if isinstance(state, BarrierConcurrencyState):
        # logger.debug("\n\ninsert decider_state\n\n")
        child_state = get_state_from_state_tuple(state_tuple[STATE_TUPLE_CHILD_STATES_INDEX][UNIQUE_DECIDER_STATE_ID])
        # do_storage_test(child_state)
        for t in list(state.transitions.values()):
            if UNIQUE_DECIDER_STATE_ID in [t.from_state, t.to_state]:
                state.remove_transition(t.transition_id)
        try:
            state.add_state(child_state)
        except:
            if not UNIQUE_DECIDER_STATE_ID in state.states:
                logger.error("Could not insert DeciderState!!! while it is in NOT in already!!! {0} {1}".format(
                    UNIQUE_DECIDER_STATE_ID in state.states, child_state.state_id == UNIQUE_DECIDER_STATE_ID))

    if isinstance(state, ExecutionState):
        state.script_text = state_tuple[STATE_TUPLE_SCRIPT_TEXT_INDEX]
    # print("------------- ", state)
    for child_state_id, child_state_tuple in state_tuple[STATE_TUPLE_CHILD_STATES_INDEX].items():
        child_state = get_state_from_state_tuple(child_state_tuple)
        # do_storage_test(child_state)

        # print("++++ new cild", child_state  # child_state_tuple, child_state)
        if not child_state.state_id == UNIQUE_DECIDER_STATE_ID:
            try:
                state.add_state(child_state)
            except Exception as e:
                logger.debug(str(e))
                logger.error(
                    "try to add state %s to state %s with states %s" % (child_state, state, state.states.keys()))

        def print_states(state):
            if isinstance(state, ContainerState):
                for state_id, child_state in state.states.items():
                    logger.verbose(child_state.get_path())
                    print_states(child_state)
                    # print("got from tuple:")
                    # print_states(state)

    # Child states were added, now we can add transitions and data flows
    if isinstance(state_info, tuple):
        state.transitions = transitions
        state.data_flows = data_flows

    return state


def meta_dump_or_deepcopy(meta):
    """Function to observe meta data vivi-dict copy process and to debug it at one point"""
    if DEBUG_META_REFERENCES:  # debug copy
        from rafcon.gui.helpers.meta_data import meta_data_reference_check
        meta_data_reference_check(meta)
    return copy.deepcopy(meta)


def get_state_element_meta(state_model, with_parent_linkage=True, with_verbose=False, level=None):
    meta_dict = {'state': copy.deepcopy(state_model.meta), 'is_start': False, 'data_flows': {}, 'transitions': {},
                 'outcomes': {}, 'input_data_ports': {}, 'output_data_ports': {}, 'scoped_variables': {}, 'states': {},
                 'related_parent_transitions': {}, 'related_parent_data_flows': {}}
    if with_parent_linkage:
        with_parent_linkage = False
        if not state_model.state.is_root_state:
            child_state_id = state_model.state.state_id
            for transition_m in state_model.parent.transitions:
                transition = transition_m.transition
                if transition.from_state == child_state_id or transition.to_state == child_state_id:
                    meta_dict['related_parent_transitions'][transition.transition_id] = meta_dump_or_deepcopy(transition_m.meta)
            for data_flow_m in state_model.parent.data_flows:
                data_flow = data_flow_m.data_flow
                if data_flow.from_state == child_state_id or data_flow.to_state == child_state_id:
                    meta_dict['related_parent_data_flows'][data_flow.data_flow_id] = meta_dump_or_deepcopy(data_flow_m.meta)

    # logger.verbose("get meta {0} {1}".format(state_model.state.state_id, state_model.meta))
    if with_verbose:
        logger.verbose("STORE META for STATE: {0} {1}".format(state_model.state.state_id, state_model.state.name))
    meta_dict['is_start'] = state_model.is_start
    for elem in state_model.outcomes:
        meta_dict['outcomes'][elem.outcome.outcome_id] = meta_dump_or_deepcopy(elem.meta)
        if with_verbose:
            logger.verbose("outcome: id {0} all ids {1} ids in dict {2}"
                           "".format(elem.outcome.outcome_id, elem.parent.state.outcomes.keys(), meta_dict['outcomes'].keys()))
    for elem in state_model.input_data_ports:
        meta_dict['input_data_ports'][elem.data_port.data_port_id] = meta_dump_or_deepcopy(elem.meta)
        if with_verbose:
            logger.verbose("input: id {0} all ids {1} ids in dict {2}"
                           "".format(elem.data_port.data_port_id, elem.parent.state.input_data_ports.keys(), meta_dict['input_data_ports'].keys()))
    for elem in state_model.output_data_ports:
        meta_dict['output_data_ports'][elem.data_port.data_port_id] = meta_dump_or_deepcopy(elem.meta)
        if with_verbose:
            logger.verbose("output: id {0} all ids {1} ids in dict {2}"
                           "".format(elem.data_port.data_port_id, elem.parent.state.output_data_ports.keys(), meta_dict['output_data_ports'].keys()))

    # logger.verbose("store meta of state id {0} data -> {1}".format(state_model.state.state_id, state_model.meta))
    meta_dict['state'] = meta_dump_or_deepcopy(state_model.meta)
    if isinstance(state_model, ContainerStateModel):
        for child_state_id, child_state_m in state_model.states.items():
            meta_dict['states'][child_state_m.state.state_id] = get_state_element_meta(child_state_m, with_parent_linkage)
            if with_verbose:
                logger.verbose("FINISHED STORE META for STATE: id {0} other ids {1} parent state-id {2}"
                               "".format(child_state_id, meta_dict['states'].keys(), state_model.state.state_id))
        for elem in state_model.transitions:
            meta_dict['transitions'][elem.transition.transition_id] = meta_dump_or_deepcopy(elem.meta)
            if with_verbose:
                logger.verbose("transition: id {0} all ids {1} ids in dict {2}"
                               "".format(elem.transition.transition_id, elem.parent.state.transitions.keys(), meta_dict['transitions'].keys()))
        for elem in state_model.data_flows:
            meta_dict['data_flows'][elem.data_flow.data_flow_id] = meta_dump_or_deepcopy(elem.meta)
            if with_verbose:
                logger.verbose("data_flow: id {0} all ids {1} ids in dict {2}"
                               "".format(elem.data_flow.data_flow_id, elem.parent.state.data_flows.keys(), meta_dict['data_flows'].keys()))
        for elem in state_model.scoped_variables:
            meta_dict['scoped_variables'][elem.scoped_variable.data_port_id] = meta_dump_or_deepcopy(elem.meta)
            if with_verbose:
                logger.verbose("scoped_variable: id {0} all ids {1} ids in dict {2}"
                               "".format(elem.scoped_variable.data_port_id, elem.parent.state.scoped_variables.keys(), meta_dict['scoped_variables'].keys()))

    # store meta_data_was_scaled parameter to avoid repetitive port scaling
    if isinstance(state_model, LibraryStateModel):
        meta_dict['meta_data_was_scaled'] = state_model.meta_data_was_scaled

    return meta_dict


def check_state_model_for_is_start_state(state_model):
        if state_model.is_start and not state_model.state.is_root_state and state_model.state.parent.get_start_state() \
                and not state_model.state.parent.get_start_state().state_id == state_model.state.state_id:
            if not (isinstance(state_model.parent.state, BarrierConcurrencyState) or
                    isinstance(state_model.parent.state, PreemptiveConcurrencyState)):
                logger.warning("start_state_id of {0} is not consistent with is_start_state in state-model of {1} "
                               "{2}!={3}".format(state_model.parent.state, state_model.state,
                                                 state_model.state.parent.get_start_state().state_id,
                                                 state_model.state.state_id))


def insert_state_meta_data(meta_dict, state_model, with_verbose=False, level=None):
    # meta_dict = {'state': state_model.meta, 'data_flows': {}, 'transitions': {}, 'outcomes': {},
    #              'input_data_ports': {}, 'output_data_ports': {}, 'scoped_variables': {}}

    def missing_meta_data_log_msg(state_model, elem, meta_dict, dict_key, existing_model_list):
        logger.verbose("Storage Dict seems to miss Meta-Data of {5} in State: {0} {1} for {5}:"
                       " {2}\nreal: {3}\nstorage: {4}".format(state_model.state.state_id,
                                                              state_model.state.name,
                                                              elem,
                                                              existing_model_list,
                                                              meta_dict[dict_key],
                                                              dict_key[:-1].replace('_', '-')))

    # logger.verbose("insert meta data of state {0} data -> {1}".format(state_model.state.state_id, meta_dict['state']))
    state_model.meta = meta_dump_or_deepcopy(meta_dict['state'])
    if with_verbose:
        logger.verbose("INSERT META for STATE: {0} {1}".format(state_model.state.state_id, state_model.state.name))

    for elem in state_model.outcomes:
        if elem.outcome.outcome_id in meta_dict['outcomes']:
            elem.meta = meta_dump_or_deepcopy(meta_dict['outcomes'][elem.outcome.outcome_id])
        else:
            missing_meta_data_log_msg(state_model, elem.outcome, meta_dict, 'outcomes',
                                      [oc_m.outcome.outcome_id for oc_m in state_model.outcomes])
    for elem in state_model.input_data_ports:
        if elem.data_port.data_port_id in meta_dict['input_data_ports']:
            elem.meta = meta_dump_or_deepcopy(meta_dict['input_data_ports'][elem.data_port.data_port_id])
        else:
            missing_meta_data_log_msg(state_model, elem.data_port, meta_dict, 'input_data_ports',
                                      [ip_m.data_port.data_port_id for ip_m in state_model.input_data_ports])

    for elem in state_model.output_data_ports:
        if elem.data_port.data_port_id in meta_dict['output_data_ports']:
            elem.meta = meta_dump_or_deepcopy(meta_dict['output_data_ports'][elem.data_port.data_port_id])
        else:
            missing_meta_data_log_msg(state_model, elem.data_port, meta_dict, 'output_data_ports',
                                      [op_m.data_port.data_port_id for op_m in state_model.output_data_ports])

    if isinstance(state_model, ContainerStateModel):
        for state_id, state_m in state_model.states.items():
            # TODO check if decider miss the meta or it has to be like that UNDO, REDO?
            if state_m.state.state_id in meta_dict['states']:
                if level is None:
                    insert_state_meta_data(meta_dict['states'][state_m.state.state_id], state_m, with_verbose)
                elif level > 0:
                    insert_state_meta_data(meta_dict['states'][state_m.state.state_id], state_m, with_verbose, level - 1)
                else:
                    pass
            else:
                if not UNIQUE_DECIDER_STATE_ID == state_m.state.state_id:
                    logger.warning("no meta data for STATE: '{0}' in storage".format(state_m.state.state_id))

            if with_verbose:
                logger.verbose("FINISHED META for STATE: ", state_m.state.state_id)
        for elem in state_model.transitions:
            if elem.transition.transition_id in meta_dict['transitions']:
                elem.meta = meta_dump_or_deepcopy(meta_dict['transitions'][elem.transition.transition_id])
            else:
                # TODO check if BarrierState miss the meta or it has to be like that UNDO, REDO?
                if not isinstance(state_model.state, BarrierConcurrencyState):
                    missing_meta_data_log_msg(state_model, elem.transition, meta_dict, 'transitions',
                                              [t_m.transition.transition_id for t_m in state_model.transitions])

        for elem in state_model.data_flows:
            if elem.data_flow.data_flow_id in meta_dict['data_flows']:
                elem.meta = meta_dump_or_deepcopy(meta_dict['data_flows'][elem.data_flow.data_flow_id])
            else:
                missing_meta_data_log_msg(state_model, elem.data_flow, meta_dict, 'data_flows',
                                          [df_m.data_flow.data_flow_id for df_m in state_model.data_flows])

        for elem in state_model.scoped_variables:
            if elem.scoped_variable.data_port_id in meta_dict['scoped_variables']:
                elem.meta = meta_dump_or_deepcopy(meta_dict['scoped_variables'][elem.scoped_variable.data_port_id])
            else:
                missing_meta_data_log_msg(state_model, elem.scoped_variable, meta_dict, 'scoped_variables',
                                          [sv_m.scoped_variable.data_port_id for sv_m in state_model.scoped_variables])

    # set meta_data_was_scaled parameter to avoid repetitive port scaling
    if isinstance(state_model, LibraryStateModel):
        state_model.meta_data_was_scaled = meta_dict['meta_data_was_scaled']

    # state_model.is_start = copy.deepcopy(meta_dict['is_start'])
    check_state_model_for_is_start_state(state_model)


class CoreObjectIdentifier(object):
    # TODO generalize and include into utils

    type_related_list_name_dict = {InputDataPort.__name__: 'input_data_ports',
                                   OutputDataPort.__name__: 'output_data_ports',
                                   ScopedVariable.__name__: 'scoped_variables',
                                   DataFlow.__name__: 'data_flows',
                                   Outcome.__name__: 'outcomes',
                                   Transition.__name__: 'transitions',
                                   State.__name__: 'states',
                                   DeciderState.__name__: 'states'}

    def __init__(self, core_obj_or_cls):
        if not(type(core_obj_or_cls) in core_object_list or core_obj_or_cls in core_object_list):
            logger.warning("\n{0}\n{1}\n{2}".format(core_obj_or_cls, type(core_obj_or_cls),core_object_list))
        assert type(core_obj_or_cls) in core_object_list or core_obj_or_cls in core_object_list
        self._sm_id = None
        self._path = None
        # type can be, object types (of type Transition e.g.) or class
        self._type = None
        self._id = None
        self._list_name = None

        if type(core_obj_or_cls) in core_object_list:
            self._type = type(core_obj_or_cls).__name__
        else:
            self._type = 'class'

        if not self._type == 'class':
            if self._type in ['ExecutionState', 'HierarchyState', 'BarrierConcurrencyState',
                              'PreemptiveConcurrencyState', 'LibraryState', 'DeciderState']:
                self._path = core_obj_or_cls.get_path()
                self._id = core_obj_or_cls.state_id
                self._sm_id = core_obj_or_cls.get_state_machine().state_machine_id
            else:
                if isinstance(core_obj_or_cls.parent, State):
                    self._path = core_obj_or_cls.parent.get_path()
                    if core_obj_or_cls.parent.get_state_machine() is None:
                        logger.warning('state has no state machine -> {0} {1}'.format(core_obj_or_cls.parent.name, core_obj_or_cls.parent.get_path()))
                    else:
                        self._sm_id = core_obj_or_cls.parent.get_state_machine().state_machine_id
                else:
                    logger.warning("identifier of core object {0} without parent is mostly useless".format(self._type))

            if self._type in ['InputDataPort', 'OutputDataPort', 'ScopedVariable']:
                self._id = core_obj_or_cls.data_port_id
                self._list_name = self.type_related_list_name_dict[self._type]
            elif self._type == 'Transition':
                self._id = core_obj_or_cls.transition_id
                self._list_name = self.type_related_list_name_dict[self._type]
            elif self._type == 'DataFlow':
                self._id = core_obj_or_cls.data_flow_id
                self._list_name = self.type_related_list_name_dict[self._type]
            elif self._type == 'Outcome':
                self._id = core_obj_or_cls.outcome_id
                self._list_name = self.type_related_list_name_dict[self._type]
            elif self._type == 'StateMachine':
                # self.__sm_id = core_obj_cls.state_machine_id
                pass
            elif self._type == 'GlobalVariableManager':
                pass
            elif self._type == 'LibraryManager':
                pass
            else:
                pass

    def __str__(self):
        return "{0}:{1}:{2}:{3}".format(self._type, self._sm_id, self._path, self._id)


class AbstractAction(object):
    __version_id = None

    def __init__(self, parent_path, state_machine_model, overview=None):
        self.action_type = None
        self.state_machine_model = state_machine_model
        self.parent_path = parent_path

        self.before_overview = NotificationOverview() if overview is None else overview
        self.before_storage = self.get_storage()  # tuple of state and states-list of storage tuple

        self.after_overview = None
        self.after_storage = None  # tuple of state and states-list of storage tuple

    def prepare_destruction(self):
        self.before_overview.prepare_destruction()
        if self. after_overview:
            self.after_overview.prepare_destruction()
        self.state_machine_model = None

    @property
    def version_id(self):
        return self.__version_id

    @version_id.setter
    def version_id(self, value):
        if self.__version_id is None:
            self.__version_id = value
        else:
            logger.warning("The version_id of an action is not allowed to be modify after first assignment")

    def description(self):
        return "{0} {1} {2}".format(self.action_type, self.parent_path, self.__class__.__name__)

    def as_dict(self):
        return {"parent_path": self.parent_path,
                "before_overview": self.before_overview, "after_overview": self.after_overview,
                "before_storage": self.before_storage, "after_storage": self.after_storage}

    def set_after(self, overview):
        self.after_overview = overview
        self.after_storage = self.get_storage()

    def get_storage(self):
        pass

    def undo(self):
        pass

    def redo(self):
        pass


class ActionDummy(AbstractAction):
    def __init__(self, parent_path=None, state_machine_model=None, overview=None):
        AbstractAction.__init__(self, parent_path, state_machine_model, overview)
        self.action_type = 'do_nothing'


class MetaAction(AbstractAction):

    def __init__(self, parent_path, state_machine_model, overview):

        assert isinstance(overview, NotificationOverview)
        assert overview['type'] == 'signal'
        AbstractAction.__init__(self, parent_path, state_machine_model, overview)
        self.action_type = "change " + overview['signal'][-1]['change']

        overview['method_name'].append("change " + overview['signal'][-1]['change'])
        overview['info'][-1]['method_name'] = "change " + overview['signal'][-1]['change']
        overview['instance'].append(overview['model'][-1])
        overview['info'][-1]['instance'] = overview['model'][-1]

        meta_str = json.dumps(overview['model'][-1].meta, cls=JSONObjectEncoder,
                              indent=4, check_circular=False, sort_keys=True)
        # print(meta_str)
        self.meta = json.loads(meta_str, cls=JSONObjectDecoder, substitute_modules=substitute_modules)

    def get_storage(self):
        state_model = self.state_machine_model.get_state_model_by_path(self.parent_path)
        return get_state_element_meta(state_model)

    def get_state_model_changed(self):
        return self.state_machine_model.get_state_model_by_path(self.parent_path)

    def undo(self):
        # TODO check why levels are not working
        # TODO in future emit signal only for respective model
        state_m = self.get_state_model_changed()
        # logger.info("META-Action undo {}".format(state_m.state.get_path()))
        if self.before_overview['signal'][-1]['affects_children']:
            insert_state_meta_data(meta_dict=self.before_storage, state_model=state_m)
            state_m.meta_signal.emit(MetaSignalMsg("undo_meta_action", "all", True))
            # if state_m.state.is_root_state:
            #     self.state_machine_model.state_meta_signal.emit(MetaSignalMsg("undo_meta_action", "all", False))
        else:
            insert_state_meta_data(meta_dict=self.before_storage, state_model=state_m)
            # if state_m.state.is_root_state:
            #     self.state_machine_model.state_meta_signal.emit(MetaSignalMsg("undo_meta_action", "all", False))
            state_m.meta_signal.emit(MetaSignalMsg("undo_meta_action", "all", False))

    def redo(self):
        # TODO check why levels are not working
        # TODO in future emit signal only for respective model
        state_m = self.get_state_model_changed()
        # logger.info("META-Action undo {}".format(state_m.state.get_path()))
        if self.before_overview['signal'][-1]['affects_children']:
            insert_state_meta_data(meta_dict=self.after_storage, state_model=state_m)
            state_m.meta_signal.emit(MetaSignalMsg("redo_meta_action", "all", True))
            # if state_m.state.is_root_state:
            #     self.state_machine_model.state_meta_signal.emit(MetaSignalMsg("redo_meta_action", "all", False))
        else:
            insert_state_meta_data(meta_dict=self.after_storage, state_model=state_m)
            # if state_m.state.is_root_state:
            #     self.state_machine_model.state_meta_signal.emit(MetaSignalMsg("redo_meta_action", "all", False))
            state_m.meta_signal.emit(MetaSignalMsg("redo_meta_action", "all", False))


class Action(ModelMT, AbstractAction):

    def __init__(self, parent_path, state_machine_model, overview):
        ModelMT.__init__(self)
        assert isinstance(overview, NotificationOverview)
        AbstractAction.__init__(self, parent_path, state_machine_model, overview)
        self.state_machine = state_machine_model.state_machine
        self.action_type = overview['method_name'][-1]
        self.before_info = overview['info'][-1]

    def get_storage(self):
        state_tuple = get_state_tuple(self.state_machine_model.state_machine.get_state_by_path(self.parent_path))
        state_model = self.state_machine_model.get_state_model_by_path(self.parent_path)
        state_tuple[STATE_TUPLE_META_DICT_INDEX].update(get_state_element_meta(state_model))
        return state_tuple

    def get_state_changed(self):
        if not self.state_machine.get_state_by_path(self.parent_path) or \
                not self.state_machine.get_state_by_path(self.parent_path).parent:
            # if self.state_machine.get_state_by_path(self.parent_path).parent is None:
            #     logger.info("state is root_state -> take root_state for undo")
            # else:
            if self.state_machine.get_state_by_path(self.parent_path).parent is not None:
                logger.warning("State machine could not get state by path -> take root_state for undo")
            state = self.state_machine.root_state
        else:
            state = self.state_machine.get_state_by_path(self.parent_path)

        return state

    def redo(self):
        """ General Redo, that takes all elements in the parent path state stored of the before action state machine
        status.
        :return:
        """
        self.set_state_to_version(self.get_state_changed(), self.after_storage)

    def undo(self):
        """ General Undo, that takes all elements in the parent path state stored of the after action state machine
        status.
        :return:
        """
        self.set_state_to_version(self.get_state_changed(), self.before_storage)

    def emit_undo_redo_signal(self, action_parent_m, affected_models, after):
        msg = ActionSignalMsg(action='undo/redo', origin='model', action_parent_m=action_parent_m,
                              affected_models=affected_models, after=after, kwargs={})
        # logger.info("{1} emit history signal: {0}".format(msg, self.__class__.__name__))
        action_parent_m.action_signal.emit(msg)

    def compare_models(self, previous_model, actual_model):
        if previous_model is not actual_model:
            logger.warning("The model of the state changes is performed on should not be another one, afterwards. "
                           "\n{0}\n{1}".format(previous_model, actual_model))

    def set_state_to_version(self, state, storage_version):
        # print(state.get_path(), '\n', storage_version[STATE_TUPLE_PATH_INDEX])
        assert state.get_path() == storage_version[STATE_TUPLE_PATH_INDEX]
        # print(self.parent_path, self.parent_path.split('/'), len(self.parent_path.split('/')))
        path_of_state = state.get_path()
        storage_version_of_state = get_state_from_state_tuple(storage_version)

        assert storage_version_of_state

        previous_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        # TODO affected models should be more to allow recursive notification scheme and less updated elements
        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=[previous_model, ], after=False)

        # if self.action_type == 'change_state_type':
        #     self.storage_version_for_state_type_change_signal_hook = storage_version
        #     assert isinstance(self.state_machine_model.root_state.state, State)
        #     state_parent = self.before_overview["instance"][-1]
        #     old_state_changed_ref = self.before_overview["args"][-1][1]
        #     state = self.state_machine.get_state_by_path(old_state_changed_ref.get_path())
        #     old_state_changed_in_storage = storage_version_of_state.states[state.state_id]
        #     if isinstance(old_state_changed_in_storage, (HierarchyState,
        #                                                  BarrierConcurrencyState,
        #                                                  PreemptiveConcurrencyState)):
        #         new_state_class = old_state_changed_in_storage.__class__
        #     else:
        #         logger.info("SM set_root_state_to_version: with NO type change")
        #         new_state_class = ExecutionState
        #
        #     old_root_state_m = self.state_machine_model.root_state
        #     self.observe_model(self.state_machine_model.root_state)
        #
        #     state_parent.change_state_type(state, new_state_class)
        #     self.storage_version_for_state_type_change_signal_hook = None
        #     actual_state_model = self.state_machine_model.get_state_model_by_path(old_state_changed_ref.get_path())
        #     self.relieve_model(old_root_state_m)
        # else:

        self.update_state(state, storage_version_of_state)

        # logger.debug("\n\n\n\n\n\n\nINSERT STATE META: %s %s || Action\n\n\n\n\n\n\n" % (path_of_state, state))
        actual_state_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        self.compare_models(previous_model, actual_state_model)
        insert_state_meta_data(meta_dict=storage_version[STATE_TUPLE_META_DICT_INDEX], state_model=actual_state_model)

        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=[previous_model, ], after=True)

    @ModelMT.observe("action_signal", signal=True)
    def action_signal(self, model, prop_name, info):
        msg = info['arg']
        if msg.action == 'change_state_type' and msg.after:
            new_state_m = msg.affected_models[-1]
            logger.info("action state-type-change action-signal hook for root {}".format(new_state_m))
            storage_version = self.storage_version_for_state_type_change_signal_hook
            root_state_version_from_storage = get_state_from_state_tuple(storage_version)

            self.update_state(new_state_m.state, root_state_version_from_storage)

            insert_state_meta_data(meta_dict=storage_version[STATE_TUPLE_META_DICT_INDEX], state_model=new_state_m)

    def update_state(self, state, stored_state):
        # logger.info("PPP\n{0}\n{1}".format(state, stored_state))
        assert type(stored_state) is type(state)

        is_root = state.is_root_state

        if isinstance(state, ContainerState):

            # print(state.data_flows.keys())
            for data_flow_id in list(state.data_flows.keys()):
                state.remove_data_flow(data_flow_id)

            if not isinstance(state, BarrierConcurrencyState):
                for t_id in list(state.transitions.keys()):
                    # if not UNIQUE_DECIDER_STATE_ID in [state.transitions[t_id].from_state, state.transitions[t_id].to_state]: # funst nicht
                    state.remove_transition(t_id)

            for old_state_id in list(state.states.keys()):
                # try:
                state.remove_state(old_state_id, force=True)
                # except Exception as e:
                #     print("ERROR: ", old_state_id, UNIQUE_DECIDER_STATE_ID, state)
                #     raise

        if is_root:
            for outcome_id in list(state.outcomes.keys()):
                if not outcome_id < 0:
                    state.remove_outcome(outcome_id)

            for dp_id in list(state.input_data_ports.keys()):
                state.remove_input_data_port(dp_id)

            # print(" \n\n\n ########### start removing output data_ports ", state.output_data_ports.keys(), "\n\n\n")
            for dp_id in list(state.output_data_ports.keys()):
                state.remove_output_data_port(dp_id)

        if isinstance(state, ContainerState):
            for dp_id in list(state.scoped_variables.keys()):
                # print("scoped_variable ", dp_id)
                state.remove_scoped_variable(dp_id)

        state.name = stored_state.name
        state.semantic_data = stored_state.semantic_data
        # state.script = stored_state.script
        # logger.debug("script0: " + stored_state.script.script)
        if isinstance(state, ExecutionState):
            state.script_text = stored_state.script_text

        if is_root:
            for dp_id, dp in stored_state.input_data_ports.items():
                # print("generate input data port", dp_id)
                state.add_input_data_port(dp.name, dp.data_type, dp.default_value, dp.data_port_id)
                # print("got input data ports", dp_id, state.input_data_ports.keys())
                assert dp_id in state.input_data_ports

            # print(" \n\n\n ########### start adding output data_ports ", state.output_data_ports.keys(), "\n\n\n")
            for dp_id, dp in stored_state.output_data_ports.items():
                scoped_str = str([])
                if isinstance(state, ContainerState):
                    scoped_str = str(list(state.scoped_variables.keys()))
                # print("\n\n\n ------- ############ generate output data port", dp_id, state.input_data_ports.keys(), \)
                #     state.output_data_ports.keys(), scoped_str, "\n\n\n"
                state.add_output_data_port(dp.name, dp.data_type, dp.default_value, dp.data_port_id)
                # print("\n\n\n ------- ############ got output data ports", dp_id, state.output_data_ports.keys(), "\n\n\n")
                assert dp_id in state.output_data_ports

            for oc_id, oc in stored_state.outcomes.items():
                # print(oc_id, state.outcomes, type(oc_id), oc_id < 0, oc_id == 0, oc_id == -1, oc_id == -2)
                if not oc_id < 0:
                    # print("add_outcome", oc_id)
                    state.add_outcome(oc.name, oc_id)
            # print("\n\n\n++++++++++++++++ ", stored_state.outcomes, state.outcomes, "\n\n\n++++++++++++++++ ")
            for oc_id, oc in stored_state.outcomes.items():
                # print(oc_id, state.outcomes)
                assert oc_id in state.outcomes

        if isinstance(state, ContainerState):
            # logger.debug("UPDATE STATES")
            for dp_id, sv in stored_state.scoped_variables.items():
                state.add_scoped_variable(sv.name, sv.data_type, sv.default_value, sv.data_port_id)

            if UNIQUE_DECIDER_STATE_ID in stored_state.states:
                state.add_state(stored_state.states[UNIQUE_DECIDER_STATE_ID], storage_load=True)

            for new_state in stored_state.states.values():
                # print("++++ new child", new_state)
                if not new_state.state_id == UNIQUE_DECIDER_STATE_ID:
                    state.add_state(new_state)
                    # state.states[new_state.state_id].script = new_state.script
                    # logger.debug("script1: " + new_state.script_text)
                    if isinstance(new_state, ExecutionState):
                        state.states[new_state.state_id].script_text = new_state.script_text

            if isinstance(state, BarrierConcurrencyState):
                for t_id in list(state.transitions.keys()):
                    state.remove_transition(t_id)

            for t_id, t in stored_state.transitions.items():
                state.add_transition(t.from_state, t.from_outcome, t.to_state, t.to_outcome, t.transition_id)

            # logger.debug("CHECK self TRANSITIONS of unique state%s" % state.transitions.keys())
            for t in list(state.transitions.values()):
                # logger.debug(str([t.from_state, t.from_outcome, t.to_state, t.to_outcome]))
                if UNIQUE_DECIDER_STATE_ID == t.from_state and UNIQUE_DECIDER_STATE_ID == t.to_state:
                    # logger.error("found DECIDER_STATE_SELF_TRANSITION")
                    state.remove_transition(t.transition_id)

            for df_id, df in stored_state.data_flows.items():
                state.add_data_flow(df.from_state, df.from_key, df.to_state, df.to_key, df.data_flow_id)

    def add_core_object_to_state(self, state, core_obj):
        # logger.info("RUN ADD CORE OBJECT FOR {0} {1}".format(state.state_id, core_obj))
        if isinstance(core_obj, State):
            state.add_state(core_obj)
        elif isinstance(core_obj, Transition):
            t = core_obj
            state.add_transition(t.from_state, t.from_outcome, t.to_state, t.to_outcome, t.transition_id)
        elif isinstance(core_obj, Outcome):
            state.add_outcome(core_obj.name, core_obj.outcome_id)
        elif isinstance(core_obj, DataFlow):
            df = core_obj
            state.add_data_flow(df.from_state, df.from_key, df.to_state, df.to_key, df.data_flow_id)
        elif isinstance(core_obj, InputDataPort):
            dp = core_obj
            state.add_input_data_port(dp.name, dp.data_type, dp.default_value, dp.data_port_id)
        elif isinstance(core_obj, OutputDataPort):
            dp = core_obj
            state.add_output_data_port(dp.name, dp.data_type, dp.default_value, dp.data_port_id)
        elif isinstance(core_obj, ScopedVariable):
            sv = core_obj
            state.add_scoped_variable(sv.name, sv.data_type, sv.default_value, sv.data_port_id)
        else:
            logger.warning("Type: {0} is no valid core object that can be added.".format(type(core_obj)))

    @staticmethod
    def remove_core_object_from_state(state, core_obj):
        if isinstance(core_obj, State):
            state.remove_state(core_obj.state_id, force=True)
        elif isinstance(core_obj, Transition):
            state.remove_transition(core_obj.transition_id)
        elif isinstance(core_obj, Income):
            state.remove_income()
        elif isinstance(core_obj, Outcome):
            state.remove_outcome(core_obj.outcome_id)
        elif isinstance(core_obj, DataFlow):
            state.remove_data_flow(core_obj.data_flow_id)
        elif isinstance(core_obj, InputDataPort):
            state.remove_input_data_port(core_obj.data_port_id)
        elif isinstance(core_obj, OutputDataPort):
            state.remove_output_data_port(core_obj.data_port_id)
        elif isinstance(core_obj, ScopedVariable):
            state.remove_scoped_variable(core_obj.data_port_id)
        else:
            logger.warning("Type: {0} is no valid core object that can be removed.".format(type(core_obj)))


class StateMachineAction(Action, ModelMT):
    """ The state machine action is currently only used for root state type changes and their undo/redo functionality.
    """

    def __init__(self, parent_path, state_machine_model, overview):
        ModelMT.__init__(self)
        assert isinstance(overview['model'][0].state_machine, StateMachine)
        Action.__init__(self, parent_path, state_machine_model, overview)

        self.with_verbose = False
        self.storage_version_for_state_type_change_signal_hook = None

    def set_root_state_to_version(self, state, storage_version):
        # logger.debug("\n\n\n\n\n\n\nINSERT STATE: %s  || %s || StateMachineAction\n\n\n\n\n\n\n" % (state.get_path(), state))
        # self.state_machine.root_state = get_state_from_state_tuple(storage_version)
        root_state_version_from_storage = get_state_from_state_tuple(storage_version)
        # logger.debug("\n\n\n\n\n\n\nINSERT STATE META: %s || %s || %s || StateMachineAction\n\n\n\n\n\n\n" % (state.get_path(), state, root_state_version_fom_storage))
        # actual_state_model = self.state_machine_model.get_state_model_by_path(state.get_path())

        if self.with_verbose:
            logger.verbose("#H# TRY STATE_HELPER storage type {0} current type {1}"
                           "".format(type(root_state_version_from_storage), state.__class__))
            if root_state_version_from_storage.__class__ == state.__class__:
                logger.verbose("SM set_root_state_to_version: with NO type change")

        new_state_class = root_state_version_from_storage.__class__

        logger.debug("DO root version change " + self.action_type)

        previous_model = self.state_machine_model.root_state
        affected_models = [previous_model, ]
        # TODO affected models should be more to allow recursive notification scheme and less updated elements
        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=affected_models, after=False)

        if self.action_type == 'change_root_state_type':
            # observe root state model (type change signal)
            self.storage_version_for_state_type_change_signal_hook = storage_version
            assert isinstance(self.state_machine_model.root_state.state, State)
            old_root_state_m = self.state_machine_model.root_state
            self.observe_model(self.state_machine_model.root_state)

            # self.state_machine.change_root_state_type(new_state_class)
            import rafcon.gui.helpers.state as gui_helper_state
            gui_helper_state.change_state_type(old_root_state_m, new_state_class)
            self.storage_version_for_state_type_change_signal_hook = None
            self.relieve_model(old_root_state_m)
        else:
            raise TypeError("Wrong action type")
        # else:
        #     import rafcon.gui.helpers.state_machine as gui_helper_state_machine
        #     new_state = gui_helper_state_machine.create_new_state_from_state_with_type(state, new_state_class)
        #
        #     self.update_state(new_state, root_state_version_from_storage)
        #
        #     self.state_machine.root_state = new_state  # root_state_version_fom_storage
        #     insert_state_meta_data(meta_dict=storage_version[STATE_TUPLE_META_DICT_INDEX],
        #                            state_model=self.state_machine_model.root_state)

        affected_models.append(self.state_machine_model.root_state)
        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=affected_models, after=True)

    @ModelMT.observe("action_signal", signal=True)
    def action_signal(self, model, prop_name, info):
        # logger.verbose("#H# STATE_MACHINE_REDO_UNDO: " + str(NotificationOverview(info, False, self.__class__.__name__)))
        msg = info['arg']
        if msg.action == 'change_root_state_type' and msg.after:
            new_state_m = msg.affected_models[-1]
            logger.info("action state-type-change action-signal hook for root {}".format(new_state_m))
            storage_version = self.storage_version_for_state_type_change_signal_hook
            root_state_version_from_storage = get_state_from_state_tuple(storage_version)

            self.update_state(new_state_m.state, root_state_version_from_storage)

            insert_state_meta_data(meta_dict=storage_version[STATE_TUPLE_META_DICT_INDEX], state_model=new_state_m)

    def redo(self):
        # logger.verbose("#H# STATE_MACHINE_REDO STARTED")
        state = self.state_machine.root_state

        self.set_root_state_to_version(state, self.after_storage)
        # logger.verbose("#H# STATE_MACHINE_REDO FINISHED")

    def undo(self):
        """ General Undo, that takes all elements in the parent and
        :return:
        """
        # logger.verbose("#H# STATE_MACHINE_UNDO STARTED")
        state = self.state_machine.root_state

        self.set_root_state_to_version(state, self.before_storage)
        # logger.verbose("#H# STATE_MACHINE_UNDO FINISHED")


class AddObjectAction(Action):
    """ The class handles all adding object action of 7 valid kinds
    (of In-OutputDataPort, ScopedVariable, DataFlow, Outcome, Transition and State)
    """
    possible_method_names = ['add_state', 'add_outcome', 'add_input_data_port', 'add_output_data_port',
                             'add_transition', 'add_data_flow', 'add_scoped_variable']

    def __init__(self, parent_path, state_machine_model, overview):
        Action.__init__(self, parent_path, state_machine_model, overview)
        # logger.info("create AddObject Action for: {0} for prop_name: {1}".format(self.before_info['method_name'], self.before_info['prop_name']))

        assert overview['method_name'][-1] in self.possible_method_names
        assert overview['prop_name'][-1] == 'state' and isinstance(overview['instance'][-1], State)

        self.changed_object = getattr(self.before_info['model'], self.before_info['prop_name'])
        assert self.changed_object is overview['instance'][-1]
        # logger.info("self.changed_object is {0}".format(self.changed_object))

        self.parent_identifier = ''
        self.added_object_identifier = ''
        self.added_object_args = ''

        self.parent_identifier = self.parent_path

    def set_after(self, overview):
        Action.set_after(self, overview)
        # logger.info("add_object \n" + str(self.after_info))
        # get new object from respective list and create identifier
        list_name = overview['method_name'][-1].replace('add_', '') + 's'
        new_object = getattr(overview['args'][-1][0], list_name)[overview['result'][-1]]
        self.added_object_identifier = CoreObjectIdentifier(new_object)

    def redo(self):
        """
        :return: Redo of adding object action is simply done by adding the object again from the after_storage of the
                 parent state.
        """
        # logger.info("RUN REDO AddObject " + self.before_info['method_name'])

        state = self.get_state_changed()
        storage_version = self.after_storage

        assert state.get_path() == storage_version[STATE_TUPLE_PATH_INDEX]
        path_of_state = state.get_path()
        storage_version_of_state = get_state_from_state_tuple(storage_version)

        previous_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=[previous_model, ], after=False)

        if self.added_object_identifier._type in ['InputDataPort', 'OutputDataPort', 'Outcome']:
            [state, storage_version_of_state] = self.correct_reference_state(state,
                                                                             storage_version_of_state,
                                                                             storage_path=storage_version[STATE_TUPLE_PATH_INDEX])
        list_name = self.action_type.replace('add_', '') + 's'
        core_obj = getattr(storage_version_of_state, list_name)[self.added_object_identifier._id]
        self.add_core_object_to_state(state, core_obj)

        actual_state_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        self.compare_models(previous_model, actual_state_model)
        insert_state_meta_data(meta_dict=storage_version[STATE_TUPLE_META_DICT_INDEX],
                               state_model=actual_state_model, level=None if self.action_type == 'add_state' else 1)
        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=[previous_model, ], after=True)

    def undo(self):

        # find object
        state = self.get_state_changed()
        storage_version = self.after_storage

        assert state.get_path() == storage_version[STATE_TUPLE_PATH_INDEX]
        path_of_state = state.get_path()
        storage_version_of_state = get_state_from_state_tuple(storage_version)

        previous_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=[previous_model, ], after=False)

        if self.added_object_identifier._type in ['InputDataPort', 'OutputDataPort', 'Outcome']:
            [state, storage_version_of_state] = self.correct_reference_state(state,
                                                                             storage_version_of_state,
                                                                             storage_path=storage_version[STATE_TUPLE_PATH_INDEX])

        list_name = self.action_type.replace('add_', '') + 's'
        core_obj = getattr(storage_version_of_state, list_name)[self.added_object_identifier._id]
        # logger.info(str(type(core_obj)) + str(core_obj))
        # undo
        self.remove_core_object_from_state(state, core_obj)

        # logger.debug("\n\n\n\n\n\n\nINSERT STATE META: %s %s || Action\n\n\n\n\n\n\n" % (path_of_state, state))
        actual_state_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        self.compare_models(previous_model, actual_state_model)
        insert_state_meta_data(meta_dict=storage_version[STATE_TUPLE_META_DICT_INDEX],
                               state_model=actual_state_model, level=1)
        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=[previous_model, ], after=True)

    def correct_reference_state(self, state, storage_version_of_state, storage_path):

        partial_path = self.added_object_identifier._path.split('/')
        for path_element in storage_path.split('/'):
            partial_path.pop(0)
        for path_element in partial_path:
            storage_version_of_state = storage_version_of_state.states[path_element]
            state = state.states[path_element]
            # logger.info("state is now: {0} {1}".format(state.state_id, storage_version_of_state.state_id))

        return state, storage_version_of_state


class RemoveObjectAction(Action):
    possible_method_names = ['remove_state', 'remove_outcome', 'remove_input_data_port', 'remove_output_data_port',
                             'remove_transition', 'remove_data_flow', 'remove_scoped_variable']

    def __init__(self, parent_path, state_machine_model, overview):
        Action.__init__(self, parent_path, state_machine_model, overview)
        # logger.info("create RemoveObject Action for: {0} for prop_name: {1}".format(self.before_info['method_name'], self.before_info['prop_name']))

        assert overview['method_name'][-1] in self.possible_method_names
        assert overview['prop_name'][-1] == 'state' and isinstance(overview['instance'][-1], State)

        self.instance_path = overview['instance'][-1].get_path()
        self.changed_object = getattr(self.before_info['model'], self.before_info['prop_name'])
        # logger.info("self.changed_object is {0}".format(self.changed_object))

        self.parent_identifier = ''
        self.removed_object_identifier = ''
        self.removed_object_args = ''
        if "outcome" in self.before_info['method_name'] or "data_port" in self.before_info['method_name']:
            pass
        else:
            self.parent_identifier = self.parent_path
        self.get_object_identifier()
        self.before_linkage = {'internal': {'transitions': [], 'data_flows': []},
                               'external': {'transitions': [], 'data_flows': []}}
        self.after_linkage = {'internal': {'transitions': [], 'data_flows': []},
                              'external': {'transitions': [], 'data_flows': []}}
        self.removed_linkage = {'internal': {'transitions': [], 'data_flows': []},
                                'external': {'transitions': [], 'data_flows': []}}
        self.added_linkage = {'internal': {'transitions': [], 'data_flows': []},
                              'external': {'transitions': [], 'data_flows': []}}
        self.store_related_elements(self.before_linkage)

    def as_dict(self):
        d = Action.as_dict(self)
        d.update({"before_linkage": self.before_linkage,
                  "after_linkage": self.after_linkage,
                  "removed_linkage": self.removed_linkage,
                  "added_linkage": self.added_linkage})
        return d

    def set_after(self, overview):
        Action.set_after(self, overview)
        self.store_related_elements(self.after_linkage)
        self.diff_related_elements()

    def get_object_identifier(self):
        # logger.info("remove_object \n" + str(self.before_info))
        overview = self.before_overview
        # get new object from respective list and create identifier
        object_type_name = overview['method_name'][-1].replace('remove_', '')
        list_name = object_type_name + 's'
        if object_type_name + '_id' in overview['kwargs'][-1]:
            object_id = overview['kwargs'][-1][object_type_name + '_id']
        else:
            if len(overview['args'][-1]) < 2:
                logger.error("Length of args-tuple is shorter as assumed.")
            else:
                object_id = overview['args'][-1][1]
        new_object = getattr(overview['args'][-1][0], list_name)[object_id]
        self.removed_object_identifier = CoreObjectIdentifier(new_object)
        # logger.info("removed_object with identifier {0}".format(self.removed_object_identifier))

    def undo(self):
        # logger.info("RUN UNDO RemoveObject " + self.before_info['method_name'])

        state = self.get_state_changed()
        storage_version = self.before_storage

        assert state.get_path() == storage_version[STATE_TUPLE_PATH_INDEX]
        path_of_state = state.get_path()
        storage_version_of_state = get_state_from_state_tuple(storage_version)

        previous_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=[previous_model, ], after=False)

        if self.removed_object_identifier._type in ['InputDataPort', 'OutputDataPort', 'Outcome']:
            [state, storage_version_of_state] = self.correct_reference_state(state,
                                                                             storage_version_of_state,
                                                                             storage_path=storage_version[STATE_TUPLE_PATH_INDEX])
        list_name = self.action_type.replace('remove_', '') + 's'
        core_obj = getattr(storage_version_of_state, list_name)[self.removed_object_identifier._id]
        # logger.info(str(type(core_obj)) + str(core_obj))
        if self.action_type not in ['remove_transition', 'remove_data_flow']:
            self.add_core_object_to_state(state, core_obj)

        self.adjust_linkage()

        # logger.debug("\n\n\n\n\n\n\nINSERT STATE META: %s %s || Action\n\n\n\n\n\n\n" % (path_of_state, state))
        actual_state_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        self.compare_models(previous_model, actual_state_model)
        insert_state_meta_data(meta_dict=storage_version[STATE_TUPLE_META_DICT_INDEX],
                               state_model=actual_state_model, level=None if self.action_type == 'remove_state' else 1)

        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=[previous_model, ], after=True)

    def redo(self):

        # prepare what state and method is needed
        if self.action_type == 'remove_state':
            path_of_state = self.parent_path
            method_name = 'remove_state'
        else:
            path_of_state = self.removed_object_identifier._path
            method_name = 'remove_' + self.removed_object_identifier._list_name[:-1]
        parent_with_all_changes = self.get_state_changed()
        path_of_parent_with_all_changes = parent_with_all_changes.get_path()

        # signal that undo/redo action will be performed -> stop graphical editor
        previous_model = self.state_machine_model.get_state_model_by_path(path_of_parent_with_all_changes)
        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=[previous_model, ], after=False)

        # remove element from core
        state = self.state_machine.get_state_by_path(path_of_state)
        remove_function = getattr(state, method_name)
        remove_function(self.removed_object_identifier._id)

        assert self.get_state_changed() is parent_with_all_changes
        actual_state_model = self.state_machine_model.get_state_model_by_path(path_of_parent_with_all_changes)
        self.compare_models(previous_model, actual_state_model)
        insert_state_meta_data(meta_dict=self.after_storage[STATE_TUPLE_META_DICT_INDEX],
                               state_model=actual_state_model, level=1)

        # signal that undo/redo action was performed -> run graphical editor update
        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=[previous_model, ], after=True)

    def correct_reference_state(self, state, storage_version_of_state, storage_path):

        partial_path = self.removed_object_identifier._path.split('/')
        for path_element in storage_path.split('/'):
            logger.debug("pop: " + partial_path.pop(0))
        for path_element in partial_path:
            storage_version_of_state = storage_version_of_state.states[path_element]
            state = state.states[path_element]
            logger.debug("state is now: {0} {1}".format(state.state_id, storage_version_of_state.state_id))

        return state, storage_version_of_state

    def store_related_elements(self, linkage_dict):

        state = self.state_machine.get_state_by_path(self.instance_path)
        if isinstance(state, HierarchyState):
            for t in state.transitions.values():
                t_dict = {'from_state': t.from_state, 'from_outcome': t.from_outcome,
                          'to_state': t.to_state, 'to_outcome': t.to_outcome, 'transition_id': t.transition_id}
                linkage_dict['internal']['transitions'].append(t_dict)

            for df in state.data_flows.values():
                df_dict = {'from_state': df.from_state, 'from_key': df.from_key,
                           'to_state': df.to_state, 'to_key': df.to_key, 'data_flow_id': df.data_flow_id}
                linkage_dict['internal']['data_flows'].append(df_dict)

        if isinstance(state.parent, State):
            for t in state.parent.transitions.values():
                t_dict = {'from_state': t.from_state, 'from_outcome': t.from_outcome,
                          'to_state': t.to_state, 'to_outcome': t.to_outcome, 'transition_id': t.transition_id}
                linkage_dict['external']['transitions'].append(t_dict)

            for df in state.parent.data_flows.values():
                df_dict = {'from_state': df.from_state, 'from_key': df.from_key,
                           'to_state': df.to_state, 'to_key': df.to_key, 'data_flow_id': df.data_flow_id}
                linkage_dict['external']['data_flows'].append(df_dict)

    def diff_related_elements(self):

        self.removed_linkage['internal']['transitions'] = [kwargs for kwargs in self.before_linkage['internal']['transitions']
                                                           if kwargs not in self.after_linkage['internal']['transitions']]
        self.removed_linkage['internal']['data_flows'] = [kwargs for kwargs in self.before_linkage['internal']['data_flows']
                                                          if kwargs not in self.after_linkage['internal']['data_flows']]
        self.added_linkage['internal']['transitions'] = [kwargs for kwargs in self.after_linkage['internal']['transitions']
                                                         if kwargs not in self.before_linkage['internal']['transitions']]
        self.added_linkage['internal']['data_flows'] = [kwargs for kwargs in self.after_linkage['internal']['data_flows']
                                                        if kwargs not in self.before_linkage['internal']['data_flows']]

        self.removed_linkage['external']['transitions'] = [kwargs for kwargs in self.before_linkage['external']['transitions']
                                                           if kwargs not in self.after_linkage['external']['transitions']]
        self.removed_linkage['external']['data_flows'] = [kwargs for kwargs in self.before_linkage['external']['data_flows']
                                                          if kwargs not in self.after_linkage['external']['data_flows']]
        self.added_linkage['external']['transitions'] = [kwargs for kwargs in self.after_linkage['external']['transitions']
                                                         if kwargs not in self.before_linkage['external']['transitions']]
        self.added_linkage['external']['data_flows'] = [kwargs for kwargs in self.after_linkage['external']['data_flows']
                                                        if kwargs not in self.before_linkage['external']['data_flows']]

    def adjust_linkage(self):
        state = self.state_machine.get_state_by_path(self.instance_path)

        for kwargs in self.removed_linkage['internal']['transitions']:
            state.add_transition(kwargs['from_state'], kwargs['from_outcome'], kwargs['to_state'], kwargs['to_outcome'], kwargs['transition_id'])
        for kwargs in self.removed_linkage['external']['transitions']:
            state.parent.add_transition(kwargs['from_state'], kwargs['from_outcome'], kwargs['to_state'], kwargs['to_outcome'], kwargs['transition_id'])
        for kwargs in self.removed_linkage['internal']['data_flows']:
            state.add_data_flow(kwargs['from_state'], kwargs['from_key'], kwargs['to_state'], kwargs['to_key'], kwargs['data_flow_id'])
        for kwargs in self.removed_linkage['external']['data_flows']:
            state.parent.add_data_flow(kwargs['from_state'], kwargs['from_key'], kwargs['to_state'], kwargs['to_key'], kwargs['data_flow_id'])

        assert self.added_linkage['internal']['transitions'] == []
        assert self.added_linkage['external']['transitions'] == []
        assert self.added_linkage['internal']['data_flows'] == []
        assert self.added_linkage['external']['data_flows'] == []


class StateElementAction(AbstractAction):

    possible_method_names = []
    possible_args = []
    _object_class = None

    def __init__(self, parent_path, state_machine_model, overview):
        AbstractAction.__init__(self, parent_path, state_machine_model, overview)

        # validate class type
        assert isinstance(self.before_overview['instance'][-1], self._object_class)

        # validate method call -- action type
        self.action_type = overview['method_name'][-1]
        if not self.action_type in self.possible_method_names:
            logger.error("{0} is not possible with overview {1}".format(self.__class__.__name__, overview))
        assert self.action_type in self.possible_method_names

        # validate object path
        self.object_identifier = CoreObjectIdentifier(self.before_overview['instance'][-1])
        assert self.parent_path == self.object_identifier._path

        self.before_arguments = self.get_set_of_arguments(self.before_overview['instance'][-1])
        self.after_arguments = None

        self.state_machine = state_machine_model.state_machine

    def as_dict(self):
        d = AbstractAction.as_dict(self)
        d.update({"before_arguments": self.before_arguments,
                  "after_arguments": self.after_arguments})
        return d

    @staticmethod
    def get_set_of_arguments(elem):
        raise NotImplementedError()

    def set_after(self, overview):
        self.after_overview = overview
        assert isinstance(self.after_overview['instance'][-1], self._object_class)
        self.after_arguments = self.get_set_of_arguments(self.after_overview['instance'][-1])


class DataFlowAction(StateElementAction):

    possible_method_names = ['modify_origin', 'from_state', 'from_key',
                             'modify_target', 'to_state', 'to_key']
    possible_args = ['from_state', 'from_key', 'to_state', 'to_key']
    _object_class = DataFlow

    def __init__(self, parent_path, state_machine_model, overview):
        StateElementAction.__init__(self, parent_path, state_machine_model, overview)

    @staticmethod
    def get_set_of_arguments(df):
        return {'from_state': df.from_state, 'from_key': df.from_key, 'to_state': df.to_state, 'to_key': df.to_key,
                'data_flow_id': df.data_flow_id}

    def undo(self):
        # if the data_flow_id would be changed and this considered in the core parent element self.after_argument here would be used
        df = self.state_machine.get_state_by_path(self.parent_path).data_flows[self.before_arguments['data_flow_id']]
        self.set_data_flow_version(df, self.before_arguments)

    def redo(self):
        df = self.state_machine.get_state_by_path(self.parent_path).data_flows[self.before_arguments['data_flow_id']]
        self.set_data_flow_version(df, self.after_arguments)

    def set_data_flow_version(self, df, arguments):
        if self.action_type in self.possible_args:
            exec("df.{0} = arguments['{0}']".format(self.action_type))
        elif self.action_type == 'modify_origin':
            df.modify_origin(from_state=arguments['from_state'], from_key=arguments['from_key'])
        elif self.action_type == 'modify_target':
            df.modify_target(to_state=arguments['to_state'], to_key=arguments['to_key'])
        else:
            raise TypeError("Only types of the following list are allowed. {0}".format(self.possible_method_names))


class TransitionAction(StateElementAction):

    possible_method_names = ['modify_origin', 'from_state', 'from_outcome',
                             'modify_target', 'to_state', 'to_outcome']
    possible_args = ['from_state', 'from_outcome', 'to_state', 'to_key']
    _object_class = Transition

    def __init__(self, parent_path, state_machine_model, overview):
        StateElementAction.__init__(self, parent_path, state_machine_model, overview)

    @staticmethod
    def get_set_of_arguments(t):
        return {'from_state': t.from_state, 'from_outcome': t.from_outcome, 'to_state': t.to_state, 'to_outcome': t.to_outcome,
                'transition_id': t.transition_id}

    def undo(self):
        # if the transition_id would be changed and this considered in the core parent element self.after_argument here would be used
        t = self.state_machine.get_state_by_path(self.parent_path).transitions[self.before_arguments['transition_id']]
        self.set_transition_version(t, self.before_arguments)

    def redo(self):
        t = self.state_machine.get_state_by_path(self.parent_path).transitions[self.before_arguments['transition_id']]
        self.set_transition_version(t, self.after_arguments)

    def set_transition_version(self, t, arguments):
        if self.action_type in self.possible_args:
            exec("t.{0} = arguments['{0}']".format(self.action_type))
        elif self.action_type == 'modify_origin':
            t.modify_origin(from_state=arguments['from_state'], from_outcome=arguments['from_outcome'])
        elif self.action_type == 'modify_target':
            t.modify_target(to_state=arguments['to_state'], to_outcome=arguments['to_outcome'])
        else:
            raise TypeError("Only types of the following list are allowed. {0}".format(self.possible_method_names))


class DataPortAction(StateElementAction):

    possible_method_names = ['name', 'data_type', 'default_value', 'change_data_type']
    possible_args = ['name', 'default_value']
    _object_class = DataPort

    def __init__(self, parent_path, state_machine_model, overview):
        StateElementAction.__init__(self, parent_path, state_machine_model, overview)

    @staticmethod
    def get_set_of_arguments(dp):
        return {'name': dp.name, 'data_type': dp.data_type, 'default_value': dp.default_value,
                'data_port_id': dp.data_port_id}

    def undo(self):
        dp = self.state_machine.get_state_by_path(self.parent_path).get_data_port_by_id(self.before_arguments['data_port_id'])
        self.set_data_port_version(dp, self.before_arguments)

    def redo(self):
        dp = self.state_machine.get_state_by_path(self.parent_path).get_data_port_by_id(self.before_arguments['data_port_id'])
        self.set_data_port_version(dp, self.after_arguments)

    def set_data_port_version(self, dp, arguments):
        if self.action_type in self.possible_args:
            exec("dp.{0} = arguments['{0}']".format(self.action_type))
        elif self.action_type == 'data_type':
            dp.data_type = arguments['data_type']
            dp.default_value = arguments['default_value']
        elif self.action_type == 'change_data_type':
            dp.change_data_type(data_type=arguments['data_type'], default_value=arguments['default_value'])
        else:
            raise TypeError("Only types of the following list are allowed. {0}".format(self.possible_method_names))


class ScopedVariableAction(DataPortAction):

    def __init__(self, parent_path, state_machine_model, overview):
        DataPortAction.__init__(self, parent_path, state_machine_model, overview)


class OutcomeAction(StateElementAction):

    possible_method_names = ['name']
    possible_args = ['name']
    _object_class = Outcome

    def __init__(self, parent_path, state_machine_model, overview):
        StateElementAction.__init__(self, parent_path, state_machine_model, overview)

    @staticmethod
    def get_set_of_arguments(oc):
        return {'name': oc.name, 'outcome_id': oc.outcome_id}

    def undo(self):
        # if the outcome_id would be changed and this considered in the core parent element self.after_argument here would be used
        oc = self.state_machine.get_state_by_path(self.parent_path).outcomes[self.before_arguments['outcome_id']]
        self.set_outcome_version(oc, self.before_arguments)

    def redo(self):
        oc = self.state_machine.get_state_by_path(self.parent_path).outcomes[self.before_arguments['outcome_id']]
        self.set_outcome_version(oc, self.after_arguments)

    def set_outcome_version(self, oc, arguments):
        if self.action_type in self.possible_args:
            exec("oc.{0} = arguments['{0}']".format(self.action_type))
        else:
            raise TypeError("Only types of the following list are allowed. {0}".format(self.possible_method_names))


class StateAction(Action):

    not_possible_method_names = ['input_data', 'output_data', 'concurrency_queue', 'state_id',  # any not observed
                                 'final_outcome', 'preempted', 'active', 'is_root_state',  # any not observed
                                 'scoped_data'].extend(BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS)
    possible_method_names = ['parent',  # will be ignored
                             'name', 'description', 'script', 'script_text',  # State
                             'outcomes', 'input_data_ports', 'output_data_ports',  # State
                             'states', 'scoped_variables', 'data_flows', 'transitions', 'start_state_id',  # ContainerState
                             'change_state_type',
                             'add_input_data_port', 'remove_input_data_port',  # LibraryState
                             'add_output_data_port', 'remove_output_data_port',
                             'set_input_runtime_value', 'set_output_runtime_value',
                             'set_use_input_runtime_value', 'set_use_output_runtime_value',
                             'input_data_port_runtime_values', 'output_data_port_runtime_values',
                             'use_runtime_value_input_data_ports', 'use_runtime_value_output_data_ports',
                             'group_states', 'ungroup_state', 'substitute_state', 'paste', 'cut',
                             'semantic_data', 'add_semantic_data', 'remove_semantic_data'
                             ]
    possible_args = ['name', 'description', 'script_text', 'start_state_id',  # ContainerState
                     'library_name', 'library_path', 'version', 'state_copy',  # LibraryState
                     'input_data_port_runtime_values', 'output_data_port_runtime_values',
                     'use_runtime_value_input_data_ports', 'use_runtime_value_output_data_ports',
                     'set_input_runtime_value', 'set_output_runtime_value',
                     'set_use_input_runtime_value', 'set_use_output_runtime_value',
                     'semantic_data'
                     ]
    substitute_dict = {'set_input_runtime_value': 'input_data_port_runtime_values',
                       'set_output_runtime_value': 'output_data_port_runtime_values',
                       'set_use_input_runtime_value': 'use_runtime_value_input_data_ports',
                       'set_use_output_runtime_value': 'use_runtime_value_output_data_ports'}

    def __init__(self, parent_path, state_machine_model, overview):
        """ method_name: 'parent' is ignored
        """
        if overview['method_name'][-1] in ['outcomes', 'input_data_ports', 'output_data_ports']:  # need State's parent
            if isinstance(overview['instance'][-1].parent, State):
                parent_path = overview['instance'][-1].parent.get_path()
        Action.__init__(self, parent_path, state_machine_model, overview)

        self.state_machine = state_machine_model.state_machine
        if self.action_type not in self.possible_method_names:
            logger.error("action_type: '{0}' not in {1}".format(self.action_type, self.possible_method_names))
        assert self.action_type in self.possible_method_names
        assert isinstance(self.before_overview['instance'][-1], State)
        self.object_identifier = CoreObjectIdentifier(self.before_overview['instance'][-1])
        if overview['method_name'][-1] in ['outcomes', 'input_data_ports', 'output_data_ports']:
            assert self.parent_path == CoreObjectIdentifier(self.before_overview['instance'][-1].parent)._path
        else:
            assert self.parent_path == self.object_identifier._path
        self.before_arguments = self.get_set_of_arguments(self.before_overview['instance'][-1])
        self.after_arguments = None
        if self.action_type == 'script_text' and isinstance(self.before_overview['args'][-1][1], string_types):
            d = difflib.Differ()
            diff = list(d.compare(self.before_overview['args'][-1][0].script_text.split('\n'),
                                  self.before_overview['args'][-1][1].split('\n')))
            self.script_diff = '\n'.join(diff)
        else:
            self.script_diff = None
        if self.action_type == 'description':
            d = difflib.Differ()
            diff = list(d.compare(self.before_overview['args'][-1][0].description.split('\n') if self.before_overview['args'][-1][0].description else [''] ,
                                  self.before_overview['args'][-1][1].split('\n') if self.before_overview['args'][-1][1] else ['']))
            self.description_diff = '\n'.join(diff)
        else:
            self.description_diff = None
        if 'semantic_data' in overview['method_name'][-1]:
            self.action_type = 'semantic_data'

    def as_dict(self):
        d = Action.as_dict(self)
        d.update({"before_arguments": self.before_arguments, "after_arguments": self.after_arguments})
        return d

    @staticmethod
    def get_set_of_arguments(s):
        if isinstance(s, ContainerState):
            return {'name': s.name, 'description': s.description, 'state_id': s.state_id,
                    'start_state_id': s.start_state_id, 'semantic_data': copy.deepcopy(s.semantic_data)}
        elif isinstance(s, LibraryState):
            return {'name': s.name, 'description': s.description, 'state_id': s.state_id,
                    'semantic_data': copy.deepcopy(s.semantic_data),
                    'library_name': s.library_name, 'library_path': s.library_path, 'version': s.version, # LibraryState
                    'state_copy': s.state_copy,
                    'input_data_port_runtime_values': copy.deepcopy(s.input_data_port_runtime_values),
                    'output_data_port_runtime_values': copy.deepcopy(s.output_data_port_runtime_values),
                    'use_runtime_value_input_data_ports': copy.deepcopy(s.use_runtime_value_input_data_ports),
                    'use_runtime_value_output_data_ports': copy.deepcopy(s.use_runtime_value_output_data_ports)}
        else:
            return {'name': s.name, 'description': s.description, 'script_text': s.script_text,
                    'state_id': s.state_id, 'semantic_data': copy.deepcopy(s.semantic_data)}

    def set_after(self, overview):
        Action.set_after(self, overview)
        self.after_overview = overview
        assert isinstance(self.after_overview['instance'][-1], State)
        self.after_arguments = self.get_set_of_arguments(self.after_overview['instance'][-1])

    def undo(self):
        if self.action_type in ['parent', 'outcomes', 'input_data_ports', 'output_data_ports']:
            Action.undo(self)
        elif self.action_type in ['states', 'scoped_variables', 'data_flows', 'transitions', 'change_state_type',
                                  'group_states', 'ungroup_state', 'substitute_state', 'paste', 'cut']:
            Action.undo(self)
        elif self.action_type in ['add_input_data_port', 'remove_input_data_port',  # LibraryState
                                  'add_output_data_port', 'remove_output_data_port']:
            logger.warning('undoing {0} for a LibraryState is not implemented'.format(self.action_type))
        elif self.action_type in self.possible_args:
            s = self.state_machine.get_state_by_path(self.parent_path)
            self.set_attr_to_version(s, self.before_arguments)
        else:
            assert False

    def redo(self):
        if self.action_type in ['outcomes', 'input_data_ports', 'output_data_ports']:
            Action.redo(self)
        elif self.action_type in ['states', 'scoped_variables', 'data_flows', 'transitions', 'change_state_type',
                                  'group_states', 'ungroup_state', 'substitute_state', 'paste', 'cut']:
            Action.redo(self)
        elif self.action_type in ['add_input_data_port', 'remove_input_data_port',  # LibraryState
                                  'add_output_data_port', 'remove_output_data_port']:
            logger.warning('redoing {0} for a LibraryState is not implemented'.format(self.action_type))
        elif self.action_type in self.possible_args:
            s = self.state_machine.get_state_by_path(self.parent_path)
            self.set_attr_to_version(s, self.after_arguments)
        else:
            assert False

    def set_attr_to_version(self, s, arguments):
        if self.action_type in self.possible_args:
            exec("s.{0} = copy.deepcopy(arguments['{0}'])".format(self.substitute_dict.get(self.action_type,
                                                                                           self.action_type)))
        else:
            assert False


class Group(Action):
    def __init__(self, *args, **kwargs):
        Action.__init__(self, *args, **kwargs)


class UnGroup(Action):
    def __init__(self, *args, **kwargs):
        Action.__init__(self, *args, **kwargs)
