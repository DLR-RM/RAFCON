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
from collections import namedtuple

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

StateImage = namedtuple('StateImage', ['core_data', 'meta_data', 'state_path', 'semantic_data', 'file_system_path', 'script_text', 'children'])
StateImage.__new__.__defaults__ = (None, None, None, None, None, None, None)  # Make all optional

def create_state_image(state_m):
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
    state = state_m.state
    core_data = json.dumps(state, cls=JSONObjectEncoder,
                           indent=4, check_circular=False, sort_keys=True)

    child_state_images = {}
    if isinstance(state, ContainerState):
        for child_state_id, child_state_m in state_m.states.items():
            child_state_images[child_state_id] = create_state_image(child_state_m)

    meta_data = get_state_element_meta(state_m)

    script_content = state.script.script if isinstance(state, ExecutionState) else None

    stage_image = StateImage(core_data=core_data, meta_data=meta_data, state_path=state.get_path(),
                             semantic_data=copy.deepcopy(state.semantic_data), file_system_path=state.file_system_path,
                             script_text=script_content, children=child_state_images)
    return stage_image


def create_state_from_image(state_image):
    # Transitions and data flows are not added, as also states are not added
    # We have to wait until the child states are loaded, before adding transitions and data flows, as otherwise the
    # validity checks for transitions and data flows would fail
    state_info = json.loads(state_image.core_data,
                            cls=JSONObjectDecoder, substitute_modules=substitute_modules)
    if not isinstance(state_info, tuple):
        state = state_info
    else:
        state = state_info[0]
        transitions = state_info[1]
        data_flows = state_info[2]

    state._file_system_path = state_image.file_system_path
    state.semantic_data = state_image.semantic_data

    if isinstance(state, BarrierConcurrencyState):
        child_state = create_state_from_image(state_image.children[UNIQUE_DECIDER_STATE_ID])
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
        try:
            state.script_text = state_image.script_text
        except:
            pass  # Tolerate script compilation errors
    # print("------------- ", state)
    for child_state_id, child_state_tuple in state_image.children.items():
        child_state = create_state_from_image(child_state_tuple)
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

    if with_verbose:
        logger.verbose("STORE META for STATE: {0} {1}".format(state_model.state.state_id, state_model.state.name))
    meta_dict['is_start'] = state_model.is_start

    meta_dict['income'] = meta_dump_or_deepcopy(state_model.income.meta)

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

    state_model.meta = meta_dump_or_deepcopy(meta_dict['state'])
    if with_verbose:
        logger.verbose("INSERT META for STATE: {0} {1}".format(state_model.state.state_id, state_model.state.name))

    if 'income' in meta_dict:
        state_model.income.meta = meta_dump_or_deepcopy(meta_dict['income'])
    else:
        missing_meta_data_log_msg(state_model, state_model.income, meta_dict, 'income', [state_model.income.core_element_id])
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

    _sm_id = None
    _path = None
    # type can be object types (of type Transition e.g.) or class
    _type = None
    _id = None
    _list_name = None

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
            logger.warning("\n{0}\n{1}\n{2}".format(core_obj_or_cls, type(core_obj_or_cls), core_object_list))
        assert type(core_obj_or_cls) in core_object_list or core_obj_or_cls in core_object_list

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
    action_type = None
    after_overview = None
    after_state_image = None

    def __init__(self, parent_path, state_machine_model, overview=None):
        self.parent_path = parent_path
        self.state_machine_model = state_machine_model

        self.before_overview = NotificationOverview() if overview is None else overview
        self.before_state_image = self.get_state_image()  # tuple of state and states-list of storage tuple

    def prepare_destruction(self):
        self.before_overview.prepare_destruction()
        if self. after_overview:
            self.after_overview.prepare_destruction()
        self.state_machine_model = None

    def description(self):
        return "{0} {1} {2}".format(self.action_type, self.parent_path, self.__class__.__name__)

    def as_dict(self):
        return {"parent_path": self.parent_path,
                "before_overview": self.before_overview, "after_overview": self.after_overview,
                "before_state_image": self.before_state_image, "after_state_image": self.after_state_image}

    def set_after(self, overview):
        self.after_overview = overview
        self.after_state_image = self.get_state_image()

    def get_state_image(self):
        pass

    def undo(self):
        pass

    def redo(self):
        pass


class ActionDummy(AbstractAction):
    def __init__(self, parent_path=None, state_machine_model=None, overview=None):
        AbstractAction.__init__(self, parent_path, state_machine_model, overview)
        self.action_type = 'do_nothing'


class MetaDataAction(AbstractAction):

    def __init__(self, parent_path, state_machine_model, overview):

        assert isinstance(overview, NotificationOverview)
        assert overview.type == 'signal'
        AbstractAction.__init__(self, parent_path, state_machine_model, overview)
        self.action_type = "change " + overview.get_signal_message().change

        meta_str = json.dumps(overview.get_affected_model().meta, cls=JSONObjectEncoder,
                              indent=4, check_circular=False, sort_keys=True)
        # print(meta_str)
        self.meta = json.loads(meta_str, cls=JSONObjectDecoder, substitute_modules=substitute_modules)

    def get_state_image(self):
        parent_state_model = self.state_machine_model.get_state_model_by_path(self.parent_path)
        meta_data = get_state_element_meta(parent_state_model)
        state_image = StateImage(meta_data=meta_data)
        return state_image

    def get_state_model_changed(self):
        return self.state_machine_model.get_state_model_by_path(self.parent_path)

    def undo(self):
        # TODO check why levels are not working
        # TODO in future emit signal only for respective model
        state_m = self.get_state_model_changed()
        if self.before_overview.get_signal_message().affects_children:
            insert_state_meta_data(meta_dict=self.before_state_image.meta_data, state_model=state_m)
            state_m.meta_signal.emit(MetaSignalMsg("undo_meta_action", "all", True))
            # if state_m.state.is_root_state:
            #     self.state_machine_model.state_meta_signal.emit(MetaSignalMsg("undo_meta_action", "all", False))
        else:
            insert_state_meta_data(meta_dict=self.before_state_image.meta_data, state_model=state_m)
            # if state_m.state.is_root_state:
            #     self.state_machine_model.state_meta_signal.emit(MetaSignalMsg("undo_meta_action", "all", False))
            state_m.meta_signal.emit(MetaSignalMsg("undo_meta_action", "all", False))

    def redo(self):
        # TODO check why levels are not working
        # TODO in future emit signal only for respective model
        state_m = self.get_state_model_changed()
        if self.before_overview.get_signal_message().affects_children:
            insert_state_meta_data(meta_dict=self.after_state_image.meta_data, state_model=state_m)
            state_m.meta_signal.emit(MetaSignalMsg("redo_meta_action", "all", True))
            # if state_m.state.is_root_state:
            #     self.state_machine_model.state_meta_signal.emit(MetaSignalMsg("redo_meta_action", "all", False))
        else:
            insert_state_meta_data(meta_dict=self.after_state_image.meta_data, state_model=state_m)
            # if state_m.state.is_root_state:
            #     self.state_machine_model.state_meta_signal.emit(MetaSignalMsg("redo_meta_action", "all", False))
            state_m.meta_signal.emit(MetaSignalMsg("redo_meta_action", "all", False))


class Action(ModelMT, AbstractAction):

    def __init__(self, parent_path, state_machine_model, overview):
        ModelMT.__init__(self)
        assert isinstance(overview, NotificationOverview)
        AbstractAction.__init__(self, parent_path, state_machine_model, overview)
        self.state_machine = state_machine_model.state_machine
        self.action_type = overview.get_cause()

    def get_state_image(self):
        parent_state_m = self.state_machine_model.get_state_model_by_path(self.parent_path)
        state_image = create_state_image(parent_state_m)
        return state_image

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
        self.update_state_from_image(self.get_state_changed(), self.after_state_image)

    def undo(self):
        """ General Undo, that takes all elements in the parent path state stored of the after action state machine
        status.
        :return:
        """
        self.update_state_from_image(self.get_state_changed(), self.before_state_image)

    def emit_undo_redo_signal(self, action_parent_m, affected_models, after):
        msg = ActionSignalMsg(action='undo/redo', origin='model', action_parent_m=action_parent_m,
                              affected_models=affected_models, after=after, kwargs={})
        action_parent_m.action_signal.emit(msg)

    def compare_models(self, previous_model, actual_model):
        if previous_model is not actual_model:
            logger.warning("The model of the state changes is performed on should not be another one, afterwards. "
                           "\n{0}\n{1}".format(previous_model, actual_model))

    def update_state_from_image(self, state, state_image):
        assert state.get_path() == state_image.state_path
        # print(self.parent_path, self.parent_path.split('/'), len(self.parent_path.split('/')))
        path_of_state = state.get_path()
        state_from_image = create_state_from_image(state_image)

        previous_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        # TODO affected models should be more to allow recursive notification scheme and less updated elements
        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=[previous_model, ], after=False)

        self.update_state(state, state_from_image)

        actual_state_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        self.compare_models(previous_model, actual_state_model)
        insert_state_meta_data(meta_dict=state_image.meta_data, state_model=actual_state_model)

        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=[previous_model, ], after=True)

    @ModelMT.observe("action_signal", signal=True)
    def action_signal(self, model, prop_name, info):
        msg = info['arg']
        if msg.action == 'change_state_type' and msg.after:
            new_state_m = msg.affected_models[-1]
            logger.info("action state-type-change action-signal hook for root {}".format(new_state_m))
            state_image = self.state_image_for_state_type_change_signal_hook
            root_state_image = create_state_from_image(state_image)

            self.update_state(new_state_m.state, root_state_image)

            insert_state_meta_data(meta_dict=state_image.meta_dict, state_model=new_state_m)

    def update_state(self, state, stored_state):
        assert type(stored_state) is type(state)

        is_root = state.is_root_state

        if isinstance(state, ContainerState):

            for data_flow_id in list(state.data_flows.keys()):
                state.remove_data_flow(data_flow_id)

            if not isinstance(state, BarrierConcurrencyState):
                for t_id in list(state.transitions.keys()):
                    state.remove_transition(t_id)

            for old_state_id in list(state.states.keys()):
                state.remove_state(old_state_id, force=True)

        if is_root:
            for outcome_id in list(state.outcomes.keys()):
                if not outcome_id < 0:
                    state.remove_outcome(outcome_id)

            for dp_id in list(state.input_data_ports.keys()):
                state.remove_input_data_port(dp_id)

            for dp_id in list(state.output_data_ports.keys()):
                state.remove_output_data_port(dp_id)

        if isinstance(state, ContainerState):
            for dp_id in list(state.scoped_variables.keys()):
                state.remove_scoped_variable(dp_id)

        state.name = stored_state.name
        state.semantic_data = stored_state.semantic_data
        if isinstance(state, ExecutionState):
            try:
                state.script_text = stored_state.script_text
            except:
                pass  # Tolerate script compilation errors

        if is_root:
            for dp_id, dp in stored_state.input_data_ports.items():
                state.add_input_data_port(dp.name, dp.data_type, dp.default_value, dp.data_port_id)
                assert dp_id in state.input_data_ports

            for dp_id, dp in stored_state.output_data_ports.items():
                scoped_str = str([])
                if isinstance(state, ContainerState):
                    scoped_str = str(list(state.scoped_variables.keys()))
                state.add_output_data_port(dp.name, dp.data_type, dp.default_value, dp.data_port_id)
                assert dp_id in state.output_data_ports

            for oc_id, oc in stored_state.outcomes.items():
                if not oc_id < 0:
                    state.add_outcome(oc.name, oc_id)
            for oc_id, oc in stored_state.outcomes.items():
                assert oc_id in state.outcomes

        if isinstance(state, ContainerState):
            for dp_id, sv in stored_state.scoped_variables.items():
                state.add_scoped_variable(sv.name, sv.data_type, sv.default_value, sv.data_port_id)

            if UNIQUE_DECIDER_STATE_ID in stored_state.states:
                state.add_state(stored_state.states[UNIQUE_DECIDER_STATE_ID], storage_load=True)

            for new_state in stored_state.states.values():
                if not new_state.state_id == UNIQUE_DECIDER_STATE_ID:
                    state.add_state(new_state)
                    if isinstance(new_state, ExecutionState):
                        try:
                            state.states[new_state.state_id].script_text = new_state.script_text
                        except:
                            pass  # Tolerate script compilation errors

            if isinstance(state, BarrierConcurrencyState):
                for t_id in list(state.transitions.keys()):
                    state.remove_transition(t_id)

            for t_id, t in stored_state.transitions.items():
                state.add_transition(t.from_state, t.from_outcome, t.to_state, t.to_outcome, t.transition_id)

            for t in list(state.transitions.values()):
                if UNIQUE_DECIDER_STATE_ID == t.from_state and UNIQUE_DECIDER_STATE_ID == t.to_state:
                    state.remove_transition(t.transition_id)

            for df_id, df in stored_state.data_flows.items():
                state.add_data_flow(df.from_state, df.from_key, df.to_state, df.to_key, df.data_flow_id)

    def add_core_object_to_state(self, state, core_obj):
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
        Action.__init__(self, parent_path, state_machine_model, overview)

        self.with_verbose = False
        self.state_image_for_state_type_change_signal_hook = None

    def update_root_state_from_image(self, state, state_image):
        root_state_image = create_state_from_image(state_image)

        if self.with_verbose:
            logger.verbose("#H# TRY STATE_HELPER storage type {0} current type {1}"
                           "".format(type(root_state_image), state.__class__))
            if root_state_image.__class__ == state.__class__:
                logger.verbose("SM update_root_state_from_image: with NO type change")

        new_state_class = root_state_image.__class__

        logger.verbose("DO root version change " + self.action_type)

        previous_model = self.state_machine_model.root_state
        affected_models = [previous_model, ]
        # TODO affected models should be more to allow recursive notification scheme and less updated elements
        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=affected_models, after=False)

        if self.action_type == 'change_root_state_type':
            # observe root state model (type change signal)
            self.state_image_for_state_type_change_signal_hook = state_image
            assert isinstance(self.state_machine_model.root_state.state, State)
            old_root_state_m = self.state_machine_model.root_state
            self.observe_model(self.state_machine_model.root_state)

            import rafcon.gui.helpers.state as gui_helper_state
            gui_helper_state.change_state_type(old_root_state_m, new_state_class)
            self.state_image_for_state_type_change_signal_hook = None
            self.relieve_model(old_root_state_m)
        else:
            raise TypeError("Wrong action type")

        affected_models.append(self.state_machine_model.root_state)
        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=affected_models, after=True)

    @ModelMT.observe("action_signal", signal=True)
    def action_signal(self, model, prop_name, info):
        msg = info['arg']
        if msg.action == 'change_root_state_type' and msg.after:
            new_state_m = msg.affected_models[-1]
            logger.info("action state-type-change action-signal hook for root {}".format(new_state_m))
            state_image = self.state_image_for_state_type_change_signal_hook
            root_state_image = create_state_from_image(state_image)

            self.update_state(new_state_m.state, root_state_image)

            insert_state_meta_data(meta_dict=state_image.meta_data, state_model=new_state_m)

    def redo(self):
        state = self.state_machine.root_state

        self.update_root_state_from_image(state, self.after_state_image)

    def undo(self):
        """ General Undo, that takes all elements in the parent and
        :return:
        """
        state = self.state_machine.root_state

        self.update_root_state_from_image(state, self.before_state_image)


class AddObjectAction(Action):
    """ The class handles all adding object action of 7 valid kinds
    (of In-OutputDataPort, ScopedVariable, DataFlow, Outcome, Transition and State)
    """
    possible_method_names = ['add_state', 'add_outcome', 'add_input_data_port', 'add_output_data_port',
                             'add_transition', 'add_data_flow', 'add_scoped_variable']

    def __init__(self, parent_path, state_machine_model, overview):
        Action.__init__(self, parent_path, state_machine_model, overview)

        self.changed_object = overview.get_affected_core_element()

        self.parent_identifier = ''
        self.added_object_identifier = ''
        self.added_object_args = ''

        self.parent_identifier = self.parent_path

    def set_after(self, overview):
        Action.set_after(self, overview)
        # get new object from respective list and create identifier
        list_name = overview.get_cause().replace('add_', '') + 's'
        new_object = getattr(overview.get_method_args()[0], list_name)[overview.get_result()]
        self.added_object_identifier = CoreObjectIdentifier(new_object)

    def redo(self):
        """
        :return: Redo of adding object action is simply done by adding the object again from the after_state_image of the
                 parent state.
        """
        state = self.get_state_changed()
        state_image = self.after_state_image

        assert state.get_path() == state_image.state_path
        path_of_state = state.get_path()
        state_image_of_state = create_state_from_image(state_image)

        previous_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=[previous_model, ], after=False)

        if self.added_object_identifier._type in ['InputDataPort', 'OutputDataPort', 'Outcome']:
            [state, state_image_of_state] = self.correct_reference_state(state,
                                                                             state_image_of_state,
                                                                             storage_path=state_image.state_path)
        list_name = self.action_type.replace('add_', '') + 's'
        core_obj = getattr(state_image_of_state, list_name)[self.added_object_identifier._id]
        self.add_core_object_to_state(state, core_obj)

        actual_state_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        self.compare_models(previous_model, actual_state_model)
        insert_state_meta_data(meta_dict=state_image.meta_data,
                               state_model=actual_state_model, level=None if self.action_type == 'add_state' else 1)
        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=[previous_model, ], after=True)

    def undo(self):

        # find object
        state = self.get_state_changed()
        state_image = self.after_state_image

        assert state.get_path() == state_image.state_path
        path_of_state = state.get_path()
        state_image_of_state = create_state_from_image(state_image)

        previous_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=[previous_model, ], after=False)

        if self.added_object_identifier._type in ['InputDataPort', 'OutputDataPort', 'Outcome']:
            [state, state_image_of_state] = self.correct_reference_state(state,
                                                                             state_image_of_state,
                                                                             storage_path=state_image.state_path)

        list_name = self.action_type.replace('add_', '') + 's'
        core_obj = getattr(state_image_of_state, list_name)[self.added_object_identifier._id]
        # undo
        self.remove_core_object_from_state(state, core_obj)

        actual_state_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        self.compare_models(previous_model, actual_state_model)
        insert_state_meta_data(meta_dict=state_image.meta_data,
                               state_model=actual_state_model, level=1)
        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=[previous_model, ], after=True)

    def correct_reference_state(self, state, state_image_of_state, storage_path):

        partial_path = self.added_object_identifier._path.split('/')
        for path_element in storage_path.split('/'):
            partial_path.pop(0)
        for path_element in partial_path:
            state_image_of_state = state_image_of_state.states[path_element]
            state = state.states[path_element]

        return state, state_image_of_state


class RemoveObjectAction(Action):
    possible_method_names = ['remove_state', 'remove_outcome', 'remove_input_data_port', 'remove_output_data_port',
                             'remove_transition', 'remove_data_flow', 'remove_scoped_variable']

    def __init__(self, parent_path, state_machine_model, overview):
        Action.__init__(self, parent_path, state_machine_model, overview)

        assert overview.get_cause() in self.possible_method_names
        assert overview.get_affected_property() == 'state' and isinstance(overview.get_affected_core_element(), State)

        self.instance_path = overview.get_affected_core_element().get_path()
        self.changed_object = self.changed_object = overview.get_affected_core_element()

        self.parent_identifier = ''
        self.removed_object_identifier = ''
        self.removed_object_args = ''

        if "outcome" in overview.get_cause() or "data_port" in overview.get_cause():
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
        overview = self.before_overview
        # get new object from respective list and create identifier
        object_type_name = overview.get_cause().replace('remove_', '')
        list_name = object_type_name + 's'
        if object_type_name + '_id' in overview.get_method_kwargs():
            object_id = overview.get_method_kwargs()[object_type_name + '_id']
        else:
            if len(overview.get_method_args()) < 2:
                logger.error("Length of args-tuple is shorter as assumed.")
            else:
                object_id = overview.get_method_args()[1]
        new_object = getattr(overview.get_method_args()[0], list_name)[object_id]
        self.removed_object_identifier = CoreObjectIdentifier(new_object)

    def undo(self):
        state = self.get_state_changed()
        state_image = self.before_state_image

        assert state.get_path() == state_image.state_path
        path_of_state = state.get_path()
        state_image_of_state = create_state_from_image(state_image)

        previous_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=[previous_model, ], after=False)

        if self.removed_object_identifier._type in ['InputDataPort', 'OutputDataPort', 'Outcome']:
            [state, state_image_of_state] = self.correct_reference_state(state,
                                                                             state_image_of_state,
                                                                             storage_path=state_image.state_path)
        list_name = self.action_type.replace('remove_', '') + 's'
        core_obj = getattr(state_image_of_state, list_name)[self.removed_object_identifier._id]

        if self.action_type not in ['remove_transition', 'remove_data_flow']:
            self.add_core_object_to_state(state, core_obj)

        self.adjust_linkage()

        actual_state_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        self.compare_models(previous_model, actual_state_model)
        insert_state_meta_data(meta_dict=state_image.meta_data,
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
        insert_state_meta_data(meta_dict=self.after_state_image.meta_data,
                               state_model=actual_state_model, level=1)

        # signal that undo/redo action was performed -> run graphical editor update
        self.emit_undo_redo_signal(action_parent_m=previous_model, affected_models=[previous_model, ], after=True)

    def correct_reference_state(self, state, state_image_of_state, storage_path):

        partial_path = self.removed_object_identifier._path.split('/')
        for path_element in storage_path.split('/'):
            logger.debug("pop: " + partial_path.pop(0))
        for path_element in partial_path:
            state_image_of_state = state_image_of_state.states[path_element]
            state = state.states[path_element]
            logger.debug("state is now: {0} {1}".format(state.state_id, state_image_of_state.state_id))

        return state, state_image_of_state

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
    after_arguments = None

    def __init__(self, parent_path, state_machine_model, overview):
        AbstractAction.__init__(self, parent_path, state_machine_model, overview)

        # validate class type
        assert isinstance(self.before_overview.get_affected_core_element(), self._object_class)

        # validate method call -- action type
        self.action_type = overview.get_cause()
        if not self.action_type in self.possible_method_names:
            logger.error("{0} is not possible with overview {1}".format(self.__class__.__name__, overview))
        assert self.action_type in self.possible_method_names

        # validate object path
        self.object_identifier = CoreObjectIdentifier(self.before_overview.get_affected_core_element())
        assert self.parent_path == self.object_identifier._path

        self.before_arguments = self.get_set_of_arguments(self.before_overview.get_affected_core_element())

        self.state_machine = state_machine_model.state_machine

    def as_dict(self):
        d = AbstractAction.as_dict(self)
        d.update({"before_arguments": self.before_arguments,
                  "after_arguments": self.after_arguments})
        return d

    @staticmethod
    def get_set_of_arguments(elem):
        raise NotImplementedError()

    def get_state_image(self):
        # TODO: This method should not be called. It is currently only called by AbstractAction.__init__ which is
        # called by StateElementAction.__init__
        # => refactor StateElementAction to not create state image
        return None

    def set_after(self, overview):
        self.after_overview = overview
        assert isinstance(self.after_overview.get_affected_core_element(), self._object_class)
        self.after_arguments = self.get_set_of_arguments(self.after_overview.get_affected_core_element())


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
        self.update_data_flow_from_image(df, self.before_arguments)

    def redo(self):
        df = self.state_machine.get_state_by_path(self.parent_path).data_flows[self.before_arguments['data_flow_id']]
        self.update_data_flow_from_image(df, self.after_arguments)

    def update_data_flow_from_image(self, df, arguments):
        if self.action_type in self.possible_args:
            setattr(df, self.action_type, arguments[self.action_type])
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
        self.update_transition_from_image(t, self.before_arguments)

    def redo(self):
        t = self.state_machine.get_state_by_path(self.parent_path).transitions[self.before_arguments['transition_id']]
        self.update_transition_from_image(t, self.after_arguments)

    def update_transition_from_image(self, t, arguments):
        if self.action_type in self.possible_args:
            setattr(t, self.action_type, arguments[self.action_type])
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
        self.update_data_port_from_image(dp, self.before_arguments)

    def redo(self):
        dp = self.state_machine.get_state_by_path(self.parent_path).get_data_port_by_id(self.before_arguments['data_port_id'])
        self.update_data_port_from_image(dp, self.after_arguments)

    def update_data_port_from_image(self, dp, arguments):
        if self.action_type in self.possible_args:
            setattr(dp, self.action_type, arguments[self.action_type])
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
        self.update_outcome_from_image(oc, self.before_arguments)

    def redo(self):
        oc = self.state_machine.get_state_by_path(self.parent_path).outcomes[self.before_arguments['outcome_id']]
        self.update_outcome_from_image(oc, self.after_arguments)

    def update_outcome_from_image(self, oc, arguments):
        if self.action_type in self.possible_args:
            setattr(oc, self.action_type, arguments[self.action_type])
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
        if overview.get_cause() in ['outcomes', 'input_data_ports', 'output_data_ports']:  # need State's parent
            if isinstance(overview.get_affected_core_element().parent, State):
                parent_path = overview.get_affected_core_element().parent.get_path()
        Action.__init__(self, parent_path, state_machine_model, overview)

        self.state_machine = state_machine_model.state_machine
        if self.action_type not in self.possible_method_names:
            logger.error("action_type: '{0}' not in {1}".format(self.action_type, self.possible_method_names))
        assert self.action_type in self.possible_method_names
        assert isinstance(self.before_overview.get_affected_core_element(), State)
        self.object_identifier = CoreObjectIdentifier(self.before_overview.get_affected_core_element())
        if overview.get_cause() in ['outcomes', 'input_data_ports', 'output_data_ports']:
            assert self.parent_path == CoreObjectIdentifier(self.before_overview.get_affected_core_element().parent)._path
        else:
            assert self.parent_path == self.object_identifier._path
        self.before_arguments = self.get_set_of_arguments(self.before_overview.get_affected_core_element())
        self.after_arguments = None
        if self.action_type == 'script_text' and isinstance(self.before_overview.get_method_args()[-1][1], string_types):
            d = difflib.Differ()
            diff = list(d.compare(self.before_overview.get_method_args()[0].script_text.split('\n'),
                                  self.before_overview.get_method_args()[1].split('\n')))
            self.script_diff = '\n'.join(diff)
        else:
            self.script_diff = None
        if self.action_type == 'description':
            d = difflib.Differ()
            diff = list(d.compare(self.before_overview.get_method_args()[0].description.split('\n') if self.before_overview.get_method_args()[0].description else [''] ,
                                  self.before_overview.get_method_args()[1].split('\n') if self.before_overview.get_method_args()[1] else ['']))
            self.description_diff = '\n'.join(diff)
        else:
            self.description_diff = None
        if 'semantic_data' in overview.get_cause():
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
        assert isinstance(self.after_overview.get_affected_core_element(), State)
        self.after_arguments = self.get_set_of_arguments(self.after_overview.get_affected_core_element())

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
            self.update_property_from_image(s, self.before_arguments)
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
            self.update_property_from_image(s, self.after_arguments)
        else:
            assert False

    def update_property_from_image(self, s, arguments):
        if self.action_type in self.possible_args:
            property = self.substitute_dict.get(self.action_type, self.action_type)
            setattr(s, property, copy.deepcopy(arguments[property]))
        else:
            assert False


class Group(Action):
    def __init__(self, *args, **kwargs):
        Action.__init__(self, *args, **kwargs)


class UnGroup(Action):
    def __init__(self, *args, **kwargs):
        Action.__init__(self, *args, **kwargs)
