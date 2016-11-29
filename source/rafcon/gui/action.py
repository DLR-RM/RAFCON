"""Action class for history

The Action-Class provides a general redo or undo functionality for any action, as long as the the class object was
initialized with consistent arguments.

This general Action (one procedure for all possible edition) procedure is expansive and complex, therefore it is aimed
to define specific _-Action-Classes for simple/specific edit actions.
"""
import copy
import json
import difflib

from gtkmvc import ModelMT

from jsonconversion.decoder import JSONObjectDecoder
from jsonconversion.encoder import JSONObjectEncoder

from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState, DeciderState
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.states.library_state import LibraryState
from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from rafcon.core.states.state import State
from rafcon.core.state_machine import StateMachine
from rafcon.core.state_elements.data_flow import DataFlow
from rafcon.core.state_elements.data_port import DataPort
from rafcon.core.state_elements.data_port import InputDataPort, OutputDataPort
from rafcon.core.state_elements.outcome import Outcome
from rafcon.core.state_elements.scope import ScopedData, ScopedVariable
from rafcon.core.state_elements.transition import Transition
from rafcon.core.storage import storage
from rafcon.core.constants import UNIQUE_DECIDER_STATE_ID
from rafcon.core.global_variable_manager import GlobalVariableManager
from rafcon.core.library_manager import LibraryManager
from rafcon.core.script import Script

import rafcon.gui.singleton as mvc_singleton
import rafcon.gui.config as gui_config
from rafcon.gui.models.container_state import ContainerState, ContainerStateModel
from rafcon.gui.models.signals import MetaSignalMsg
from rafcon.gui.utils.notification_overview import NotificationOverview

from rafcon.utils import log
from rafcon.utils.storage_utils import substitute_modules
from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE, BY_EXECUTION_TRIGGERED_OBSERVABLE_STATE_METHODS

logger = log.get_logger(__name__)

core_object_list = [Transition, DataFlow, Outcome, InputDataPort, OutputDataPort, ScopedData, ScopedVariable, Script,
                    GlobalVariableManager, LibraryManager, StateMachine,
                    ExecutionState, HierarchyState, BarrierConcurrencyState, PreemptiveConcurrencyState, LibraryState,
                    DeciderState]

DEBUG_META_REFERENCES = False
HISTORY_DEBUG_LOG_FILE = RAFCON_TEMP_PATH_BASE + '/test_file.txt'


def get_state_tuple(state, state_m=None):
    """ Generates a tuple that holds the state as yaml-strings and its meta data in a dictionary.
    The tuple consists of:
    [0] yaml_str for state,
    [1] dict of child_state tuples,
    [2] script of state,
    [3] dict of model_meta-data of self and elements
    [4] path of state in state machine
    [5] script_text
    #   states-meta - [state-, transitions-, data_flows-, outcomes-, inputs-, outputs-, scopes, states-meta]

    :param rafcon.core.states.state.State state: The state that should be stored
    :return: state_tuple tuple
    """
    # state_str = yaml.dump(state)
    state_str = json.dumps(state, cls=JSONObjectEncoder, nested_jsonobjects=False,
                           indent=4, check_circular=False, sort_keys=True)

    # print "++++++++++", state
    state_tuples_dict = {}
    if isinstance(state, ContainerState):
        # print state.states, "\n"
        for child_state_id, child_state in state.states.iteritems():
            # print "child_state: %s" % child_state_id, child_state, "\n"
            state_tuples_dict[child_state_id] = get_state_tuple(child_state)

    if state_m is not None:
        state_meta_dict = get_state_element_meta(state_m)
    else:
        state_meta_dict = {}

    if not isinstance(state, ExecutionState):
        script_content = "Dummy Script"
        script = Script(parent=None)
    else:
        script_content = state.script.script
        script = state.script
    state_tuple = (state_str, state_tuples_dict, script, state_meta_dict, state.get_path(), script_content)

    return state_tuple


def get_state_from_state_tuple(state_tuple):
    # print "++++ new state", state_tuple

    # Transitions and data flows are not added, as also states are not added
    # We have to wait until the child states are loaded, before adding transitions and data flows, as otherwise the
    # validity checks for transitions and data flows would fail
    # state_info = yaml.load(state_tuple[0])
    state_info = json.loads(state_tuple[0], cls=JSONObjectDecoder, substitute_modules=substitute_modules)
    if not isinstance(state_info, tuple):
        state = state_info
    else:
        state = state_info[0]
        transitions = state_info[1]
        data_flows = state_info[2]

    if isinstance(state, BarrierConcurrencyState):
        # logger.debug("\n\ninsert decider_state\n\n")
        child_state = get_state_from_state_tuple(state_tuple[1][UNIQUE_DECIDER_STATE_ID])
        # do_storage_test(child_state)
        for t in state.transitions.values():
            if UNIQUE_DECIDER_STATE_ID in [t.from_state, t.to_state]:
                state.remove_transition(t.transition_id)
        try:
            state.add_state(child_state)
        except:
            if not UNIQUE_DECIDER_STATE_ID in state.states:
                logger.error("Could not insert DeciderState!!! while it is in NOT in already!!! {0} {1}".format(
                    UNIQUE_DECIDER_STATE_ID in state.states, child_state.state_id == UNIQUE_DECIDER_STATE_ID))

    # state.script = state_tuple[2]
    if isinstance(state, ExecutionState):
        state.script_text = state_tuple[5]
    # print "------------- ", state
    for child_state_id, child_state_tuple in state_tuple[1].iteritems():
        child_state = get_state_from_state_tuple(child_state_tuple)
        # do_storage_test(child_state)

        # print "++++ new cild", child_state  # child_state_tuple, child_state
        if not child_state.state_id == UNIQUE_DECIDER_STATE_ID:
            try:
                state.add_state(child_state)
            except Exception as e:
                logger.debug(str(e))
                logger.error(
                    "try to add state %s to state %s with states %s" % (child_state, state, state.states.keys()))

        def print_states(state):
            if isinstance(state, ContainerState):
                for state_id, child_state in state.states.iteritems():
                    print child_state.get_path()
                    print_states(child_state)
                    # print "got from tuple:"
                    # print_states(state)

    # Child states were added, now we can add transitions and data flows
    if isinstance(state_info, tuple):
        state.transitions = transitions
        state.data_flows = data_flows

    return state


def reference_free_check(v1, v2, prepend=[]):
    """Returns elements of a dict that have the same memory addresses except strings."""
    d = {'value': {}, 'same_ref': [], 'same_ref_value': [], 'missing_keys1': [], 'missing_keys2': []}
    v1_keys = v1.keys()
    v2_keys = v2.keys()
    not_to_check = set(v1_keys).symmetric_difference(v2_keys)
    d['missing_keys1'] = filter(lambda k: k in not_to_check, v1_keys)
    d['missing_keys2'] = filter(lambda k: k in not_to_check, v2_keys)
    for key in set(v1_keys + v2_keys):
        if key not in not_to_check:
            if not hasattr(v1[key], 'keys'):
                if isinstance(v1[key], str):
                    d['value'].update({key: v1[key]})
                else:
                    if id(v1[key]) == id(v2[key]):
                        if not isinstance(v1[key], tuple):
                            d['same_ref'].append(prepend + [key])
                            d['same_ref_value'].append(str(v1[key]) + " == " + str(v2[key]) + ', ' + str(type(v1[key])) + " == " + str(type(v2[key])))
                    else:
                        d['value'].update({key: v1[key]})
            else:
                if id(v1[key]) == id(v2[key]):
                    d['same_ref'].append(prepend + [key])
                    d['same_ref_value'].append(str(v1[key]) + " == " + str(v2[key]) + ', ' + str(type(v1[key])) + " == " + str(type(v2[key])))
                else:
                    d['value'].update({key: reference_free_check(v1[key], v2[key], prepend=prepend + [key])})

    return d


def meta_dump_or_deepcopy(meta):
    if DEBUG_META_REFERENCES:  # debug copy
        meta_source = meta
        meta_str = json.dumps(meta, cls=JSONObjectEncoder, nested_jsonobjects=False,
                              indent=4, check_circular=False, sort_keys=True)
        meta_dump_copy = json.loads(meta_str, cls=JSONObjectDecoder, substitute_modules=substitute_modules)
        meta_deepcopy = copy.deepcopy(meta)

        meta_source_str = json.dumps(meta, cls=JSONObjectEncoder, nested_jsonobjects=False,
                                     indent=4, check_circular=False, sort_keys=True)
        meta_dump_copy_str = json.dumps(meta, cls=JSONObjectEncoder, nested_jsonobjects=False,
                                        indent=4, check_circular=False, sort_keys=True)
        meta_deepcopy_str = json.dumps(meta, cls=JSONObjectEncoder, nested_jsonobjects=False,
                                       indent=4, check_circular=False, sort_keys=True)
        assert meta_dump_copy_str == meta_source_str
        assert meta_dump_copy_str == meta_deepcopy_str

        def diff_print(diff):
            if diff['same_ref']:
                print "same_ref: ", diff['same_ref'], diff['same_ref_value']
                assert False
            for value in diff['value'].itervalues():
                if isinstance(value, dict):
                    diff_print(value)

        source_dump_diff = reference_free_check(meta_source, meta_dump_copy)
        source_deep_diff = reference_free_check(meta_source, meta_deepcopy)
        print "source_dump_diff"
        diff_print(source_dump_diff)
        print "source_deep_diff"
        diff_print(source_deep_diff)


    # print meta_str
    # if gui_config.global_gui_config.get_config_value('GAPHAS_EDITOR'):
    #     meta_str = json.dumps(meta, cls=JSONObjectEncoder, nested_jsonobjects=False,
    #                           indent=4, check_circular=False, sort_keys=True)
    #     return json.loads(meta_str, cls=JSONObjectDecoder, substitute_modules=substitute_modules)
    # else:
    return copy.deepcopy(meta)


def get_state_element_meta(state_model, with_parent_linkage=True, with_prints=False, level=None):
    meta_dict = {'state': copy.deepcopy(state_model.meta), 'is_start': False, 'data_flows': {}, 'transitions': {},
                 'outcomes': {}, 'input_data_ports': {}, 'output_data_ports': {}, 'scoped_variables': {}, 'states': {},
                 'related_parent_transitions': {}, 'related_parent_data_flows': {}}
    if with_parent_linkage:
        with_parent_linkage = False
        if not state_model.state.is_root_state:
            state_id = state_model.state.state_id
            for transition_m in state_model.parent.transitions:
                transition = transition_m.transition
                if transition.from_state == state_id or transition.to_state == state_id:
                    meta_dict['related_parent_transitions'][transition.transition_id] = meta_dump_or_deepcopy(transition_m.meta)
            for data_flow_m in state_model.parent.data_flows:
                data_flow = data_flow_m.data_flow
                if data_flow.from_state == state_id or data_flow.to_state == state_id:
                    meta_dict['related_parent_data_flows'][data_flow.data_flow_id] = meta_dump_or_deepcopy(data_flow_m.meta)

    if with_prints:
        print "STORE META for STATE: ", state_model.state.state_id, state_model.state.name
    meta_dict['is_start'] = state_model.is_start
    for elem in state_model.outcomes:
        meta_dict['outcomes'][elem.outcome.outcome_id] = meta_dump_or_deepcopy(elem.meta)
        if with_prints:
            print "outcome: ", elem.outcome.outcome_id, elem.parent.state.outcomes.keys(), meta_dict['outcomes'].keys()
    for elem in state_model.input_data_ports:
        meta_dict['input_data_ports'][elem.data_port.data_port_id] = meta_dump_or_deepcopy(elem.meta)
        if with_prints:
            print "input: ", elem.data_port.data_port_id, elem.parent.state.input_data_ports.keys(), \
                meta_dict['input_data_ports'].keys()
    for elem in state_model.output_data_ports:
        meta_dict['output_data_ports'][elem.data_port.data_port_id] = meta_dump_or_deepcopy(elem.meta)
        if with_prints:
            print "output: ", elem.data_port.data_port_id, elem.parent.state.output_data_ports.keys(), \
                meta_dict['output_data_ports'].keys()

    meta_dict['state'] = meta_dump_or_deepcopy(state_model.meta)
    if isinstance(state_model, ContainerStateModel):
        for state_id, state_m in state_model.states.iteritems():
            meta_dict['states'][state_m.state.state_id] = get_state_element_meta(state_m, with_parent_linkage)
            if with_prints:
                print "FINISHED STORE META for STATE: ", state_id, meta_dict['states'].keys(), \
                    state_model.state.state_id
        for elem in state_model.transitions:
            meta_dict['transitions'][elem.transition.transition_id] = meta_dump_or_deepcopy(elem.meta)
            if with_prints:
                print "transition: ", elem.transition.transition_id, elem.parent.state.transitions.keys(), \
                    meta_dict['transitions'].keys(), elem.parent.state.state_id
        for elem in state_model.data_flows:
            meta_dict['data_flows'][elem.data_flow.data_flow_id] = meta_dump_or_deepcopy(elem.meta)
            if with_prints:
                print "data_flow: ", elem.data_flow.data_flow_id, elem.parent.state.data_flows.keys(), \
                    meta_dict['data_flows'].keys()
        for elem in state_model.scoped_variables:
            meta_dict['scoped_variables'][elem.scoped_variable.data_port_id] = meta_dump_or_deepcopy(elem.meta)
            if with_prints:
                print "scoped_variable: ", elem.scoped_variable.data_port_id, \
                    elem.parent.state.scoped_variables.keys(), meta_dict['scoped_variables'].keys()
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


def insert_state_meta_data(meta_dict, state_model, with_prints=False, level=None):
    # meta_dict = {'state': state_model.meta, 'data_flows': {}, 'transitions': {}, 'outcomes': {},
    #              'input_data_ports': {}, 'output_data_ports': {}, 'scoped_variables': {}}

    def missing_meta_data_warning(state_model, elem, meta_dict, dict_key, existing_model_list):
        logger.warning("Storage Dict seems to miss Meta-Data of {5} in State: {0} {1} for {5}:"
                       " {2}\nreal: {3}\nstorage: {4}".format(state_model.state.state_id,
                                                              state_model.state.name,
                                                              elem,
                                                              existing_model_list,
                                                              meta_dict[dict_key],
                                                              dict_key[:-1].replace('_', '-')))

    state_model.meta = meta_dump_or_deepcopy(meta_dict['state'])
    if with_prints:
        print "INSERT META for STATE: ", state_model.state.state_id, state_model.state.name

    for elem in state_model.outcomes:
        if elem.outcome.outcome_id in meta_dict['outcomes']:
            elem.meta = meta_dump_or_deepcopy(meta_dict['outcomes'][elem.outcome.outcome_id])
        else:
            missing_meta_data_warning(state_model, elem.outcome, meta_dict, 'outcomes',
                                      [oc_m.outcome.outcome_id for oc_m in state_model.outcomes])
    for elem in state_model.input_data_ports:
        if elem.data_port.data_port_id in meta_dict['input_data_ports']:
            elem.meta = meta_dump_or_deepcopy(meta_dict['input_data_ports'][elem.data_port.data_port_id])
        else:
            missing_meta_data_warning(state_model, elem.data_port, meta_dict, 'input_data_ports',
                                      [ip_m.data_port.data_port_id for ip_m in state_model.input_data_ports])

    for elem in state_model.output_data_ports:
        if elem.data_port.data_port_id in meta_dict['output_data_ports']:
            elem.meta = meta_dump_or_deepcopy(meta_dict['output_data_ports'][elem.data_port.data_port_id])
        else:
            missing_meta_data_warning(state_model, elem.data_port, meta_dict, 'output_data_ports',
                                      [op_m.data_port.data_port_id for op_m in state_model.output_data_ports])

    if isinstance(state_model, ContainerStateModel):
        for state_id, state_m in state_model.states.iteritems():
            if with_prints:
                print "FIN: ", state_id, state_m.state.state_id, meta_dict['states'].keys(), state_model.state.state_id
            if state_m.state.state_id in meta_dict['states']:
                if level is None:
                    insert_state_meta_data(meta_dict['states'][state_m.state.state_id], state_m, with_prints)
                elif level > 0:
                    insert_state_meta_data(meta_dict['states'][state_m.state.state_id], state_m, with_prints, level - 1)
                else:
                    pass
            else:
                logger.warning("no meta data for STATE: '{0}' in storage".format(state_m.state.state_id))

            if with_prints:
                print "FINISHED META for STATE: ", state_m.state.state_id
        for elem in state_model.transitions:
            if elem.transition.transition_id in meta_dict['transitions']:
                elem.meta = meta_dump_or_deepcopy(meta_dict['transitions'][elem.transition.transition_id])
            else:
                missing_meta_data_warning(state_model, elem.transition, meta_dict, 'transitions',
                                          [t_m.transition.transition_id for t_m in state_model.transitions])

        for elem in state_model.data_flows:
            if elem.data_flow.data_flow_id in meta_dict['data_flows']:
                elem.meta = meta_dump_or_deepcopy(meta_dict['data_flows'][elem.data_flow.data_flow_id])
            else:
                missing_meta_data_warning(state_model, elem.data_flow, meta_dict, 'data_flows',
                                          [df_m.data_flow.data_flow_id for df_m in state_model.data_flows])

        for elem in state_model.scoped_variables:
            if elem.scoped_variable.data_port_id in meta_dict['scoped_variables']:
                elem.meta = meta_dump_or_deepcopy(meta_dict['scoped_variables'][elem.scoped_variable.data_port_id])
            else:
                missing_meta_data_warning(state_model, elem.scoped_variable, meta_dict, 'scoped_variables',
                                          [sv_m.scoped_variable.data_port_id for sv_m in state_model.scoped_variables])

    # state_model.is_start = copy.deepcopy(meta_dict['is_start'])
    check_state_model_for_is_start_state(state_model)


class ActionDummy:
    def __init__(self, overview=None):
        if overview is None:
            self.before_overview = NotificationOverview()
        else:
            self.before_overview = overview
        self.after_overview = None

    def set_after(self, overview=None):
        self.after_overview = overview

    def get_storage(self):
        pass

    def undo(self):
        pass

    def redo(self):
        pass


class MetaAction:

    def __init__(self, parent_path, state_machine_model, overview):

        assert isinstance(overview, NotificationOverview)
        assert overview['type'] == 'signal'

        self.type = "change " + overview['meta_signal'][-1]['change']
        overview['method_name'].append("change " + overview['meta_signal'][-1]['change'])
        overview['info'][-1]['method_name'] = "change " + overview['meta_signal'][-1]['change']
        overview['instance'].append(overview['model'][-1])
        overview['info'][-1]['instance'] = overview['model'][-1]

        meta_str = json.dumps(overview['model'][-1].meta, cls=JSONObjectEncoder, nested_jsonobjects=False,
                              indent=4, check_circular=False, sort_keys=True)
        # print meta_str
        self.meta = json.loads(meta_str, cls=JSONObjectDecoder, substitute_modules=substitute_modules)

        self.state_machine = state_machine_model.state_machine
        self.state_machine_model = state_machine_model
        self.parent_path = parent_path

        self.before_overview = overview
        self.before_storage = self.get_storage()  # tuple of state and states-list of storage tuple

        self.after_overview = None
        self.after_storage = None  # tuple of state and states-list of storage tuple

        self.__version_id = None

    @property
    def version_id(self):
        return self.__version_id

    @version_id.setter
    def version_id(self, value):
        if self.__version_id is None:
            self.__version_id = value
        else:
            logger.warning("The version_id of an action is not allowed to be modify after first assignment")

    def set_after(self, overview):
        self.after_overview = overview
        self.after_storage = self.get_storage()  # tuple of state and states-list of storage tuple

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
        if self.before_overview['meta_signal'][-1]['affects_children']:
            insert_state_meta_data(meta_dict=self.before_storage, state_model=state_m)
            state_m.meta_signal.emit(MetaSignalMsg("redo_meta_action", "all", True))
            # if state_m.state.is_root_state:
            #     self.state_machine_model.state_meta_signal.emit(MetaSignalMsg("undo_meta_action", "all", False))
        else:
            insert_state_meta_data(meta_dict=self.before_storage, state_model=state_m)
            # if state_m.state.is_root_state:
            #     self.state_machine_model.state_meta_signal.emit(MetaSignalMsg("undo_meta_action", "all", False))
            state_m.meta_signal.emit(MetaSignalMsg("redo_meta_action", "all", False))

    def redo(self):
        # TODO check why levels are not working
        # TODO in future emit signal only for respective model
        state_m = self.get_state_model_changed()
        # logger.info("META-Action undo {}".format(state_m.state.get_path()))
        if self.before_overview['meta_signal'][-1]['affects_children']:
            insert_state_meta_data(meta_dict=self.after_storage, state_model=state_m)
            state_m.meta_signal.emit(MetaSignalMsg("redo_meta_action", "all", True))
            # if state_m.state.is_root_state:
            #     self.state_machine_model.state_meta_signal.emit(MetaSignalMsg("redo_meta_action", "all", False))
        else:
            insert_state_meta_data(meta_dict=self.after_storage, state_model=state_m)
            # if state_m.state.is_root_state:
            #     self.state_machine_model.state_meta_signal.emit(MetaSignalMsg("redo_meta_action", "all", False))
            state_m.meta_signal.emit(MetaSignalMsg("redo_meta_action", "all", False))


class Action(ModelMT):
    __version_id = None

    def __init__(self, parent_path, state_machine_model, overview):
        ModelMT.__init__(self)
        assert isinstance(overview, NotificationOverview)
        self.type = overview['method_name'][-1]
        self.state_machine = state_machine_model.state_machine
        self.state_machine_model = state_machine_model
        self.parent_path = parent_path

        self.before_overview = overview
        self.before_model = overview['model'][0]
        self.before_prop_name = overview['prop_name'][-1]
        self.before_info = overview['info'][-1]
        self.before_storage = self.get_storage()  # tuple of state and states-list of storage tuple

        self.after_overview = None
        self.after_model = None
        self.after_prop_name = None
        self.after_info = None
        self.after_storage = None  # tuple of state and states-list of storage tuple

        # self.__version_id = None

    @property
    def version_id(self):
        return self.__version_id

    @version_id.setter
    def version_id(self, value):
        if self.__version_id is None:
            self.__version_id = value
        else:
            logger.warning("The version_id of an action is not allowed to be modify after first assignment")

    def set_after(self, overview):
        self.after_overview = overview
        self.after_model = overview['model'][0]
        self.after_prop_name = overview['prop_name'][0]
        self.after_info = overview['info'][-1]
        self.after_storage = self.get_storage()  # tuple of state and states-list of storage tuple

    def get_storage(self):

        state_tuple = get_state_tuple(self.state_machine.get_state_by_path(self.parent_path))
        state_model = self.state_machine_model.get_state_model_by_path(self.parent_path)
        state_tuple[3].update(get_state_element_meta(state_model))
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

    def stop_graphical_viewer(self):
        """ The function avoid interference with and unnecessary drawing of the graphical edit while undo or redo
        of an action by stopping the drawing process and returning the graphical viewer object.
        It is a general functionality all Action*-Classes may need.
        :return: g_sm_editor -> the actual graphical viewer for further use
        """

        # logger.debug("\n\n\n\n\n\n\nINSERT STATE: %s %s || %s || Action\n\n\n\n\n\n\n" % (path_of_state, state, storage_version_of_state))
        mw_ctrl = mvc_singleton.main_window_controller
        g_sm_editor = None
        if mvc_singleton.main_window_controller:
            g_sm_editor = mw_ctrl.get_controller_by_path(ctrl_path=['state_machines_editor_ctrl',
                                                                    self.state_machine.state_machine_id],
                                                         with_print=False)

        # We are only interested in OpenGL editors, not Gaphas ones
        try:
            import rafcon.gui.controllers.graphical_editor as graphical_editor_opengl
            if g_sm_editor and isinstance(g_sm_editor, graphical_editor_opengl.GraphicalEditorController):
                g_sm_editor.suspend_drawing = True
        except ImportError as e:
            logger.debug("OpenGL-Graphical-Editor can not be imported: {0}".format(e))

        return g_sm_editor

    @staticmethod
    def run_graphical_viewer(g_sm_editor, responsible_m):
        """ Enables and re-initiate graphical viewer's drawing process.
        :param g_sm_editor: graphical state machine editor
        """

        try:
            import rafcon.gui.controllers.graphical_editor as graphical_editor_opengl
            if g_sm_editor and isinstance(g_sm_editor, graphical_editor_opengl.GraphicalEditorController):
                g_sm_editor.suspend_drawing = False
                # TODO integrate meta-data affects_children status
                responsible_m.meta_signal.emit(MetaSignalMsg("undo_redo_action", "all", True))
        except ImportError as e:
            logger.debug("OpenGL-Graphical-Editor can not be imported: {0}".format(e))

        try:
            import rafcon.gui.controllers.graphical_editor_gaphas as graphical_editor_gaphas
            if g_sm_editor and isinstance(g_sm_editor, graphical_editor_gaphas.GraphicalEditorController):
                g_sm_editor.manual_notify_after(responsible_m)
        except ImportError as e:
            logger.debug("Gaphas-Graphical-Editor can not be imported: {0}".format(e))

    def redo(self):
        """ General Redo, that takes all elements in the parent path state stored of the before action state machine status.
        :return:
        """

        self.set_state_to_version(self.get_state_changed(), self.after_storage)

    def undo(self):
        """ General Undo, that takes all elements in the parent path state stored of the after action state machine status.
        :return:
        """

        self.set_state_to_version(self.get_state_changed(), self.before_storage)

    def set_state_to_version(self, state, storage_version):
        import rafcon.gui.state_machine_helper as state_machine_helper
        # print state.get_path(), '\n', storage_version[4]
        assert state.get_path() == storage_version[4]
        # print self.parent_path, self.parent_path.split('/'), len(self.parent_path.split('/'))
        path_of_state = state.get_path()
        storage_version_of_state = get_state_from_state_tuple(storage_version)

        assert storage_version_of_state

        g_sm_editor = self.stop_graphical_viewer()

        # if self.type == 'change_state_type':
        #     self.storage_version_for_state_type_change_signal_hook = storage_version
        #     assert isinstance(self.state_machine_model.root_state.state, State)
        #     state_parent = self.before_overview["instance"][-1]
        #     old_state_changed_ref = self.before_overview["args"][-1][1]
        #     state = self.state_machine.get_state_by_path(old_state_changed_ref.get_path())
        #     old_state_changed_in_storage = storage_version_of_state.states[state.state_id]
        #     if isinstance(old_state_changed_in_storage, state_machine_helper.HierarchyState):
        #         new_state_class = state_machine_helper.HierarchyState
        #     elif isinstance(old_state_changed_in_storage, state_machine_helper.BarrierConcurrencyState):
        #         new_state_class = state_machine_helper.BarrierConcurrencyState
        #     elif isinstance(old_state_changed_in_storage, state_machine_helper.PreemptiveConcurrencyState):
        #         new_state_class = state_machine_helper.PreemptiveConcurrencyState
        #     else:
        #         logger.info("SM set_root_state_to_version: with NO type change")
        #         new_state_class = state_machine_helper.ExecutionState
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
        insert_state_meta_data(meta_dict=storage_version[3], state_model=actual_state_model)

        self.run_graphical_viewer(g_sm_editor, actual_state_model)

    @ModelMT.observe("state_type_changed_signal", signal=True)
    def hook_for_type_change_operation(self, model, prop_name, info):
        g_sm_editor = self.stop_graphical_viewer()
        msg = info['arg']
        new_state_m = msg.new_state_m
        logger.info("action state-type-change hook for root {}".format(new_state_m))
        storage_version = self.storage_version_for_state_type_change_signal_hook
        root_state_version_from_storage = get_state_from_state_tuple(storage_version)

        self.update_state(new_state_m.state, root_state_version_from_storage)

        insert_state_meta_data(meta_dict=storage_version[3], state_model=new_state_m)  # self.state_machine_model.root_state)

    def update_state(self, state, stored_state):

        assert type(stored_state) is type(state)

        is_root = state.is_root_state

        if isinstance(state, ContainerState):

            # print state.data_flows.keys()
            for data_flow_id in state.data_flows.keys():
                state.remove_data_flow(data_flow_id)

            if not isinstance(state, BarrierConcurrencyState):
                for t_id in state.transitions.keys():
                    # if not UNIQUE_DECIDER_STATE_ID in [state.transitions[t_id].from_state, state.transitions[t_id].to_state]: # funst nicht
                    state.remove_transition(t_id)

            for old_state_id in state.states.keys():
                # try:
                state.remove_state(old_state_id, force=True)
                # except Exception as e:
                #     print "ERROR: ", old_state_id, UNIQUE_DECIDER_STATE_ID, state
                #     raise

        if is_root:
            for outcome_id in state.outcomes.keys():
                if not outcome_id < 0:
                    state.remove_outcome(outcome_id)

            for dp_id in state.input_data_ports.keys():
                state.remove_input_data_port(dp_id)

            # print " \n\n\n ########### start removing output data_ports ", state.output_data_ports.keys(), "\n\n\n"
            for dp_id in state.output_data_ports.keys():
                state.remove_output_data_port(dp_id)

        if isinstance(state, ContainerState):
            for dp_id in state.scoped_variables.keys():
                # print "scoped_variable ", dp_id
                state.remove_scoped_variable(dp_id)

        state.name = stored_state.name
        # state.script = stored_state.script
        # logger.debug("script0: " + stored_state.script.script)
        if isinstance(state, ExecutionState):
            state.script_text = stored_state.script_text

        if is_root:
            for dp_id, dp in stored_state.input_data_ports.iteritems():
                # print "generate input data port", dp_id
                state.add_input_data_port(dp.name, dp.data_type, dp.default_value, dp.data_port_id)
                # print "got input data ports", dp_id, state.input_data_ports.keys()
                assert dp_id in state.input_data_ports.keys()

            # print " \n\n\n ########### start adding output data_ports ", state.output_data_ports.keys(), "\n\n\n"
            for dp_id, dp in stored_state.output_data_ports.iteritems():
                scoped_str = str([])
                if isinstance(state, ContainerState):
                    scoped_str = str(state.scoped_variables.keys())
                # print "\n\n\n ------- ############ generate output data port", dp_id, state.input_data_ports.keys(), \
                #     state.output_data_ports.keys(), scoped_str, "\n\n\n"
                state.add_output_data_port(dp.name, dp.data_type, dp.default_value, dp.data_port_id)
                # print "\n\n\n ------- ############ got output data ports", dp_id, state.output_data_ports.keys(), "\n\n\n"
                assert dp_id in state.output_data_ports.keys()

            for oc_id, oc in stored_state.outcomes.iteritems():
                # print oc_id, state.outcomes, type(oc_id), oc_id < 0, oc_id == 0, oc_id == -1, oc_id == -2
                if not oc_id < 0:
                    # print "add_outcome", oc_id
                    state.add_outcome(oc.name, oc_id)
            # print "\n\n\n++++++++++++++++ ", stored_state.outcomes, state.outcomes, "\n\n\n++++++++++++++++ "
            for oc_id, oc in stored_state.outcomes.iteritems():
                # print oc_id, state.outcomes
                assert oc_id in state.outcomes

        if isinstance(state, ContainerState):
            # logger.debug("UPDATE STATES")
            for dp_id, sv in stored_state.scoped_variables.iteritems():
                state.add_scoped_variable(sv.name, sv.data_type, sv.default_value, sv.data_port_id)

            if UNIQUE_DECIDER_STATE_ID in stored_state.states:
                state.add_state(stored_state.states[UNIQUE_DECIDER_STATE_ID], storage_load=True)

            for new_state in stored_state.states.values():
                # print "++++ new child", new_state
                if not new_state.state_id == UNIQUE_DECIDER_STATE_ID:
                    state.add_state(new_state)
                    # state.states[new_state.state_id].script = new_state.script
                    # logger.debug("script1: " + new_state.script_text)
                    if isinstance(new_state, ExecutionState):
                        state.states[new_state.state_id].script_text = new_state.script_text
                    s_path = state.states[new_state.state_id].get_file_system_path()
                    sm_id = self.state_machine.state_machine_id
                    storage.unmark_path_for_removal_for_sm_id(sm_id, s_path)
                    # print "unmark from removal: ", s_path
                    if isinstance(new_state, ContainerState):
                        def unmark_state(state_, sm_id_):
                            spath = state_.get_file_system_path()
                            storage.unmark_path_for_removal_for_sm_id(sm_id_, spath)
                            # print "unmark from removal: ", spath
                            if isinstance(state_, ContainerState):
                                for child_state in state_.states.values():
                                    unmark_state(child_state, sm_id_)
                                    # do_storage_test(state_)

                        unmark_state(new_state, sm_id)
                        # check if tmp folder otherwise everything is Ok

                        # if is -> do check if exists and write the script if not!!!! TODO
            # for t in state.transitions.values():
            #     # logger.debug(str([t.from_state, t.from_outcome, t.to_state, t.to_outcome]))
            #     if (UNIQUE_DECIDER_STATE_ID == t.from_state or UNIQUE_DECIDER_STATE_ID == t.to_state) and \
            #             UNIQUE_DECIDER_STATE_ID not in state.states:
            #         logger.error("found DECIDER_STATE_TRANSITION_WITHOUT_DECIDER_STATE")
            #         state.remove_transition(t.transition_id)

            if isinstance(state, BarrierConcurrencyState):
                for t_id in state.transitions.keys():
                    state.remove_transition(t_id)

            for t_id, t in stored_state.transitions.iteritems():
                state.add_transition(t.from_state, t.from_outcome, t.to_state, t.to_outcome, t.transition_id)

            # logger.debug("CHECK self TRANSITIONS of unique state%s" % state.transitions.keys())
            for t in state.transitions.values():
                # logger.debug(str([t.from_state, t.from_outcome, t.to_state, t.to_outcome]))
                if UNIQUE_DECIDER_STATE_ID == t.from_state and UNIQUE_DECIDER_STATE_ID == t.to_state:
                    # logger.error("found DECIDER_STATE_SELF_TRANSITION")
                    state.remove_transition(t.transition_id)

            for df_id, df in stored_state.data_flows.iteritems():
                state.add_data_flow(df.from_state, df.from_key, df.to_state, df.to_key, df.data_flow_id)

                # self.before_model.transitions._notify_method_after(state, 'data_flow_change', None, (self.before_model,), {})

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
    def __init__(self, parent_path, state_machine_model, overview):
        ModelMT.__init__(self)
        assert isinstance(overview['model'][0].state_machine, StateMachine)
        Action.__init__(self, parent_path, state_machine_model, overview)

        self.with_print = False
        self.storage_version_for_state_type_change_signal_hook = None

    def set_root_state_to_version(self, state, storage_version):
        import rafcon.gui.state_machine_helper as state_machine_helper
        # logger.debug("\n\n\n\n\n\n\nINSERT STATE: %s  || %s || StateMachineAction\n\n\n\n\n\n\n" % (state.get_path(), state))
        # self.state_machine.root_state = get_state_from_state_tuple(storage_version)
        root_state_version_from_storage = get_state_from_state_tuple(storage_version)
        # logger.debug("\n\n\n\n\n\n\nINSERT STATE META: %s || %s || %s || StateMachineAction\n\n\n\n\n\n\n" % (state.get_path(), state, root_state_version_fom_storage))
        # actual_state_model = self.state_machine_model.get_state_model_by_path(state.get_path())

        if self.with_print:
            print "\n#H# TRY STATE_HELPER ", type(root_state_version_from_storage), \
                isinstance(root_state_version_from_storage, state_machine_helper.HierarchyState), "\n"
        if isinstance(root_state_version_from_storage, state_machine_helper.HierarchyState):
            new_state_class = state_machine_helper.HierarchyState
        elif isinstance(root_state_version_from_storage, state_machine_helper.BarrierConcurrencyState):
            new_state_class = state_machine_helper.BarrierConcurrencyState
        elif isinstance(root_state_version_from_storage, state_machine_helper.PreemptiveConcurrencyState):
            new_state_class = state_machine_helper.PreemptiveConcurrencyState
        else:
            if self.with_print:
                logger.info("SM set_root_state_to_version: with NO type change")
            new_state_class = state_machine_helper.ExecutionState
        logger.debug("DO root version change " + self.type)
                 # logger.debug("DO root version change")
        # observe root state model (type change signal)
        g_sm_editor = self.stop_graphical_viewer()

        if self.type == 'change_root_state_type':
            self.storage_version_for_state_type_change_signal_hook = storage_version
            assert isinstance(self.state_machine_model.root_state.state, State)
            old_root_state_m = self.state_machine_model.root_state
            self.observe_model(self.state_machine_model.root_state)

            self.state_machine.change_root_state_type(new_state_class)
            self.storage_version_for_state_type_change_signal_hook = None
            self.relieve_model(old_root_state_m)
        else:
            new_state = state_machine_helper.create_new_state_from_state_with_type(state, new_state_class)

            self.update_state(new_state, root_state_version_from_storage)

            self.state_machine.root_state = new_state  # root_state_version_fom_storage
            # self.state_machine.root_state.script = storage_version[2]
            insert_state_meta_data(meta_dict=storage_version[3], state_model=self.state_machine_model.root_state)

        self.run_graphical_viewer(g_sm_editor, self.state_machine_model.root_state)

    @ModelMT.observe("state_type_changed_signal", signal=True)
    def hook_for_type_change_operation(self, model, prop_name, info):
        g_sm_editor = self.stop_graphical_viewer()
        msg = info['arg']
        new_state_m = msg.new_state_m
        logger.info("action state-type-change hook for root {}".format(new_state_m))
        storage_version = self.storage_version_for_state_type_change_signal_hook
        root_state_version_from_storage = get_state_from_state_tuple(storage_version)

        self.update_state(new_state_m.state, root_state_version_from_storage)

        insert_state_meta_data(meta_dict=storage_version[3], state_model=new_state_m)  # self.state_machine_model.root_state)

    def redo(self):
        # print "#H# STATE_MACHINE_REDO STARTED"
        state = self.state_machine.root_state

        self.set_root_state_to_version(state, self.after_storage)
        # print "#H# STATE_MACHINE_REDO FINISHED"

    def undo(self):
        """ General Undo, that takes all elements in the parent and
        :return:
        """
        # print "#H# STATE_MACHINE_UNDO STARTED"
        state = self.state_machine.root_state

        self.set_root_state_to_version(state, self.before_storage)
        # print "#H# STATE_MACHINE_UNDO FINISHED"


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
        :return:Redo of adding object action is simply done by adding the object again from the after_storage of the parent state.
        """
        # logger.info("RUN REDO AddObject " + self.before_info['method_name'])

        state = self.get_state_changed()
        storage_version = self.after_storage

        assert state.get_path() == storage_version[4]
        path_of_state = state.get_path()
        storage_version_of_state = get_state_from_state_tuple(storage_version)

        g_sm_editor = self.stop_graphical_viewer()

        if self.added_object_identifier._type in ['InputDataPort', 'OutputDataPort', 'Outcome']:
            [state, storage_version_of_state] = self.correct_reference_state(state,
                                                                             storage_version_of_state,
                                                                             storage_path=storage_version[4])
        list_name = self.type.replace('add_', '') + 's'
        core_obj = getattr(storage_version_of_state, list_name)[self.added_object_identifier._id]
        self.add_core_object_to_state(state, core_obj)

        actual_state_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        insert_state_meta_data(meta_dict=storage_version[3], state_model=actual_state_model, level=1)

        self.run_graphical_viewer(g_sm_editor, actual_state_model)

    def undo(self):

        # find object
        state = self.get_state_changed()
        storage_version = self.after_storage

        assert state.get_path() == storage_version[4]
        path_of_state = state.get_path()
        storage_version_of_state = get_state_from_state_tuple(storage_version)

        g_sm_editor = self.stop_graphical_viewer()

        if self.added_object_identifier._type in ['InputDataPort', 'OutputDataPort', 'Outcome']:
            [state, storage_version_of_state] = self.correct_reference_state(state, storage_version_of_state, storage_version[4])

        list_name = self.type.replace('add_', '') + 's'
        core_obj = getattr(storage_version_of_state, list_name)[self.added_object_identifier._id]
        # logger.info(str(type(core_obj)) + str(core_obj))
        # undo
        self.remove_core_object_from_state(state, core_obj)

        # logger.debug("\n\n\n\n\n\n\nINSERT STATE META: %s %s || Action\n\n\n\n\n\n\n" % (path_of_state, state))
        actual_state_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        insert_state_meta_data(meta_dict=storage_version[3], state_model=actual_state_model, level=1)

        self.run_graphical_viewer(g_sm_editor, actual_state_model)

    def correct_reference_state(self, state, storage_version_of_state, storage_path):

        partial_path = self.added_object_identifier._path.split('/')
        for path_element in storage_path.split('/'):
            partial_path.pop(0)
        for path_element in partial_path:
            storage_version_of_state = storage_version_of_state.states[path_element]
            state = state.states[path_element]
            # logger.info("state is now: {0} {1}".format(state.state_id, storage_version_of_state.state_id))

        return state, storage_version_of_state


class CoreObjectIdentifier:
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
                self._sm_id = core_obj_or_cls.get_sm_for_state().state_machine_id
            else:
                if isinstance(core_obj_or_cls.parent, State):
                    self._path = core_obj_or_cls.parent.get_path()
                    if core_obj_or_cls.parent.get_sm_for_state() is None:
                        logger.warning('state has no state machine -> {0} {1}'.format(core_obj_or_cls.parent.name, core_obj_or_cls.parent.get_path()))
                    else:
                        self._sm_id = core_obj_or_cls.parent.get_sm_for_state().state_machine_id
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

        assert state.get_path() == storage_version[4]
        path_of_state = state.get_path()
        storage_version_of_state = get_state_from_state_tuple(storage_version)

        g_sm_editor = self.stop_graphical_viewer()

        if self.removed_object_identifier._type in ['InputDataPort', 'OutputDataPort', 'Outcome']:
            [state, storage_version_of_state] = self.correct_reference_state(state,
                                                                             storage_version_of_state,
                                                                             storage_path=storage_version[4])
        list_name = self.type.replace('remove_', '') + 's'
        core_obj = getattr(storage_version_of_state, list_name)[self.removed_object_identifier._id]
        # logger.info(str(type(core_obj)) + str(core_obj))
        if self.type not in ['remove_transition', 'remove_data_flow']:
            self.add_core_object_to_state(state, core_obj)

        self.adjust_linkage()

        # logger.debug("\n\n\n\n\n\n\nINSERT STATE META: %s %s || Action\n\n\n\n\n\n\n" % (path_of_state, state))
        actual_state_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        insert_state_meta_data(meta_dict=storage_version[3], state_model=actual_state_model, level=1)

        self.run_graphical_viewer(g_sm_editor, actual_state_model)
        actual_state_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        insert_state_meta_data(meta_dict=storage_version[3], state_model=actual_state_model, level=1)

        self.run_graphical_viewer(g_sm_editor, actual_state_model)

    def redo(self):

        g_sm_editor = self.stop_graphical_viewer()

        if self.type == 'remove_state':
            state = self.state_machine.get_state_by_path(self.parent_path)
            state.remove_state(self.removed_object_identifier._id)
        else:
            state = self.state_machine.get_state_by_path(self.removed_object_identifier._path)
            remove_function = getattr(state, 'remove_' + self.removed_object_identifier._list_name[:-1])
            remove_function(self.removed_object_identifier._id)

        state = self.get_state_changed()
        path_of_state = state.get_path()
        actual_state_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        insert_state_meta_data(meta_dict=self.after_storage[3], state_model=actual_state_model, level=1)

        self.run_graphical_viewer(g_sm_editor, actual_state_model)

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
            for t in state.transitions.itervalues():
                t_dict = {'from_state': t.from_state, 'from_outcome': t.from_outcome,
                          'to_state': t.to_state, 'to_outcome': t.to_outcome, 'transition_id': t.transition_id}
                linkage_dict['internal']['transitions'].append(t_dict)

            for df in state.data_flows.itervalues():
                df_dict = {'from_state': df.from_state, 'from_key': df.from_key,
                           'to_state': df.to_state, 'to_key': df.to_key, 'data_flow_id': df.data_flow_id}
                linkage_dict['internal']['data_flows'].append(df_dict)

        if isinstance(state.parent, State):
            for t in state.parent.transitions.itervalues():
                t_dict = {'from_state': t.from_state, 'from_outcome': t.from_outcome,
                          'to_state': t.to_state, 'to_outcome': t.to_outcome, 'transition_id': t.transition_id}
                linkage_dict['external']['transitions'].append(t_dict)

            for df in state.parent.data_flows.itervalues():
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
        # print "before: \n", self.before_linkage
        # print "after: \n", self.after_linkage
        # print "REMOVED: \n", self.removed_linkage
        # print "ADDED: \n", self.added_linkage

        state = self.state_machine.get_state_by_path(self.instance_path)
        # print state.transitions.keys(), state.data_flows.keys()
        # if isinstance(state.parent, State):
        #     print state.parent.transitions.keys(), state.parent.data_flows.keys()

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


class DataFlowAction(Action):

    possible_method_names = ['modify_origin', 'from_state', 'from_key',
                             'modify_target', 'to_state', 'to_key']
    possible_args = ['from_state', 'from_key', 'to_state', 'to_key']

    def __init__(self, parent_path, state_machine_model, overview):
        # Action.__init__(self, parent_path, state_machine_model, overview)
        self.parent_path = parent_path
        self.action_type = overview['method_name'][-1]
        self.before_overview = overview
        self.state_machine_model = state_machine_model
        self.state_machine = state_machine_model.state_machine

        assert self.action_type in self.possible_method_names
        assert isinstance(self.before_overview['instance'][-1], DataFlow)
        self.object_identifier = CoreObjectIdentifier(self.before_overview['instance'][-1])
        assert self.parent_path == self.object_identifier._path
        self.before_arguments = self.get_set_of_arguments(self.before_overview['instance'][-1])
        self.after_arguments = None

    @staticmethod
    def get_set_of_arguments(df):
        return {'from_state': df.from_state, 'from_key': df.from_key, 'to_state': df.to_state, 'to_key': df.to_key,
                'data_flow_id': df.data_flow_id}

    def set_after(self, overview):
        self.after_overview = overview
        self.after_arguments = self.get_set_of_arguments(self.after_overview['instance'][-1])

    def undo(self):
        # if the data_flow_id would be changed and this considered in the core parent element self.after_argument here would be used
        df = self.state_machine.get_state_by_path(self.parent_path).data_flows[self.before_arguments['data_flow_id']]
        self.set_data_flow_version(df, self.before_arguments)

    def redo(self):
        df = self.state_machine.get_state_by_path(self.parent_path).data_flows[self.before_arguments['data_flow_id']]
        self.set_data_flow_version(df, self.after_arguments)

    def set_data_flow_version(self, df, arguments):
        if self.action_type in self.possible_args:
            exec "df.{0} = arguments['{0}']".format(self.action_type)
        elif self.action_type == 'modify_origin':
            df.modify_origin(from_state=arguments['from_state'], from_key=arguments['from_key'])
        elif self.action_type == 'modify_target':
            df.modify_target(to_state=arguments['to_state'], to_key=arguments['to_key'])
        else:
            assert False


class TransitionAction(Action):

    possible_method_names = ['modify_origin', 'from_state', 'from_outcome',
                             'modify_target', 'to_state', 'to_outcome']
    possible_args = ['from_state', 'from_outcome', 'to_state', 'to_key']

    def __init__(self, parent_path, state_machine_model, overview):
        # Action.__init__(self, parent_path, state_machine_model, overview)
        self.parent_path = parent_path
        self.action_type = overview['method_name'][-1]
        self.before_overview = overview
        self.state_machine_model = state_machine_model
        self.state_machine = state_machine_model.state_machine

        assert self.action_type in self.possible_method_names
        assert isinstance(self.before_overview['instance'][-1], Transition)
        self.object_identifier = CoreObjectIdentifier(self.before_overview['instance'][-1])
        assert self.parent_path == self.object_identifier._path
        self.before_arguments = self.get_set_of_arguments(self.before_overview['instance'][-1])
        self.after_arguments = None

    def get_set_of_arguments(self, t):
        return {'from_state': t.from_state, 'from_outcome': t.from_outcome, 'to_state': t.to_state, 'to_outcome': t.to_outcome,
                'transition_id': t.transition_id}

    def set_after(self, overview):
        self.after_overview = overview
        self.after_arguments = self.get_set_of_arguments(self.after_overview['instance'][-1])

    def undo(self):
        # if the transition_id would be changed and this considered in the core parent element self.after_argument here would be used
        t = self.state_machine.get_state_by_path(self.parent_path).transitions[self.before_arguments['transition_id']]
        self.set_transition_version(t, self.before_arguments)

    def redo(self):
        t = self.state_machine.get_state_by_path(self.parent_path).transitions[self.before_arguments['transition_id']]
        self.set_transition_version(t, self.after_arguments)

    def set_transition_version(self, t, arguments):
        if self.action_type in self.possible_args:
            exec "t.{0} = arguments['{0}']".format(self.action_type)
        elif self.action_type == 'modify_origin':
            t.modify_origin(from_state=arguments['from_state'], from_outcome=arguments['from_outcome'])
        elif self.action_type == 'modify_target':
            t.modify_target(to_state=arguments['to_state'], to_outcome=arguments['to_outcome'])
        else:
            assert False


class DataPortAction(Action):

    possible_method_names = ['name', 'data_type', 'default_value', 'change_data_type']
    possible_args = ['name', 'default_value']

    def __init__(self, parent_path, state_machine_model, overview):
        # Action.__init__(self, parent_path, state_machine_model, overview)
        self.parent_path = parent_path
        self.action_type = overview['method_name'][-1]
        self.before_overview = overview
        self.state_machine = state_machine_model.state_machine

        assert self.action_type in self.possible_method_names
        assert isinstance(self.before_overview['instance'][-1], DataPort)
        self.object_identifier = CoreObjectIdentifier(self.before_overview['instance'][-1])
        assert self.parent_path == self.object_identifier._path
        self.before_arguments = self.get_set_of_arguments(self.before_overview['instance'][-1])
        self.after_arguments = None

    def get_set_of_arguments(self, dp):
        return {'name': dp.name, 'data_type': dp.data_type, 'default_value': dp.default_value,
                'data_port_id': dp.data_port_id}

    def set_after(self, overview):
        self.after_overview = overview
        assert isinstance(self.after_overview['instance'][-1], DataPort)
        self.after_arguments = self.get_set_of_arguments(self.after_overview['instance'][-1])

    def undo(self):
        dp = self.state_machine.get_state_by_path(self.parent_path).get_data_port_by_id(self.before_arguments['data_port_id'])
        self.set_data_port_version(dp, self.before_arguments)

    def redo(self):
        dp = self.state_machine.get_state_by_path(self.parent_path).get_data_port_by_id(self.before_arguments['data_port_id'])
        self.set_data_port_version(dp, self.after_arguments)

    def set_data_port_version(self, dp, arguments):
        if self.action_type in self.possible_args:
            exec "dp.{0} = arguments['{0}']".format(self.action_type)
        elif self.action_type == 'data_type':
            dp.data_type = arguments['data_type']
            dp.default_value = arguments['default_value']
        elif self.action_type == 'change_data_type':
            dp.change_data_type(data_type=arguments['data_type'], default_value=arguments['default_value'])
        else:
            assert False


class ScopedVariableAction(DataPortAction):

    def __init__(self, parent_path, state_machine_model, overview):
        DataPortAction.__init__(self, parent_path, state_machine_model, overview)


class OutcomeAction(Action):

    possible_method_names = ['name']
    possible_args = ['name']

    def __init__(self, parent_path, state_machine_model, overview):
        # Action.__init__(self, parent_path, state_machine_model, overview)
        self.parent_path = parent_path
        self.action_type = overview['method_name'][-1]
        self.before_overview = overview
        self.state_machine = state_machine_model.state_machine

        if not self.action_type in self.possible_method_names:
            logger.error("Outcome Action is not possible with with overview {0}".format(overview))
        assert self.action_type in self.possible_method_names
        assert isinstance(self.before_overview['instance'][-1], Outcome)
        self.object_identifier = CoreObjectIdentifier(self.before_overview['instance'][-1])
        assert self.parent_path == self.object_identifier._path
        self.before_arguments = self.get_set_of_arguments(self.before_overview['instance'][-1])
        self.after_arguments = None

    @staticmethod
    def get_set_of_arguments(oc):
        return {'name': oc.name, 'outcome_id': oc.outcome_id}

    def set_after(self, overview):
        self.after_overview = overview
        assert isinstance(self.after_overview['instance'][-1], Outcome)
        self.after_arguments = self.get_set_of_arguments(self.after_overview['instance'][-1])

    def undo(self):
        # if the outcome_id would be changed and this considered in the core parent element self.after_argument here would be used
        oc = self.state_machine.get_state_by_path(self.parent_path).outcomes[self.before_arguments['outcome_id']]
        self.set_data_port_version(oc, self.before_arguments)

    def redo(self):
        oc = self.state_machine.get_state_by_path(self.parent_path).outcomes[self.before_arguments['outcome_id']]
        self.set_data_port_version(oc, self.after_arguments)

    def set_data_port_version(self, oc, arguments):
        if self.action_type in self.possible_args:
            exec "oc.{0} = arguments['{0}']".format(self.action_type)
        else:
            assert False


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
                             'group_states', 'ungroup_state', 'substitute_state']
    possible_args = ['name', 'description', 'script_text', 'start_state_id',  # ContainerState
                     'library_name', 'library_path', 'version', 'state_copy',  # LibraryState
                     'input_data_port_runtime_values', 'output_data_port_runtime_values',
                     'use_runtime_value_input_data_ports', 'use_runtime_value_output_data_ports',
                     'set_input_runtime_value', 'set_output_runtime_value',
                     'set_use_input_runtime_value', 'set_use_output_runtime_value']
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
        self.parent_path = parent_path
        self.action_type = overview['method_name'][-1]
        self.before_overview = overview
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
        if self.action_type == 'script_text' and isinstance(self.before_overview['args'][-1][1], str):
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

    @staticmethod
    def get_set_of_arguments(s):
        if isinstance(s, ContainerState):
            return {'name': s.name, 'description': s.description, 'state_id': s.state_id, 'start_state_id': s.start_state_id}
        elif isinstance(s, LibraryState):
            return {'name': s.name, 'description': s.description, 'state_id': s.state_id,
                    'library_name': s.library_name, 'library_path': s.library_path, 'version': s.version, # LibraryState
                    'state_copy': s.state_copy,
                    'input_data_port_runtime_values': copy.deepcopy(s.input_data_port_runtime_values),
                    'output_data_port_runtime_values': copy.deepcopy(s.output_data_port_runtime_values),
                    'use_runtime_value_input_data_ports': copy.deepcopy(s.use_runtime_value_input_data_ports),
                    'use_runtime_value_output_data_ports': copy.deepcopy(s.use_runtime_value_output_data_ports)}
        else:
            return {'name': s.name, 'description': s.description, 'script_text': s.script_text, 'state_id': s.state_id}

    def set_after(self, overview):
        Action.set_after(self, overview)
        self.after_overview = overview
        assert isinstance(self.after_overview['instance'][-1], State)
        self.after_arguments = self.get_set_of_arguments(self.after_overview['instance'][-1])

    def undo(self):
        if self.action_type in ['parent', 'outcomes', 'input_data_ports', 'output_data_ports']:
            Action.undo(self)
        elif self.action_type in ['states', 'scoped_variables', 'data_flows', 'transitions', 'change_state_type',
                                  'group_states', 'ungroup_state', 'substitute_state']:
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
                                  'group_states', 'ungroup_state', 'substitute_state']:
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
            exec "s.{0} = copy.deepcopy(arguments['{0}'])".format(self.substitute_dict.get(self.action_type,
                                                                                           self.action_type))
        else:
            assert False


class Group(Action):
    def __init__(self, *args, **kwargs):
        Action.__init__(self, *args, **kwargs)


class UnGroup(Action):
    def __init__(self, *args, **kwargs):
        Action.__init__(self, *args, **kwargs)
