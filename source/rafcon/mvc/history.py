"""
The module provides classes to document, undo or redo state machine edit steps.

The History-Class provides the observation functionalities to register and identify all core or mvc (graphical) edit
actions that are a actual change to the state machine. Those changes are stored as Action-Objects in the HistoryChanges-Class.

The HistoryChanges-Class provides the functionalities to organize and access all actions of the edit process.
Hereby the branching of the edit process is stored and should be accessible, too.

The Action-Class provides the a general redo or undo functionality for any action
as long as the the class object was initialized with consistent arguments.
This general Action (one procedure for all possible edition) procedure is expansive and complex dues it is aimed
to define specific Action-*-Classes for simple/specific edit actions.
"""
import copy
import sys
import traceback
import datetime

from gtkmvc import ModelMT, Observable
# import yaml
import json
from rafcon.utils.json_utils import JSONObjectDecoder, JSONObjectEncoder

from rafcon.utils import log

from rafcon.statemachine.scope import ScopedData, ScopedVariable
from rafcon.statemachine.outcome import Outcome
from rafcon.statemachine.data_flow import DataFlow
from rafcon.statemachine.transition import Transition
from rafcon.statemachine.script import Script
from rafcon.statemachine.states.state import State
from rafcon.statemachine.data_port import DataPort
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState

from rafcon.statemachine.data_port import InputDataPort, OutputDataPort
from rafcon.statemachine.global_variable_manager import GlobalVariableManager
from rafcon.statemachine.library_manager import LibraryManager
from rafcon.statemachine.state_machine import StateMachine

from rafcon.mvc.models.container_state import ContainerState, ContainerStateModel

import rafcon.mvc.statemachine_helper

import rafcon.mvc.singleton as mvc_singleton

from rafcon.statemachine.enums import UNIQUE_DECIDER_STATE_ID

logger = log.get_logger(__name__)

core_object_list = [Transition, DataFlow, Outcome, InputDataPort, OutputDataPort, ScopedData, ScopedVariable, Script,
                    GlobalVariableManager, LibraryManager, StateMachine,
                    ExecutionState, HierarchyState, BarrierConcurrencyState, PreemptiveConcurrencyState]

HISTORY_DEBUG_LOG_FILE = '/tmp/test_file.txt'


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

    :param rafcon.statemachine.states.state.State state: The state that should be stored
    :return: state_tuple tuple
    """
    # state_str = yaml.dump(state)
    state_str = json.dumps(state, cls=JSONObjectEncoder, indent=4, check_circular=False, sort_keys=True)

    # print "++++++++++", state
    state_tuples_dict = {}
    if hasattr(state, "states"):
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
        script = Script(state=state)
    else:
        script_content = state.script.script
        script = state.script
    state_tuple = (state_str, state_tuples_dict, script, state_meta_dict, state.get_path(), script_content)

    return state_tuple


class NotificationOverview(dict):
    # TODO generalize and include into utils

    def __init__(self, info, with_prints=False):

        self.info = info
        self.__type = 'before'
        if 'after' in info:
            self.__type = 'after'
        self.with_prints = with_prints

        s, new_overview = self.get_nice_info_dict_string(info)
        self.__description = str(datetime.datetime.now()) + "\n" + s
        self.new_overview = new_overview
        self.__overview = self.parent_state_of_notification_source(info)
        dict.__init__(self, self.__overview)
        if self.with_prints:
            self.print_overview()
            print str(self)

    def __str__(self):
        return self.__description

    def __setitem__(self, key, value):
        if key in ['info', 'model', 'prop_name', 'instance', 'method_name', 'level']:
            dict.__setitem__(self, key, value)

    def update(self, E=None, **F):
        if E is not None:
            for key in E.keys:
                if key not in ['info', 'model', 'prop_name', 'instance', 'method_name', 'level']:
                    E.pop(key)
            dict.update(self, E)

    def print_overview(self, overview=None):
        if overview is None:
            overview = self.__overview
        info_print = ''
        info_count = 0
        for elem in overview['info']:
            info_print += "\ninfo %s: %s" % (info_count, str(elem))
            info_count += 1

        print info_print
        print "model: ", overview['model']
        print "prop_: ", overview['prop_name']
        print "insta: ", overview['instance']
        print "metho: ", overview['method_name']
        print "level: ", overview['level']
        print "prop-: ", overview['prop_name'][-1]

    def get_all(self):
        return self.__overview

    def check_overview(self):
        overview = self.__overview
        if overview['prop_name'][-1] == 'state':
            # print "path: ", overview['instance'][-1].get_path(), "\npath: ", overview['model'][-1].state.get_path()
            assert overview['instance'][-1].get_path() == overview['model'][-1].state.get_path()
        else:
            if overview['model'][-1].parent:
                if not isinstance(overview['model'][-1].parent.state, State):  # is root_state
                    overview['model'][-1].state.get_path()
                    if self.with_prints:
                        print "Path_root: ", overview['model'][-1].state.get_path()
                else:
                    overview['model'][-1].parent.state.get_path()
                    if self.with_prints:
                        print "Path: ", overview['model'][-2].state.get_path(), "\nPath: ", \
                            overview['model'][-1].parent.state.get_path()
                    assert overview['model'][-2].state.get_path() == \
                           overview['model'][-1].parent.state.get_path().split('/')[0]

    def get_nice_info_dict_string(self, info, level='\t', overview=None):
        """ Inserts all elements of a notification info-dictionary of gtkmvc into one string and indicates levels of calls definded by 'kwargs'
        """
        overview_was_none = False
        if overview is None:
            overview_was_none = True
            overview = dict({'model': [], 'prop_name': [], 'instance': [], 'method_name': [], 'args': [], 'kwargs': []})
            overview['others'] = []
            overview['info'] = []
            if 'before' in info:
                overview['type'] = 'before'
            else:
                overview['type'] = 'after'
                overview['result'] = []

        if ('after' in info or 'before' in info) and 'model' in info:
            if 'before' in info:
                s = "{0}'before': {1}".format(level, info['before'])
            else:
                s = "{0}'after': {1}".format(level, info['after'])
        else:
            return str(info)
        overview['info'].append(info)
        # model
        s += "\n{0}'model': {1}".format(level, info['model'])
        overview['model'].append(info['model'])
        # prop_name
        s += "\n{0}'prop_name': {1}".format(level, info['prop_name'])
        overview['prop_name'].append(info['prop_name'])
        # instance
        s += "\n{0}'instance': {1}".format(level, info['instance'])
        overview['instance'].append(info['instance'])
        # method_name
        s += "\n{0}'method_name': {1}".format(level, info['method_name'])
        overview['method_name'].append(info['method_name'])
        # args
        s += "\n{0}'args': {1}".format(level, info['args'])
        overview['args'].append(info['args'])

        overview['kwargs'].append(info['kwargs'])
        if overview['type'] == 'after':
            overview['result'].append(info['result'])
        # kwargs
        s += "\n{0}'kwargs': {1}".format(level, self.get_nice_info_dict_string(info['kwargs'], level + "\t", overview))
        if overview['type'] == 'after':
            s += "\n{0}'result': {1}".format(level, info['result'])
        # additional elements not created by gtkmvc or common function calls
        overview['others'].append({})
        for key, value in info.items():
            if key in ['before', 'after', 'model', 'prop_name', 'instance', 'method_name', 'args', 'kwargs', 'result']:
                pass
            else:
                s += "\n{0}'{2}': {1}".format(level, info[key], key)
                overview['others'][len(overview['others'])-1][key] = info[key]

        if overview_was_none:
            return s, overview
        else:
            return s

    def parent_state_of_notification_source(self, info):

        if self.with_prints:
            print "----- xxxxxxx %s \n%s\n%s\n%s\n" % (self.__type, info['model'], info['prop_name'], info)

        def set_dict(info, d):
            d['model'].append(info['model'])
            d['prop_name'].append(info['prop_name'])
            d['instance'].append(info['instance'])
            d['method_name'].append(info['method_name'])
            if self.with_prints:
                print "set"

        def find_parent(info, elem):
            elem['info'].append(info)
            if 'kwargs' in info and info['kwargs']:
                if self.with_prints:
                    print 'kwargs'
                elem['level'].append('kwargs')
                set_dict(info, elem)
                if 'method_name' in info['kwargs'] and 'instance' in info['kwargs']:
                    find_parent(info['kwargs'], elem)
            elif 'info' in info and info['info']:
                if self.with_prints:
                    print 'info'
                elem['level'].append('info')
                set_dict(info, elem)
                find_parent(info['info'], elem)
                assert len(info['info']) < 2
            elif 'info' in info:
                set_dict(info, elem)
            elif 'kwargs' in info:
                set_dict(info, elem)
            else:
                if self.with_prints:
                    print 'assert'
                assert True
            return elem

        overview = find_parent(info, {'model': [], 'prop_name': [], 'instance': [], 'method_name': [], 'level': [],
                                      'info': []})

        if self.with_prints:
            self.print_overview(overview)
        return overview


def get_state_from_state_tuple(state_tuple):
    # print "++++ new state", state_tuple

    # Transitions and data flows are not added, as also states are not added
    # We have to wait until the child states are loaded, before adding transitions and data flows, as otherwise the
    # validity checks for transitions and data flows would fail
    # state_info = yaml.load(state_tuple[0])
    state_info = json.loads(state_tuple[0], cls=JSONObjectDecoder)
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

    state.script = state_tuple[2]
    state.script.script = state_tuple[5]
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
            if hasattr(state, "states"):
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


def get_state_element_meta(state_model, with_parent_linkage=True, with_prints=False):
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
                    meta_dict['related_parent_transitions'][transition.transition_id] = copy.deepcopy(transition_m.meta)
            for data_flow_m in state_model.parent.data_flows:
                data_flow = data_flow_m.data_flow
                if data_flow.from_state == state_id or data_flow.to_state == state_id:
                    meta_dict['related_parent_data_flows'][data_flow.data_flow_id] = copy.deepcopy(data_flow_m.meta)

    if with_prints:
        print "STORE META for STATE: ", state_model.state.state_id, state_model.state.name
    meta_dict['is_start'] = state_model.is_start
    for elem in state_model.outcomes:
        meta_dict['outcomes'][elem.outcome.outcome_id] = copy.deepcopy(elem.meta)
        if with_prints:
            print "outcome: ", elem.outcome.outcome_id, elem.parent.state.outcomes.keys(), meta_dict['outcomes'].keys()
    for elem in state_model.input_data_ports:
        meta_dict['input_data_ports'][elem.data_port.data_port_id] = copy.deepcopy(elem.meta)
        if with_prints:
            print "input: ", elem.data_port.data_port_id, elem.parent.state.input_data_ports.keys(), meta_dict[
                'input_data_ports'].keys()
    for elem in state_model.output_data_ports:
        meta_dict['output_data_ports'][elem.data_port.data_port_id] = copy.deepcopy(elem.meta)
        if with_prints:
            print "output: ", elem.data_port.data_port_id, elem.parent.state.output_data_ports.keys(), meta_dict[
                'output_data_ports'].keys()

    meta_dict['state'] = copy.deepcopy(state_model.meta)
    if isinstance(state_model, ContainerStateModel):
        for state_id, state_m in state_model.states.iteritems():
            meta_dict['states'][state_m.state.state_id] = get_state_element_meta(state_m, with_parent_linkage)
            if with_prints:
                print "FINISHED STORE META for STATE: ", state_id, meta_dict[
                    'states'].keys(), state_model.state.state_id
        for elem in state_model.transitions:
            meta_dict['transitions'][elem.transition.transition_id] = copy.deepcopy(elem.meta)
            if with_prints:
                print "transition: ", elem.transition.transition_id, elem.parent.state.transitions.keys(), meta_dict[
                    'transitions'].keys(), elem.parent.state.state_id
        for elem in state_model.data_flows:
            meta_dict['data_flows'][elem.data_flow.data_flow_id] = copy.deepcopy(elem.meta)
            if with_prints:
                print "data_flow: ", elem.data_flow.data_flow_id, elem.parent.state.data_flows.keys(), meta_dict[
                    'data_flows'].keys()
        for elem in state_model.scoped_variables:
            meta_dict['scoped_variables'][elem.scoped_variable.data_port_id] = copy.deepcopy(elem.meta)
            if with_prints:
                print "scoped_variable: ", elem.scoped_variable.data_port_id, elem.parent.state.scoped_variables.keys(), \
                meta_dict['scoped_variables'].keys()
    return meta_dict


def insert_state_meta_data(meta_dict, state_model, with_parent_linkage=True, with_prints=False):
    # meta_dict = {'state': state_model.meta, 'data_flows': {}, 'transitions': {}, 'outcomes': {},
    #              'input_data_ports': {}, 'output_data_ports': {}, 'scoped_variables': {}}

    # if with_parent_linkage:
    #     with_parent_linkage = False
    #     if state_model.parent is not None:  # not root_state
    #         state_id = state_model.state.state_id
    #         for t_id in meta_dict['related_parent_transitions'].keys():
    #             transition_m = state_model.parent.transitions[t_id]
    #             if transition_m.transition.from_state == state_id or transition_m.transition.to_state == state_id:
    #                 transition_m.meta = meta_dict['related_parent_transitions'][t_id]
    #         for df_id in meta_dict['related_parent_data_flows'].keys():
    #             data_flow_m = state_model.parent.data_flows[df_id]
    #             if data_flow_m.data_flow.from_state == state_id or data_flow_m.data_flow.to_state == state_id:
    #                 data_flow_m.meta = meta_dict['related_parent_data_flows'][df_id]

    state_model.meta = copy.deepcopy(meta_dict['state'])
    if with_prints:
        print "INSERT META for STATE: ", state_model.state.state_id, state_model.state.name

    for elem in state_model.outcomes:
        if with_prints:
            print "outcome: ", elem.outcome.outcome_id, meta_dict['outcomes'].keys()
        assert elem.outcome.outcome_id in meta_dict['outcomes']
        elem.meta = copy.deepcopy(meta_dict['outcomes'][elem.outcome.outcome_id])
    for elem in state_model.input_data_ports:
        if with_prints:
            print "input: ", elem.data_port.data_port_id, meta_dict['input_data_ports'].keys()
        assert elem.data_port.data_port_id in meta_dict['input_data_ports']
        elem.meta = copy.deepcopy(meta_dict['input_data_ports'][elem.data_port.data_port_id])

    for elem in state_model.output_data_ports:
        if with_prints:
            print "output: ", elem.data_port.data_port_id, meta_dict['output_data_ports'].keys()
        assert elem.data_port.data_port_id in meta_dict['output_data_ports']
        elem.meta = copy.deepcopy(meta_dict['output_data_ports'][elem.data_port.data_port_id])
    if hasattr(state_model, 'states'):
        for state_id, state_m in state_model.states.iteritems():
            if with_prints:
                print "FIN: ", state_id, state_m.state.state_id, meta_dict['states'].keys(), state_model.state.state_id
            if state_m.state.state_id != UNIQUE_DECIDER_STATE_ID:
                insert_state_meta_data(meta_dict['states'][state_m.state.state_id], state_m, with_parent_linkage)
            if with_prints:
                print "FINISHED META for STATE: ", state_m.state.state_id
        for elem in state_model.transitions:
            if with_prints:
                print "transition: ", elem.transition.transition_id, meta_dict[
                    'transitions'].keys(), elem.parent.state.transitions.keys(), elem.parent.state.state_id
            # assert elem.transition.transition_id in meta_dict['transitions']
            if elem.transition.transition_id in meta_dict['transitions']:
                elem.meta = copy.deepcopy(meta_dict['transitions'][elem.transition.transition_id])
            else:
                logger.info("Storage Dict seems to miss Meta-Data of Transition in State: %s %s for transition: %s" %
                            (state_model.state.state_id, state_model.state.name, elem.transition))
        for elem in state_model.data_flows:
            if with_prints:
                print "data_flow: ", elem.data_flow.data_flow_id, meta_dict['data_flows'].keys()
            assert elem.data_flow.data_flow_id in meta_dict['data_flows']
            elem.meta = copy.deepcopy(meta_dict['data_flows'][elem.data_flow.data_flow_id])
        for elem in state_model.scoped_variables:
            if with_prints:
                print "scoped: ", elem.scoped_variable.data_port_id, meta_dict['scoped_variables'].keys()
            assert elem.scoped_variable.data_port_id in meta_dict['scoped_variables']
            elem.meta = copy.deepcopy(meta_dict['scoped_variables'][elem.scoped_variable.data_port_id])
    state_model.is_start = copy.deepcopy(meta_dict['is_start'])
    if state_model.is_start and not state_model.state.is_root_state:  # TODO not nice that model stuff does things in core
        if not (isinstance(state_model.parent.state, BarrierConcurrencyState) or
                    isinstance(state_model.parent.state, PreemptiveConcurrencyState)):
            # logger.debug("set start_state_id %s" % state_model.parent.state)
            # state_model.parent.state.start_state_id = state_model.state.state_id
            pass
        else:
            state_model.is_start = False


class ActionDummy:
    def __init__(self):
        pass

    def set_after(self, overview):
        pass

    def undo(self):
        pass

    def redo(self):
        pass


class Action:
    def __init__(self, parent_path, state_machine_model, overview):

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
                logger.warning("statemachine could not get state by path -> take root_state for undo")
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
        if g_sm_editor and not hasattr(g_sm_editor, 'suspend_drawing'):
            g_sm_editor = False
        if g_sm_editor:
            g_sm_editor.suspend_drawing = True

        return g_sm_editor

    @staticmethod
    def run_graphical_viewer(g_sm_editor):
        """ Enables and re-initiate graphical viewer's drawing process.
        :param g_sm_editor: graphical state machine editor
        """
        if g_sm_editor:
            g_sm_editor.suspend_drawing = False
            g_sm_editor._redraw()  # is used to secure update of graphical editor # TODO remove if not necessary anymore (private)

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
        # print state.get_path(), '\n', storage_version[4]
        assert state.get_path() == storage_version[4]
        # print self.parent_path, self.parent_path.split('/'), len(self.parent_path.split('/'))
        path_of_state = state.get_path()
        storage_version_of_state = get_state_from_state_tuple(storage_version)

        assert storage_version_of_state

        g_sm_editor = self.stop_graphical_viewer()

        self.update_state(state, storage_version_of_state)

        # logger.debug("\n\n\n\n\n\n\nINSERT STATE META: %s %s || Action\n\n\n\n\n\n\n" % (path_of_state, state))
        actual_state_model = self.state_machine_model.get_state_model_by_path(path_of_state)
        insert_state_meta_data(meta_dict=storage_version[3], state_model=actual_state_model)

        self.run_graphical_viewer(g_sm_editor)

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
                try:
                    state.remove_state(old_state_id, force=True)
                except Exception as e:
                    print "ERROR: ", old_state_id, UNIQUE_DECIDER_STATE_ID, state
                    raise e

        if is_root:
            for outcome_id in state.outcomes.keys():
                if not outcome_id < 0:
                    state.remove_outcome(outcome_id)

            for dp_id in state.input_data_ports.keys():
                state.remove_input_data_port(dp_id)

            # print " \n\n\n ########### start removing output data_ports ", state.output_data_ports.keys(), "\n\n\n"
            for dp_id in state.output_data_ports.keys():
                state.remove_output_data_port(dp_id)

        if hasattr(state, 'states'):
            for dp_id in state.scoped_variables.keys():
                # print "scoped_variable ", dp_id
                state.remove_scoped_variable(dp_id)

        state.name = stored_state.name
        state.script = stored_state.script
        # # logger.debug("script0: " + stored_state.script.script)
        # state.set_script_text(stored_state.script.script)

        if is_root:
            for dp_id, dp in stored_state.input_data_ports.iteritems():
                # print "generate input data port", dp_id
                state.add_input_data_port(dp.name, dp.data_type, dp.default_value, dp.data_port_id)
                # print "got input data ports", dp_id, state.input_data_ports.keys()
                assert dp_id in state.input_data_ports.keys()

            # print " \n\n\n ########### start adding output data_ports ", state.output_data_ports.keys(), "\n\n\n"
            for dp_id, dp in stored_state.output_data_ports.iteritems():
                scoped_str = str([])
                if hasattr(state, "scoped_variables"):
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

        if hasattr(state, 'states'):
            # logger.debug("UPDATE STATES")
            for dp_id, sv in stored_state.scoped_variables.iteritems():
                state.add_scoped_variable(sv.name, sv.data_type, sv.default_value, sv.data_port_id)

            if UNIQUE_DECIDER_STATE_ID in stored_state.states:
                state.add_state(stored_state.states[UNIQUE_DECIDER_STATE_ID], storage_load=True)

            for new_state in stored_state.states.values():
                # print "++++ new child", new_state
                if not new_state.state_id == UNIQUE_DECIDER_STATE_ID:
                    state.add_state(new_state)
                    state.states[new_state.state_id].script = new_state.script
                    # # logger.debug("script1: " + new_state.script.script)
                    # state.states[new_state.state_id].set_script_text(new_state.script.script)
                    s_path = state.states[new_state.state_id].get_file_system_path()
                    sm_id = self.state_machine.state_machine_id
                    rafcon.statemachine.singleton.global_storage.unmark_path_for_removal_for_sm_id(sm_id, s_path)
                    # print "unmark from removal: ", s_path
                    if hasattr(new_state, 'states'):
                        def unmark_state(state_, sm_id_):
                            spath = state_.get_file_system_path()
                            rafcon.statemachine.singleton.global_storage.unmark_path_for_removal_for_sm_id(sm_id_,
                                                                                                           spath)
                            # print "unmark from removal: ", spath
                            if hasattr(state_, 'states'):
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

            if not isinstance(state, BarrierConcurrencyState):
                for t_id, t in stored_state.transitions.iteritems():
                    # print "\n\n\n++++++++++++++++ ", stored_state.outcomes, state.outcomes, "\n\n\n++++++++++++++++ "
                    # print "### transitions to delete ", [t.from_state, t.to_state], t
                    if UNIQUE_DECIDER_STATE_ID not in [t.from_state, t.to_state]:
                        state.add_transition(t.from_state, t.from_outcome, t.to_state, t.to_outcome, t.transition_id)
            # logger.debug("CHECK TRANSITIONS %s" % state.transitions.keys())
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


class StateMachineAction(Action):
    def __init__(self, parent_path, state_machine_model, overview):
        assert isinstance(overview['model'][0].state_machine, StateMachine)
        Action.__init__(self, parent_path, state_machine_model, overview)

        self.with_print = False

    def set_root_state_to_version(self, state, storage_version):
        import rafcon.mvc.statemachine_helper as statemachine_helper
        # logger.debug("\n\n\n\n\n\n\nINSERT STATE: %s  || %s || StateMachineAction\n\n\n\n\n\n\n" % (state.get_path(), state))
        # self.state_machine.root_state = get_state_from_state_tuple(storage_version)
        root_state_version_from_storage = get_state_from_state_tuple(storage_version)
        # logger.debug("\n\n\n\n\n\n\nINSERT STATE META: %s || %s || %s || StateMachineAction\n\n\n\n\n\n\n" % (state.get_path(), state, root_state_version_fom_storage))
        # actual_state_model = self.state_machine_model.get_state_model_by_path(state.get_path())

        if self.with_print:
            print "\n#H# TRY STATE_HELPER ", type(root_state_version_from_storage), \
                isinstance(root_state_version_from_storage, statemachine_helper.HierarchyState), "\n"
        if isinstance(root_state_version_from_storage, statemachine_helper.HierarchyState):
            new_state_class = statemachine_helper.HierarchyState
        elif isinstance(root_state_version_from_storage, statemachine_helper.BarrierConcurrencyState):
            new_state_class = statemachine_helper.BarrierConcurrencyState
        elif isinstance(root_state_version_from_storage, statemachine_helper.PreemptiveConcurrencyState):
            new_state_class = statemachine_helper.PreemptiveConcurrencyState
        else:
            if self.with_print:
                logger.info("SM set_root_state_to_version: with NO type change")
            new_state_class = statemachine_helper.ExecutionState
        # logger.debug("DO root version change")
        new_state = statemachine_helper.create_new_state_from_state_with_type(state, new_state_class)

        g_sm_editor = self.stop_graphical_viewer()

        self.update_state(new_state, root_state_version_from_storage)

        if self.with_print:
            logger.info("SM set_root_state_to_version: insert new root state")
        self.state_machine.root_state = new_state  # root_state_version_fom_storage
        self.state_machine.root_state.script = storage_version[2]
        if self.with_print:
            logger.info("SM set_root_state_to_version: insert old meta data")
        insert_state_meta_data(meta_dict=storage_version[3], state_model=self.state_machine_model.root_state)
        if self.with_print:
            logger.info("SM set_root_state_to_version: FINISHED")

        self.run_graphical_viewer(g_sm_editor)

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
        new_overview = overview.new_overview
        # logger.info("add_object \n" + str(self.after_info))
        # get new object from respective list and create identifier
        list_name = new_overview['method_name'][-1].replace('add_', '') + 's'
        new_object = getattr(new_overview['args'][-1][0], list_name)[new_overview['result'][-1]]
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
        insert_state_meta_data(meta_dict=storage_version[3], state_model=actual_state_model)

        self.run_graphical_viewer(g_sm_editor)

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
        insert_state_meta_data(meta_dict=storage_version[3], state_model=actual_state_model)

        self.run_graphical_viewer(g_sm_editor)

    def correct_reference_state(self, state, storage_version_of_state, storage_path):

        partial_path = self.added_object_identifier._path.split('/')
        for path_element in storage_path.split('/'):
            logger.info("pop: " + partial_path.pop(0))
        for path_element in partial_path:
            storage_version_of_state = storage_version_of_state.states[path_element]
            state = state.states[path_element]
            logger.info("state is now: {0} {1}".format(state.state_id, storage_version_of_state.state_id))

        return state, storage_version_of_state


class CoreObjectIdentifier:
    # TODO generalize and include into utils

    type_related_list_name_dict = {InputDataPort.__name__: 'input_data_ports',
                                   OutputDataPort.__name__: 'output_data_ports',
                                   ScopedVariable.__name__: 'scoped_variables',
                                   DataFlow.__name__: 'data_flows',
                                   Outcome.__name__: 'outcomes',
                                   Transition.__name__: 'transitions',
                                   State.__name__: 'states'}

    def __init__(self, core_obj_or_cls):
        assert type(core_obj_or_cls) in core_object_list or core_obj_or_cls in core_object_list
        print core_obj_or_cls
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
            if self._type in ['ExecutionState', 'HierarchyState', 'BarrierConcurrencyState', 'PreemptiveConcurrencyState']:
                self._path = core_obj_or_cls.get_path()
                self._id = core_obj_or_cls.state_id
                self._sm_id = core_obj_or_cls.get_sm_for_state().state_machine_id
            else:
                if isinstance(core_obj_or_cls.parent, State):
                    self._path = core_obj_or_cls.parent.get_path()
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
        new_overview = self.before_overview.new_overview
        # get new object from respective list and create identifier
        object_type_name = new_overview['method_name'][-1].replace('remove_', '')
        list_name = object_type_name + 's'
        if len(new_overview['args'][-1]) < 2:
            object_id = new_overview['kwargs'][-1][object_type_name + '_id']
        else:
            object_id = new_overview['args'][-1][1]
        new_object = getattr(new_overview['args'][-1][0], list_name)[object_id]
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
        insert_state_meta_data(meta_dict=storage_version[3], state_model=actual_state_model)

        self.run_graphical_viewer(g_sm_editor)

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
        insert_state_meta_data(meta_dict=self.after_storage[3], state_model=actual_state_model)

        self.run_graphical_viewer(g_sm_editor)

    def correct_reference_state(self, state, storage_version_of_state, storage_path):

        partial_path = self.removed_object_identifier._path.split('/')
        for path_element in storage_path.split('/'):
            logger.info("pop: " + partial_path.pop(0))
        for path_element in partial_path:
            storage_version_of_state = storage_version_of_state.states[path_element]
            state = state.states[path_element]
            logger.info("state is now: {0} {1}".format(state.state_id, storage_version_of_state.state_id))

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

    @ staticmethod
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

    not_possible_method_names = ['state_execution_status',  # observed but should be ignored
                                 'input_data', 'output_data', 'concurrency_queue','state_id',  # any not observed
                                 'final_outcome', 'preempted', 'active', 'is_root_state',  # any not observed
                                 'scoped_data', 'v_checker']
    possible_method_names = ['parent',  # will be ignored
                             'name', 'description',  # State
                             'outcomes', 'input_data_ports', 'output_data_ports',  # State
                             'states', 'scoped_variables', 'data_flows', 'transitions', 'start_state_id', # ContainerState
                             'change_state_type']
    possible_args = ['name', 'description',
                     'start_state_id']  # ContainerState

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

        assert self.action_type in self.possible_method_names
        assert isinstance(self.before_overview['instance'][-1], State)
        self.object_identifier = CoreObjectIdentifier(self.before_overview['instance'][-1])
        if overview['method_name'][-1] in ['outcomes', 'input_data_ports', 'output_data_ports']:
            assert self.parent_path == CoreObjectIdentifier(self.before_overview['instance'][-1].parent)._path
        else:
            assert self.parent_path == self.object_identifier._path
        self.before_arguments = self.get_set_of_arguments(self.before_overview['instance'][-1])
        self.after_arguments = None

    @staticmethod
    def get_set_of_arguments(s):
        if isinstance(s, ContainerState):
            return {'name': s.name, 'description': s.description, 'state_id': s.state_id, 'start_state_id': s.start_state_id}
        else:
            return {'name': s.name, 'description': s.description, 'state_id': s.state_id}

    def set_after(self, overview):
        Action.set_after(self, overview)
        self.after_overview = overview
        assert isinstance(self.after_overview['instance'][-1], State)
        self.after_arguments = self.get_set_of_arguments(self.after_overview['instance'][-1])

    def undo(self):
        if self.action_type in ['parent', 'outcomes', 'input_data_ports', 'output_data_ports']:
            Action.undo(self)
        elif self.action_type in ['states', 'scoped_variables', 'data_flows', 'transitions', 'change_state_type']:
            Action.undo(self)
        elif self.action_type in self.possible_args:
            s = self.state_machine.get_state_by_path(self.parent_path)
            self.set_data_port_version(s, self.before_arguments)
        else:
            assert False

    def redo(self):
        if self.action_type in ['outcomes', 'input_data_ports', 'output_data_ports']:
            Action.redo(self)
        elif self.action_type in ['states', 'scoped_variables', 'data_flows', 'transitions', 'change_state_type']:
            Action.redo(self)
        elif self.action_type in self.possible_args:
            s = self.state_machine.get_state_by_path(self.parent_path)
            self.set_data_port_version(s, self.after_arguments)
        else:
            assert False

    def set_data_port_version(self, s, arguments):
        if self.action_type in self.possible_args:
            exec "s.{0} = arguments['{0}']".format(self.action_type)
        else:
            assert False


class Group(Action):
    def __init__(self, *args, **kwargs):
        Action.__init__(self, *args, **kwargs)


class UnGroup(Action):
    def __init__(self, *args, **kwargs):
        Action.__init__(self, *args, **kwargs)


class HistoryTreeElement:
    def __init__(self, prev_id, action, next_id, old_next_ids):
        self.prev_id = None
        if prev_id is not None:
            self.prev_id = int(prev_id)
        self.action = action
        self.next_id = next_id
        self.old_next_ids = old_next_ids

    def __str__(self):
        return "prev_id: %s next_id: %s and other next_ids: %s" % (self.prev_id, self.next_id, self.old_next_ids)


class History(ModelMT):
    state_machine_model = None
    changes = None

    __observables__ = ("changes",)

    def __init__(self, state_machine_model):
        ModelMT.__init__(self)

        # assert isinstance(state_machine_model, StateMachineModel)
        self.state_machine_model = state_machine_model
        self.__state_machine_id = state_machine_model.state_machine.state_machine_id
        self.tmp_meta_storage = get_state_element_meta(self.state_machine_model.root_state)

        self.observe_model(self.state_machine_model)
        self.observe_model(self.state_machine_model.root_state)
        self.__buffered_root_state_model = self.state_machine_model.root_state

        self.actual_action = None
        self.actual_root_state_action = None
        self.locked = False
        self.busy = False
        self.count_before = 0

        self.changes = ChangeHistory()

        self.fake = False

        self.refactored_history = True
        self.with_prints = False
        self.with_debug_logs = False

    def get_state_element_meta_from_tmp_storage(self, state_path):
        path_elements = state_path.split('/')
        path_elements.pop(0)
        # print path_elements
        act_state_elements_meta = self.tmp_meta_storage
        for path_elem in path_elements:
            act_state_elements_meta = act_state_elements_meta['states'][path_elem]
        # print act_state_elements_meta

    def recover_specific_version(self, pointer_on_version_to_recover):
        """ Recovers a specific version of the all_time_history element by doing several undos and redos.

        :param pointer_on_version_to_recover: the id of the list element which is to recover
        :return:
        """
        logger.info("recover version %s of trail state machine edit history" % pointer_on_version_to_recover)
        # search for traceable path -> list of action to undo and list of action to redo
        actual_version_pointer = self.changes.trail_history[self.changes.trail_pointer].version_id
        # logger.debug("actual version_id %s and goal version_id %s" %
        #              (self.changes.all_time_history[actual_version_pointer].action.version_id, pointer_on_version_to_recover))
        undo_redo_list = []
        # backward
        # logger.debug("actual version id %s " % self.changes.trail_history[self.changes.trail_pointer].version_id)
        while not self.changes.trail_pointer == -1 and not int(pointer_on_version_to_recover) == int(
                actual_version_pointer):
            undo_redo_list.append((actual_version_pointer, 'undo'))
            # logger.info("%s" % self.changes.all_time_history[actual_version_pointer])
            # logger.info(str(self.changes.all_time_history[actual_version_pointer].prev_id))
            actual_version_pointer = self.changes.all_time_history[actual_version_pointer].prev_id

            # logger.info("%s %s %s " % (type(pointer_on_version_to_recover), type(actual_version_pointer), pointer_on_version_to_recover == actual_version_pointer))
            if actual_version_pointer is None:
                # logger.warning("version could not be found 'backward'")
                undo_redo_list = []
                break

        # forward
        # logger.debug("actual version id %s " % self.changes.trail_history[self.changes.trail_pointer].version_id)
        if self.changes.trail_pointer == -1:
            actual_version_pointer = self.changes.trail_history[self.changes.trail_pointer + 1].version_id
            undo_redo_list.append((actual_version_pointer, 'redo'))
        elif self.changes.trail_pointer is None:
            actual_version_pointer = self.changes.trail_history[0].version_id
        else:
            actual_version_pointer = self.changes.trail_history[self.changes.trail_pointer].version_id
        if not undo_redo_list or self.changes.trail_pointer == -1 \
                and self.changes.all_time_history[actual_version_pointer].next_id is not None:
            actual_version_pointer = self.changes.all_time_history[actual_version_pointer].next_id
            undo_redo_list.append((actual_version_pointer, 'redo'))
            while not int(pointer_on_version_to_recover) == int(actual_version_pointer):
                # logger.info("%s" % self.changes.all_time_history[actual_version_pointer])
                # logger.info(str(self.changes.all_time_history[actual_version_pointer].next_id))
                actual_version_pointer = self.changes.all_time_history[actual_version_pointer].next_id

                # logger.info("%s %s %s " % (type(pointer_on_version_to_recover), type(actual_version_pointer), pointer_on_version_to_recover == actual_version_pointer))
                if actual_version_pointer is None:
                    logger.warning("version could not be found 'forward' and 'backward'")
                    undo_redo_list = []
                    break
                undo_redo_list.append((actual_version_pointer, 'redo'))

        # logger.info("found steps to perform %s to reach version_id %s" % (undo_redo_list, pointer_on_version_to_recover))
        for elem in undo_redo_list:
            if elem[1] == 'undo':
                # do undo
                self._undo(elem[0])
            else:
                # do redo
                self._redo(elem[0])
                # self.changes.all_time_history[elem[0]].action.redo()

    def _undo(self, version_id):
        self.busy = True
        self.changes.all_time_history[version_id].action.undo()
        self.changes.trail_pointer -= 1
        self.changes.all_time_pointer -= 1
        self.busy = False
        if isinstance(self.changes.trail_history[self.changes.trail_pointer + 1], StateMachineAction):
            # logger.debug("StateMachineAction Undo")
            self._re_initiate_observation()

    def undo(self):
        if not self.changes.trail_history or self.changes.trail_pointer < 0 \
                or not self.changes.trail_pointer < len(self.changes.trail_history):
            logger.debug("There is no more TrailEditionHistory element to Undo")
            return
        # else:
        #     logger.debug("do Undo %s %s %s" % (bool(self.changes.trail_history), self.changes.trail_history, (self.changes.trail_pointer, len(self.changes.trail_history))))
        self.busy = True
        self.changes.undo()
        self.busy = False
        if isinstance(self.changes.trail_history[self.changes.trail_pointer + 1], StateMachineAction):
            # logger.debug("StateMachineAction Undo")
            self._re_initiate_observation()

    def _redo(self, version_id):
        self.busy = True
        self.changes.all_time_history[version_id].action.redo()
        self.changes.trail_pointer += 1
        self.changes.all_time_pointer += 1
        self.busy = False
        if self.changes.trail_history is not None \
                and self.changes.trail_pointer < len(self.changes.trail_history) \
                and isinstance(self.changes.trail_history[self.changes.trail_pointer], StateMachineAction):
            # logger.debug("StateMachineAction Redo")
            self._re_initiate_observation()

    def redo(self):
        if not self.changes.trail_history or self.changes.trail_history and not self.changes.trail_pointer + 1 < len(
                self.changes.trail_history):
            logger.debug("There is no more TrailHistory element to Redo")
            return
        # else:
        #     logger.debug("do Redo %s %s %s" % (bool(self.changes.trail_history), self.changes.trail_history, (self.changes.trail_pointer, len(self.changes.trail_history))))
        self.busy = True
        self.changes.redo()
        self.busy = False
        if isinstance(self.changes.trail_history[self.changes.trail_pointer], StateMachineAction):
            # logger.debug("StateMachineAction Redo")
            self._re_initiate_observation()

    def _interrupt_actual_action(self):
        # self.busy = True
        # self.actual_action.undo()
        # self.busy = False
        self.locked = False
        self.count_before = 0

    def _re_initiate_observation(self):
        # logger.info("re initiate root_state observation")
        self.relieve_model(self.__buffered_root_state_model)
        self.observe_model(self.state_machine_model.root_state)
        self.__buffered_root_state_model = self.state_machine_model.root_state

    @staticmethod
    def store_test_log_file(string):
        with open(HISTORY_DEBUG_LOG_FILE, 'a+') as f:
            f.write(string)
        f.closed

    def start_new_action(self, overview):
        """

        :param overview:
        :return:
        """
        if self.fake:
            self.actual_action = ActionDummy()
            return True

        result = True
        cause = overview['method_name'][-1]
        if self.with_prints:
            logger.info("create Action for: {0} for prop_name: {1}".format(overview['method_name'][-1], overview['prop_name'][-1]))

        if self.with_debug_logs:
            self.store_test_log_file(str(overview) + "\n")
            if isinstance(overview['instance'][-1], State):
                self.store_test_log_file(overview['method_name'][-1] + "\t" + str(overview['instance'][-1]) + "\t" + overview['instance'][-1].get_path() + "\n")
            else:
                self.store_test_log_file(overview['method_name'][-1] + "\t" + str(overview['instance'][-1]) + "\t" + overview['instance'][-1].parent.get_path() + "\n")

        if self.refactored_history:
            if isinstance(overview['instance'][-1], DataFlow) or \
                    isinstance(overview['instance'][-1], Transition) or \
                    isinstance(overview['instance'][-1], ScopedVariable):
                if isinstance(overview['instance'][-1], DataFlow):
                    assert overview['instance'][-1] is overview['model'][-1].data_flow
                    action_class = DataFlowAction
                elif isinstance(overview['instance'][-1], Transition):
                    assert overview['instance'][-1] is overview['model'][-1].transition
                    action_class = TransitionAction
                else:
                    assert overview['instance'][-1] is overview['model'][-1].scoped_variable
                    action_class = ScopedVariableAction  # is a DataPort too
                if self.with_debug_logs:
                    self.store_test_log_file("#1 DataFlow, Transition, ScopedVariable \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['instance'][0].get_path(), overview['instance'][-1].parent.get_path()))
                self.actual_action = action_class(parent_path=overview['instance'][-1].parent.get_path(),
                                                  state_machine_model=self.state_machine_model,
                                                  overview=overview)
            elif isinstance(overview['instance'][-1], Outcome):
                assert overview['instance'][-1] is overview['model'][-1].outcome
                if self.with_debug_logs:
                    self.store_test_log_file("#2 Outcome \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['instance'][0].get_path(), overview['instance'][-1].parent.get_path()))
                self.actual_action = OutcomeAction(parent_path=overview['instance'][-1].parent.get_path(),
                                                   state_machine_model=self.state_machine_model,
                                                   overview=overview)
            elif isinstance(overview['instance'][-1], DataPort):
                if isinstance(overview['instance'][-1], InputDataPort):
                    assert overview['instance'][-1] is overview['model'][-1].data_port
                else:
                    assert overview['instance'][-1] is overview['model'][-1].data_port
                if self.with_debug_logs:
                    self.store_test_log_file("#3 DataPort \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['instance'][0].get_path(), overview['instance'][-1].parent.get_path()))
                self.actual_action = DataPortAction(parent_path=overview['instance'][-1].parent.get_path(),
                                                    state_machine_model=self.state_machine_model,
                                                    overview=overview)
            elif isinstance(overview['instance'][-1], State):
                assert overview['instance'][-1] is overview['model'][-1].state
                if "add_" in cause:
                    if self.with_debug_logs:
                        self.store_test_log_file("#3 ADD \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].state.get_path()))
                    self.actual_action = AddObjectAction(parent_path=overview['instance'][-1].get_path(),
                                                         state_machine_model=self.state_machine_model,
                                                         overview=overview)
                elif "remove_" in cause:
                    assert cause in ["remove_transition", "remove_data_flow", "remove_outcome", "remove_input_data_port",
                                     "remove_output_data_port", "remove_scoped_variable", "remove_state"]
                    if ("transition" in cause or "data_flow" in cause or "scoped_variable" in cause or "state" in cause) or\
                            (("data_port" in cause or "outcome" in cause) and not isinstance(overview['model'][-1].state.parent, State)):
                        if self.with_debug_logs:
                            self.store_test_log_file("#4 REMOVE1 \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].state.get_path()))
                        # if "transition" in cause:
                        #     return self.start_new_action_old(overview)
                        self.actual_action = RemoveObjectAction(parent_path=overview['instance'][-1].get_path(),
                                                                state_machine_model=self.state_machine_model,
                                                                overview=overview)
                    elif "data_port" in cause or "outcome" in cause:

                        if isinstance(overview['instance'][-1].parent, State):
                            if self.with_debug_logs:
                                self.store_test_log_file("#5 REMOVE2 \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].parent.state.get_path()))
                            self.actual_action = RemoveObjectAction(parent_path=overview['instance'][-1].parent.get_path(),
                                                                    state_machine_model=self.state_machine_model,
                                                                    overview=overview)
                        else:
                            if self.with_debug_logs:
                                self.store_test_log_file("#5 REMOVE3 \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].parent.state.get_path()))
                            self.actual_action = RemoveObjectAction(parent_path=overview['instance'][-1].get_path(),
                                                                    state_machine_model=self.state_machine_model,
                                                                    overview=overview)
                    else:
                        logger.warning("un foreseen cause: {0} in remove state element".format(cause))
                        assert False
                else:
                    if self.with_debug_logs:
                        self.store_test_log_file("#6 STATE \n\tmodel: {0} {1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].state.get_path()))
                    self.actual_action = StateAction(parent_path=overview['instance'][-1].get_path(),
                                                     state_machine_model=self.state_machine_model,
                                                     overview=overview)
            elif isinstance(overview['instance'][-1], StateMachine):
                assert overview['instance'][-1] is overview['model'][-1].state_machine
                assert False  # should never happen
            else:  # FAILURE
                logger.warning("History may need update, tried to start observation of new action that is not classifiable "
                            "\n%s \n%s \n%s \n%s",
                            overview['model'][0], overview['prop_name'][0], overview['info'][-1], overview['info'][0])
                assert False  # should never happen

            return result

        else:
            return self.start_new_action_old(overview)

    def start_new_action_old(self, overview):

        result = True

        if isinstance(overview['instance'][-1], DataFlow) or \
                isinstance(overview['instance'][-1], Transition) or \
                isinstance(overview['instance'][-1], ScopedVariable):  # internal changes No Add or Remove Actions
            if self.with_debug_logs:
                self.store_test_log_file("$1 DataFlow, Transition, ScopedVariable Change\n model_path: {0}{1}\nparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].parent.state.get_path()))
            if self.with_prints:
                print "CHANGE OF OBJECT", overview['info'][-1]
            # the model should be StateModel or ContainerStateModel and "info" from those model notification
            self.actual_action = Action(parent_path=overview['instance'][-1].parent.get_path(),
                                        state_machine_model=self.state_machine_model,
                                        overview=overview)

        elif overview['model'][-1].parent and (isinstance(overview['instance'][-1], DataPort) or
                                               isinstance(overview['instance'][-1], Outcome) or
                                               overview['method_name'][-1] in ['add_outcome', 'remove_outcome',
                                                                               'add_output_data_port',
                                                                               'remove_output_data_port',
                                                                               'add_input_data_port',
                                                                               'remove_input_data_port']):

            if self.with_prints:
                if isinstance(overview['instance'][-1], State):
                    print "Path_root1: ", overview['instance'][-1].get_path()
                else:
                    print "Path_root1: ", overview['instance'][-1].parent.get_path()

            if overview['model'][-1].parent:
                if not isinstance(overview['model'][-1].parent.state, State):
                    level_status = 'State'
                    self.actual_action = Action(parent_path=overview['instance'][-1].get_path(),
                                                state_machine_model=self.state_machine_model,
                                                overview=overview)
                elif not isinstance(overview['model'][-1].parent.state.parent, State):  # is root_state
                    level_status = 'ParentState'
                    self.actual_action = Action(parent_path=overview['instance'][-1].parent.get_path(),
                                                state_machine_model=self.state_machine_model,
                                                overview=overview)
                else:
                    level_status = 'ParentParentState'
                    self.actual_action = Action(parent_path=overview['instance'][-1].parent.parent.get_path(),
                                                state_machine_model=self.state_machine_model,
                                                overview=overview)
                if self.with_debug_logs:
                    if isinstance(overview['instance'][-1], State):
                        self.store_test_log_file("$2 '{3}' add, remove modify of outcome, input or output\n\tmodel_path: {0}{1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].state.get_path(),level_status))
                    else:
                        self.store_test_log_file("$2 '{3}' modify of outcome, input or output\n\tmodel_path: {0}{1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].parent.state.get_path(),level_status))
            else:
                assert False

        elif overview['prop_name'][-1] == 'state':
            if self.with_prints:
                print "path: ", overview['instance'][-1].get_path(), "\npath: ", overview['model'][-1].state.get_path()
            if "add_" in overview['method_name'][-1]:
                if self.with_debug_logs:
                    self.store_test_log_file("$5 add Outcome,In-OutPut in root and State, ScopedVariable, DateFlow or Transition\n\tmodel_path: {0}{1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].state.get_path()))
                self.actual_action = Action(parent_path=overview['instance'][-1].get_path(),
                                                     state_machine_model=self.state_machine_model,
                                                     overview=overview)
            else:
                if self.with_debug_logs:
                    self.store_test_log_file("$5 remove Outcome,In-OutPut in root and State, ScopedVariables, DateFlow or Transition\n\tmodel_path: {0}{1}\n\tparent_path: {2}\n".format(overview['model'][0], overview['model'][0].state.get_path(), overview['model'][-1].state.get_path()))
                self.actual_action = Action(parent_path=overview['instance'][-1].get_path(),
                                            state_machine_model=self.state_machine_model,
                                            overview=overview)

        else:  # FAILURE
            logger.warning("History may need update, tried to start observation of new action that is not classifiable "
                        "\n%s \n%s \n%s \n%s",
                        overview['model'][0], overview['prop_name'][0], overview['info'][-1], overview['info'][0])
            return False

        return result

    def finish_new_action(self, overview):
        # logger.debug("History stores AFTER")
        if self.with_debug_logs:
            self.store_test_log_file(str(overview) + "\n")

        try:
            self.actual_action.set_after(overview)
            self.state_machine_model.history.changes.insert_action(self.actual_action)
            # logger.debug("history is now: %s" % self.state_machine_model.history.changes.single_trail_history())
            self.tmp_meta_storage = get_state_element_meta(self.state_machine_model.root_state)
        except:
            logger.debug("Failure occurred while finishing action")
            traceback.print_exc(file=sys.stdout)

    def meta_changed_notify_after(self, changed_parent_model, changed_model, recursive_changes):
        raise DeprecationWarning

    def manual_changed_notify_before(self, change_type, changed_parent_model, changed_model, recursive_changes):
        pass

    def manual_changed_notify_after(self, change_type, changed_parent_model, changed_model, recursive_changes):
        """
        :param changed_parent_model rafcon.mvc.models.container_state.ContainerStateModel: model that holds the changed model
        :param changed_model gtkmvc.Model: inherent class of gtkmvc.Model like TransitionModel, StateModel and so on
        :param recursive_changes bool: indicates if the changes are recursive and touch multiple or all recursive childs
        :return:
        """
        if change_type == 'gui_meta_data_changed':
            # store meta data

            from rafcon.mvc.models.state import StateModel
            from rafcon.mvc.models.container_state import ContainerState

            if isinstance(changed_model, StateModel) or isinstance(changed_model, ContainerState):
                logger.debug("state %s '%s' history got notification that Meta data has changed" %
                             (changed_model.state.state_id, changed_model.state.name))
                # -> in case of undo/redo overwrite Model.meta-dict

                # self.actual_action = Action('meta_data_changed', changed_parent_model.state.get_path(),  # instance path of parent
                #                             changed_model, 'meta_data_changed', {},
                #                             state_machine_model=self.state_machine_model)
                # b_tuple = self.actual_action.before_storage
                # meta_dict = self.get_state_element_meta_from_tmp_storage(changed_model.state.get_path())
                # mod_tuple = (b_tuple[0], b_tuple[1], b_tuple[2], meta_dict, b_tuple[4], b_tuple[5])
                # self.actual_action.before_storage = mod_tuple
                # self.finish_new_action(changed_model, 'meta_data_changed', {})

    @ModelMT.observe("state_machine", before=True)
    def assign_notification_change_type_root_state_before(self, model, prop_name, info):
        if info.method_name != "root_state_before_change":
            return
        if self.busy:  # if proceeding undo or redo
            return
        # print model, prop_name, info
        if info['kwargs']['method_name'] == "change_root_state_type":
            if self.with_prints:
                print "ROOT_STATE is NEW", model, prop_name, info
            overview = NotificationOverview(info)
            if self.with_debug_logs:
                self.store_test_log_file(str(overview) + "\n")
            assert overview['method_name'][-1]
            self.actual_action = StateMachineAction(parent_path=overview['instance'][-1].root_state.get_path(),
                                                    state_machine_model=self.state_machine_model,
                                                    overview=overview)
            self.count_before += 1
            if self.with_prints:
                print "LOCKED count up state_machine", self.count_before
            self.locked = True

    @ModelMT.observe("state_machine", after=True)
    def assign_notification_change_type_root_state_after(self, model, prop_name, info):
        if info.method_name != "root_state_after_change":
            return
        if info.result == "CRASH in FUNCTION" or isinstance(info.result, Exception):
            if self.with_prints:
                logger.warning("function crash detected sm_after")
            return self._interrupt_actual_action()

        if self.busy:  # if proceeding undo or redo
            return
        # print model, prop_name, info
        if info['kwargs']['method_name'] == "change_root_state_type":
            overview = NotificationOverview(info)
            if overview['method_name'][-1] == 'parent':
                return
            assert overview['method_name'][-1] == "change_root_state_type"
            # logger.debug("History state_machine_AFTER")
            if self.with_prints:
                print "ROOT_STATE is NEW", model, prop_name, info
            if self.locked:
                self.count_before -= 1
                if self.with_prints:
                    print "LOCKED count down state_machine", self.count_before
                if self.count_before == 0:
                    self.locked = False
                    if self.with_prints:
                        print "IN HISTORY", model, prop_name, info
                    if prop_name == "states" and \
                            (info.kwargs.result == "CRASH in FUNCTION" or isinstance(info.kwargs.result, Exception)):
                        if self.with_prints:
                            print "HISTORY COUNT WAS 0 AND RESET FAILURE to the previous version of the state machine"
                        self.actual_action.undo()
                    else:
                        self.finish_new_action(overview)
                        self._re_initiate_observation()
                        if self.with_prints:
                            print "HISTORY COUNT WAS OF SUCCESS"
            else:
                if self.with_prints:
                    print "HISTORY after not count"

    @ModelMT.observe("states", before=True)
    def assign_notification_states_before(self, model, prop_name, info):
        if self.with_prints:
            print "states_before: ", model, prop_name, info
        if self.busy:  # if proceeding undo or redo
            return
        else:
            # logger.debug("History states_BEFORE")  # \n%s \n%s \n%s" % (model, prop_name, info))

            overview = NotificationOverview(info, with_prints=self.with_prints)
            # skipped state changes
            if (overview['method_name'][0] == 'state_change' and overview['method_name'][-1] in ['active',
                                                                                                 'child_execution',
                                                                                                 'state_execution_status']) or \
                    not overview['method_name'][0] == 'state_change' or \
                    overview['method_name'][-1] == 'parent':
                return

            # lock changes
            if self.locked:
                self.count_before += 1
                if self.with_prints:
                    print "LOCKED count up", self.count_before
            else:
                if self.with_prints:
                    print "NEW HISTORY ELEMENT", info
                if self.start_new_action(overview):
                    self.count_before += 1
                    if self.with_prints:
                        print "LOCKED count up", self.count_before
                    self.locked = True
                else:
                    if self.with_prints:
                        print "FAILED to start NEW HISTORY ELEMENT"

    @ModelMT.observe("states", after=True)
    def assign_notification_states_after(self, model, prop_name, info):
        """
        This method is called, when any state, transition, data flow, etc. within the state machine changes. This
        then typically requires a redraw of the graphical editor, to display these changes immediately.
        :param model: The state machine model
        :param prop_name: The property that was changed
        :param info: Information about the change
        """
        if self.with_prints:
            print "states_after: ", model, prop_name, info
        if hasattr(info, "kwargs") and info.kwargs and \
                (info.kwargs['result'] == "CRASH in FUNCTION" or isinstance(info.kwargs['result'], Exception)):
            if self.with_prints:
                logger.warning("function crash detected states_after")
            return self._interrupt_actual_action()

        if self.busy or info.method_name == 'state_change' and \
                info.kwargs.prop_name == 'state' and \
                info.kwargs.method_name in ['active', 'child_execution', 'state_execution_status']:
            return
        else:
            # logger.debug("History states_AFTER")  # \n%s \n%s \n%s" % (model, prop_name, info))

            overview = NotificationOverview(info, with_prints=self.with_prints)
            # changes of parent are not observed
            if overview['method_name'][0] == 'state_change' and \
                    overview['method_name'][-1] in ['active', 'child_execution', 'state_execution_status'] or \
                    not overview['method_name'][0] == 'state_change' or \
                    overview['method_name'][-1] == 'parent':
                if self.with_prints:
                    print overview['method_name']
                return

            if self.locked:
                self.count_before -= 1
                if self.with_prints:
                    print "LOCKED count down", self.count_before
                if self.count_before == 0:
                    self.locked = False
                    if self.with_prints:
                        print "IN HISTORY", model, prop_name, info

                    self.finish_new_action(overview)
                    if self.with_prints:
                        print "HISTORY COUNT WAS OF SUCCESS"
            else:
                if self.with_prints:
                    print "HISTORY after not count"

    @ModelMT.observe("state", before=True)
    @ModelMT.observe("outcomes", before=True)
    @ModelMT.observe("is_start", before=True)
    @ModelMT.observe("transitions", before=True)
    @ModelMT.observe("data_flows", before=True)
    @ModelMT.observe("input_data_ports", before=True)
    @ModelMT.observe("output_data_ports", before=True)
    @ModelMT.observe("scoped_variables", before=True)
    def assign_notification_root_state_before(self, model, prop_name, info):
        if self.with_prints:
            print "root_state_before: ", model, prop_name, info
        if self.busy or info.method_name in ['active', 'child_execution', 'state_execution_status']:
            return
        # first element should be prop_name="state_machine", instance=StateMachine and model=StateMachineModel
        # second element should be Prop_name="states" if root_state child elements are changed
        # --- for root_state elements it has to be prop_name in ["data_flows", "transitions", "input_data_ports",
        #                                                        "output_data_ports", "scoped_variables"]
        # third (and last element) should be prop_name in ["data_flow", "transition",
        else:
            # logger.debug("History BEFORE")  # \n%s \n%s \n%s" % (model, prop_name, info))

            overview = NotificationOverview(info, with_prints=self.with_prints)
            # changes of parent are not observed
            if overview['method_name'][-1] == 'parent':
                return

            if self.locked:
                self.count_before += 1
                if self.with_prints:
                    print "LOCKED count up", self.count_before
            else:
                if self.with_prints:
                    print "NEW HISTORY ELEMENT", info

                if self.start_new_action(overview):
                    self.count_before += 1
                    if self.with_prints:
                        print "LOCKED count up", self.count_before
                    self.locked = True
                else:
                    if self.with_prints:
                        print "FAILED to start NEW HISTORY ELEMENT"

    @ModelMT.observe("state", after=True)
    @ModelMT.observe("outcomes", after=True)
    @ModelMT.observe("is_start", after=True)
    @ModelMT.observe("transitions", after=True)
    @ModelMT.observe("data_flows", after=True)
    @ModelMT.observe("input_data_ports", after=True)
    @ModelMT.observe("output_data_ports", after=True)
    @ModelMT.observe("scoped_variables", after=True)
    def assign_notification_root_state_after(self, model, prop_name, info):
        """
        This method is called, when any state, transition, data flow, etc. within the state machine changes. This
        then typically requires a redraw of the graphical editor, to display these changes immediately.
        :param model: The state machine model
        :param prop_name: The property that was changed
        :param info: Information about the change
        """
        if self.with_prints:
            print "root_state_after: ", model, prop_name, info

        if info.result == "CRASH in FUNCTION" or isinstance(info.result, Exception):
            if self.with_prints:
                logger.warning("function crash detected state_after")
            return self._interrupt_actual_action()

        if self.busy or info.method_name in ['active', 'child_execution', 'state_execution_status']:
            return
        else:
            # logger.debug("History state_AFTER")  # \n%s \n%s \n%s" % (model, prop_name, info))

            overview = NotificationOverview(info, with_prints=self.with_prints)
            # changes of parent are not observed
            if overview['method_name'][-1] == 'parent':
                return

            if self.locked:
                self.count_before -= 1
                if self.with_prints:
                    print "LOCKED count down", self.count_before
                if self.count_before == 0:
                    self.locked = False
                    if self.with_prints:
                        print "IN HISTORY", model, prop_name, info
                    if prop_name == "states" and \
                            (info.kwargs.result == "CRASH in FUNCTION" or isinstance(info.kwargs['result'], Exception)):
                        if self.with_prints:
                            print "HISTORY COUNT WAS 0 AND RESET FAILURE to the previous version of the state machine"
                        self.actual_action.undo()
                    else:
                        self.finish_new_action(overview)
                        if self.with_prints:
                            print "HISTORY COUNT WAS OF SUCCESS"
            else:
                if self.with_prints:
                    print "HISTORY after not count"


class ChangeHistory(Observable):
    """This Class should remember all historical changes insert. Basic functionalities provide typical store change,
    undo and redo functionalities. Additionally there will be implemented functionalities that never forget a single
    change that was insert for debugging reasons.
    - the pointer are pointing on the next undo ... so redo is pointer + 1
    - all_actions is a type of a tree # prev_id, action, next_id, old_next_ids
    """

    def __init__(self):
        Observable.__init__(self)
        self.trail_history = []
        self.all_time_history = []

        self.trail_pointer = None
        self.all_time_pointer = None
        self.counter = 0

        self.with_prints = False

    @Observable.observed
    def insert_action(self, action):
        # insert new element in
        action.version_id = self.counter
        self.all_time_history.append(HistoryTreeElement(prev_id=self.all_time_pointer,
                                                        action=action, next_id=None, old_next_ids=[]))
        self.counter += 1

        # set pointer of previous element
        if self.all_time_pointer is not None:
            prev_action = self.all_time_history[self.all_time_pointer]
            prev_action.old_next_ids.append(prev_action.next_id)
            prev_action.next_id = len(self.all_time_history) - 1

        # do single trail history
        if self.trail_pointer is not None:
            if self.trail_pointer > len(self.trail_history) - 1 or self.trail_pointer < 0:
                logger.error('History is broken may!!! %s' % self.trail_pointer)
            while not self.trail_pointer == len(self.trail_history) - 1:
                if self.with_prints:
                    print "pointer: %s %s" % (self.trail_pointer, len(self.trail_history))
                self.trail_history.pop()
        self.trail_history.append(action)

        # print '\n\n\n\n\n################ PUT pointer ON: Trail: %s All Time History: %s\n\n\n\n\n' % \
        #       (len(self.trail_history) - 1, len(self.all_time_history) - 1)
        self.trail_pointer = len(self.trail_history) - 1
        if self.trail_pointer == -1:
            self.trail_pointer = None
        self.all_time_pointer = len(self.all_time_history) - 1  # general should be equal to self.counter and version_id

    @Observable.observed
    def undo(self):
        # logger.debug("try undo: undo_pointer: %s history lenght: %s" % (self.trail_pointer, len(self.trail_history)))
        if self.trail_pointer is not None and not self.trail_pointer < 0:
            self.trail_history[self.trail_pointer].undo()
            self.trail_pointer -= 1
            self.all_time_pointer -= 1
        elif self.trail_pointer is not None:
            logger.warning("No UNDO left over!!!")
        else:
            logger.error("History undo FAILURE")

    @Observable.observed
    def redo(self):
        # logger.debug("try redo: undo_pointer: %s history lenght: %s" % (self.trail_pointer, len(self.trail_history)))
        if self.trail_history is not None and self.trail_pointer + 1 < len(self.trail_history):
            self.trail_history[self.trail_pointer + 1].redo()
            self.trail_pointer += 1
            self.all_time_pointer += 1
        elif self.trail_history is not None:
            logger.warning("No REDO left over!!!")
        else:
            logger.error("History redo FAILURE")

    def single_trail_history(self):
        if self.is_end():
            return self.trail_history  # [:]
        else:
            if self.with_prints:
                print "pointer: ", str(self.trail_pointer)
            return self.trail_history  # [:self.trail_pointer + 1]

    def is_end(self):
        return len(self.trail_history) - 1 == self.trail_pointer

    @Observable.observed
    def reset(self):
        logger.debug("################ RESET ChangeHistory PUT ALL TO INITIATION")
        self.trail_history = []
        self.all_time_history = []

        self.trail_pointer = None
        self.all_time_pointer = None
        self.counter = 0
