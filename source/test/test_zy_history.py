import logging
import gtk
import threading
import time
import os
import signal

# general tool elements
from rafcon.utils import log

# core elements
from rafcon.statemachine.state_machine import StateMachine
from rafcon.statemachine.script import Script, ScriptType
from rafcon.statemachine.enums import StateType
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from rafcon.statemachine.states.barrier_concurrency_state import BarrierConcurrencyState

# mvc elements
from gtkmvc.observer import Observer
from rafcon.mvc.controllers import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView
from rafcon.mvc.views import LoggingView

# singleton elements
import rafcon.statemachine.singleton
# import rafcon.mvc.singleton
from rafcon.mvc.config import global_gui_config
from rafcon.statemachine.config import global_config

# test environment elements
import variables_for_pytest
from variables_for_pytest import test_multithrading_lock, call_gui_callback
from test_z_gui_state_type_change import store_state_elements, list_store_id_dict, check_state_elements, \
    check_list_ES, check_list_HS, check_list_BCS, check_list_PCS, \
    check_list_root_ES, check_list_root_HS, check_list_root_BCS, check_list_root_PCS, \
    get_state_editor_ctrl_and_store_id_dict
from test_z_gui_states_editor_widget import check_state_editor_models

NO_SAVE = False


def on_save_activate(state_machine_m, logger):
    if state_machine_m is None or NO_SAVE:
        return
    save_path = state_machine_m.state_machine.file_system_path
    if save_path is None:
        return

    logger.debug("Saving state machine to {0}".format(save_path))
    rafcon.statemachine.singleton.global_storage.save_statemachine_as_yaml(
        state_machine_m.state_machine,
        state_machine_m.state_machine.file_system_path, delete_old_state_machine=True)

    state_machine_m.root_state.store_meta_data_for_state()
    logger.debug("Successfully saved graphics meta data.")


def save_state_machine(sm_model, path, logger, with_gui=False, menubar_ctrl=None):
    print "in Removal: \n", rafcon.statemachine.singleton.global_storage._paths_to_remove_before_sm_save.values()

    def print_states(state):
        if hasattr(state, "states"):
            for state_id, child_state in state.states.iteritems():
                print child_state.get_path(), child_state.script._path
                print_states(child_state)
    print_states(sm_model.state_machine.root_state)

    print "do SAVEING OF STATEMACHINE"
    if with_gui:
        sm_model.state_machine.file_system_path = path
        print "by Menubar_ctrl"
        call_gui_callback(menubar_ctrl.on_save_activate, None)
    else:
        sm_model.state_machine.file_system_path = path
        print "by Function"
        on_save_activate(sm_model, logger)

    from test_z_storage import check_that_all_files_are_there

    check_that_all_files_are_there(sm_model, path, False, True)


def save_and_quit(sm_model, path, menubar_ctrl, with_gui):
    if with_gui:
        sm_model.state_machine.file_system_path = path
        call_gui_callback(menubar_ctrl.on_save_activate, None)
        call_gui_callback(menubar_ctrl.on_stop_activate, None)
        call_gui_callback(menubar_ctrl.on_quit_activate, None)


class NotificationLogObserver(Observer):
    """ This observer is a abstract class to counts and store notification
    """

    def __init__(self, model, with_print=False):

        self.log = {"before": {}, "after": {}}
        self.reset()
        self.observed_model = model
        self.with_print = with_print
        self.no_failure = True

        Observer.__init__(self, model)

    def reset(self):
        """ Initiate and reset the log dictionary """

    def get_number_of_notifications(self):
        nr = 0
        for key, l in self.log['before'].iteritems():
            nr += len(l)
        for key, l in self.log['after'].iteritems():
            nr += len(l)
        return nr


class StateNotificationLogObserver(NotificationLogObserver):
    """ This observer counts and stores notification of StateModel-Class
    """

    def __init__(self, model, with_print=False):
        NotificationLogObserver.__init__(self, model, with_print)

    def reset(self):
        self.log = {"before": {'states': [], 'state': [],
                               'outcomes': [], 'input_data_ports': [], 'output_data_ports': [], 'scoped_variables': [],
                               'transitions': [], 'data_flows': [], 'is_start': []},
                    "after": {'states': [], 'state': [],
                              'outcomes': [], 'input_data_ports': [], 'output_data_ports': [], 'scoped_variables': [],
                              'transitions': [], 'data_flows': [], 'is_start': []}}
        self.no_failure = True

    @Observer.observe('states', before=True)
    @Observer.observe("state", before=True)
    @Observer.observe("outcomes", before=True)
    @Observer.observe("input_data_ports", before=True)
    @Observer.observe("output_data_ports", before=True)
    @Observer.observe("scoped_variables", before=True)
    @Observer.observe("transitions", before=True)
    @Observer.observe("data_flows", before=True)
    @Observer.observe("is_start", before=True)
    def notification_before(self, model, prop_name, info):
        # print "parent call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
        #       (prop_name, info.instance, info.method_name, info.result)
        #if info.method_name in self.method_list:
        if prop_name in self.log['before']:
            self.log['before'][prop_name].append({'model': model, 'prop_name': prop_name, 'info': info})
            if self.with_print:
                print "++++++++ log BEFORE instance '%s' and property '%s' in state %s" % \
                      (info.instance, prop_name, self.observed_model.state.name)
                print "observer: ", self
        else:
            print "!!!! NOT a prop_name '%s' to be observed in BEFORE %s %s" % (prop_name, model, info)
            self.no_failure = False

        self.parent_state_of_notification_source(model, prop_name, info, before_after='before')

    @Observer.observe('states', after=True)
    @Observer.observe("state", after=True)
    @Observer.observe("outcomes", after=True)
    @Observer.observe("input_data_ports", after=True)
    @Observer.observe("output_data_ports", after=True)
    @Observer.observe("scoped_variables", after=True)
    @Observer.observe("transitions", after=True)
    @Observer.observe("data_flows", after=True)
    @Observer.observe("is_start", after=True)
    def notification_after(self, model, prop_name, info):
        if prop_name in self.log['after']:
            self.log['after'][prop_name].append({'model': model, 'prop_name': prop_name, 'info': info})
            if self.with_print:
                print "++++++++ log AFTER instance '%s' and property '%s' in state %s" % \
                      (info.instance, prop_name, self.observed_model.state.name)
                print "observer: ", self
        else:
            print "!!!! NOT a prop_name '%s' to be observed in AFTER %s %s" % (prop_name, model, info)
            self.no_failure = False

        self.parent_state_of_notification_source(model, prop_name, info, before_after='after')

    def parent_state_of_notification_source(self, model, prop_name, info, before_after):
        if self.with_print:
            print "----- xxxxxxx %s \n%s\n%s\n%s\n" % (before_after, model, prop_name, info)

        def set_dict(info, d):
            d['model'].append(info['model'])
            d['prop_name'].append(info['prop_name'])
            d['instance'].append(info['instance'])
            d['method_name'].append(info['method_name'])
            if self.with_print:
                print "set"

        def find_parent(info, elem):
            elem['info'].append(info)
            if 'kwargs' in info and info['kwargs']:
                if self.with_print:
                    print 'kwargs'
                elem['level'].append('kwargs')
                set_dict(info, elem)
                find_parent(info['kwargs'], elem)
            elif 'info' in info and info['info']:
                if self.with_print:
                    print 'info'
                elem['level'].append('info')
                set_dict(info, elem)
                find_parent(info['info'], elem)
            elif 'info' in info:
                set_dict(info, elem)
            elif 'kwargs' in info:
                set_dict(info, elem)
            else:
                if self.with_print:
                    print 'assert info: %s elem: %s' % (info, elem)
                # assert False
            return elem

        overview = find_parent(info, {'model': [], 'prop_name': [], 'instance': [], 'method_name': [], 'level': [],
                                      'info': []})
        # info_print = ''
        # for elem in overview['info']:
        #     info_print += "\n" + str(elem)
        # print info_print
        if self.with_print:
            print overview['model']
            print overview['prop_name']
            print overview['instance']
            print overview['method_name']
            print overview['level']
            print overview['prop_name'][-1]
        if overview['prop_name'][-1] == 'state':
            if self.with_print:
                print "path: ", overview['instance'][-1].get_path(), "\npath: ", overview['model'][-1].state.get_path()
            assert overview['instance'][-1].get_path() == overview['model'][-1].state.get_path()
        else:
            if overview['model'][-1].state.is_root_state:
                overview['model'][-1].state.get_path()
                if self.with_print:
                    print "Path_root: ", overview['model'][-1].state.get_path()
            else:
                overview['model'][-1].parent.state.get_path()
                if self.with_print:
                    print "Path: ", overview['model'][-2].state.get_path(), "\nPath: ", \
                    overview['model'][-1].parent.state.get_path()
                assert overview['model'][-2].state.get_path() == overview['model'][-1].parent.state.get_path().split('/')[0]
        return overview


def create_models(*args, **kargs):

    logger = log.get_logger(__name__)
    logger.setLevel(logging.DEBUG)
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)

    state1 = ExecutionState('State1', state_id='STATE1')
    output_state1 = state1.add_output_data_port("output", "int")
    input_state1 = state1.add_input_data_port("input", "str", "zero")
    state2 = ExecutionState('State2', state_id='STATE2')
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    output_res_state2 = state2.add_output_data_port("res", "int")
    state4 = HierarchyState(name='Nested', state_id='NESTED')
    state4.add_outcome('GoGo')
    output_state4 = state4.add_output_data_port("out", "int")
    state5 = ExecutionState('Nested2', state_id='NESTED2')
    state5.add_outcome('HereWeGo')
    input_state5 = state5.add_input_data_port("in", "int", 0)
    state3 = HierarchyState(name='State3', state_id='STATE3')
    input_state3 = state3.add_input_data_port("input", "int", 0)
    output_state3 = state3.add_output_data_port("output", "int")
    state3.add_state(state4)
    state3.add_state(state5)
    state3.set_start_state(state4)
    state3.add_scoped_variable("share", "int", 3)
    state3.add_transition(state4.state_id, 0, state5.state_id, None)
    state3.add_transition(state5.state_id, 0, state3.state_id, 0)
    state3.add_data_flow(state4.state_id, output_state4, state5.state_id, input_state5)
    state3.add_outcome('Branch1')
    state3.add_outcome('Branch2')

    ctr_state = HierarchyState(name="Container", state_id='CONT2')
    ctr_state.add_state(state1)
    ctr_state.add_state(state2)
    ctr_state.add_state(state3)
    input_ctr_state = ctr_state.add_input_data_port("ctr_in", "str", "zero")
    output_ctr_state = ctr_state.add_output_data_port("ctr_out", "int")
    ctr_state.set_start_state(state1)
    ctr_state.add_transition(state1.state_id, 0, state2.state_id, None)
    ctr_state.add_transition(state2.state_id, 0, state3.state_id, None)
    ctr_state.add_transition(state3.state_id, 0, ctr_state.state_id, 0)
    ctr_state.add_data_flow(state1.state_id, output_state1, state2.state_id, input_par_state2)
    ctr_state.add_data_flow(state2.state_id, output_res_state2, state3.state_id, input_state3)
    ctr_state.add_data_flow(ctr_state.state_id, input_ctr_state, state1.state_id, input_state1)
    ctr_state.add_data_flow(state3.state_id, output_state3, ctr_state.state_id, output_ctr_state)
    ctr_state.name = "Container"

    ctr_state.add_input_data_port("input", "str", "default_value1")
    ctr_state.add_input_data_port("pos_x", "str", "default_value2")
    ctr_state.add_input_data_port("pos_y", "str", "default_value3")

    ctr_state.add_output_data_port("output", "str", "default_value1")
    ctr_state.add_output_data_port("result", "str", "default_value2")

    scoped_variable1_ctr_state = ctr_state.add_scoped_variable("scoped", "str", "default_value1")
    scoped_variable2_ctr_state = ctr_state.add_scoped_variable("my_var", "str", "default_value1")
    scoped_variable3_ctr_state = ctr_state.add_scoped_variable("ctr", "int", 42)

    ctr_state.add_data_flow(ctr_state.state_id, input_ctr_state, ctr_state.state_id, scoped_variable1_ctr_state)
    ctr_state.add_data_flow(state1.state_id, output_state1, ctr_state.state_id, scoped_variable3_ctr_state)

    state_dict = {'Container': ctr_state, 'State1': state1, 'State2': state2, 'State3': state3, 'Nested': state4, 'Nested2': state5}
    sm = StateMachine(ctr_state)
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(sm)

    for sm_in in rafcon.statemachine.singleton.state_machine_manager.state_machines.values():
        rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(sm_in.state_machine_id)
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(sm)

    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(sm)
    rafcon.mvc.singleton.state_machine_manager_model.selected_state_machine_id = sm.state_machine_id

    sm_m = rafcon.mvc.singleton.state_machine_manager_model.state_machines[sm.state_machine_id]
    sm_m.history.fake = False
    print "with_prints is: ", sm_m.history.with_prints
    sm_m.history.with_prints = False
    # return ctr_state, sm_m, state_dict
    return logger, ctr_state, sm_m, state_dict


def get_state_model_by_path(state_model, path):
    path_elems = path.split('/')
    path_elems.pop(0)
    current_state_model = state_model
    for element in path_elems:
        current_state_model = current_state_model.states[element]
    return current_state_model


def test_add_remove_history(with_print=False):
    ##################
    # Root_state elements

    # add state
    # - change state

    # remove state

    # add outcome
    # - change outcome

    # remove outcome

    # add transition
    # - change transition

    # remove transition

    # add input_data_port
    # - change input_data_port

    # remove input_data_port

    # add output_data_port
    # - change output_data_port

    # remove output_data_port

    # add scoped_variable
    # - change scoped_variable

    # remove scoped_variable

    # add data_flow
    # - change data_flow

    # remove data_flow

    # create testbed
    [logger, state, sm_model, state_dict] = create_models()

    import rafcon
    test_history_path1 = '/home_local/test_history_before'
    test_history_path2 = '/home_local/test_history_after'
    state_machine_path = '/tmp/dfc_history_test_add_remove'
    save_state_machine(sm_model, state_machine_path + '_before', logger, with_gui=False, menubar_ctrl=None)

    def store_state_machine(sm_model, path):
        rafcon.statemachine.singleton.global_storage.save_statemachine_as_yaml(
            sm_model.state_machine,
            path,
            delete_old_state_machine=True)
        sm_model.root_state.store_meta_data_for_state()

    sm_history = sm_model.history

    state1 = HierarchyState('state1', state_id='STATE1')
    input_state1 = state1.add_input_data_port("input", "str", "zero")
    output_state1 = state1.add_output_data_port("output", "int")
    output_count_state1 = state1.add_output_data_port("count", "int")

    state2 = ExecutionState('state2', state_id='STATE2')
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    input_number_state2 = state2.add_input_data_port("number", "int", 5)
    output_res_state2 = state2.add_output_data_port("res", "int")

    state_dict['Nested'].add_state(state1)
    state_dict['Nested'].add_state(state2)
    sm_history.changes.reset()
    state_dict['state1'] = state1
    state_dict['state2'] = state2

    StateNotificationLogObserver(sm_model.root_state, with_print=False)

    def do_check_for_state(state_dict, state_name):

        from test_models import check_state_for_all_models

        def check_models_for_state_with_name(state_name, state_dict, sm_model):
            state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
            state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
            check_state_for_all_models(state, state_m)

        #############
        # add outcome
        # print "\n\n###########1", state_dict[state_name].state_id, state_dict[state_name].input_data_ports.keys()
        outcome_super = state_dict[state_name].add_outcome('super')
        assert len(sm_history.changes.single_trail_history()) == 1
        # print "\n\n###########2", state_dict[state_name].state_id, state_dict[state_name].input_data_ports.keys()
        sm_history.undo()
        # print "\n\n###########3", state_dict[state_name].state_id, state_dict[state_name].input_data_ports.keys()
        sm_history.redo()
        # print "\n\n###########4", state_dict[state_name].state_id, state_dict[state_name].input_data_ports.keys()

        ################
        # remove outcome
        # def print_all(state_m):
        #     # state = state_m.state
        #     # print state_m
        #     print state_m.state.name  # , state_m.state.get_path()
        #     if hasattr(state_m, 'states'):
        #         for state_m in state_m.states:
        #             print_all(state_m)

        print sm_model, "\n", sm_model.root_state
        # print_all(sm_model.root_state)
        save_state_machine(sm_model, state_machine_path + '_before', logger, with_gui=False, menubar_ctrl=None)
        state_dict[state_name].remove_outcome(outcome_super)  # new outcome should be the third one
        assert len(sm_history.changes.single_trail_history()) == 2
        save_state_machine(sm_model, state_machine_path + '_after', logger, with_gui=False, menubar_ctrl=None)
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        sm_history.undo()
        save_state_machine(sm_model, state_machine_path + '_undo', logger, with_gui=False, menubar_ctrl=None)
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        save_state_machine(sm_model, state_machine_path + '_redo', logger, with_gui=False, menubar_ctrl=None)

        state_dict[state_name] = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())

        state4 = ExecutionState('State4', state_id='STATE4')
        state5 = ExecutionState('State5', state_id='STATE5')

        #############
        # add state
        print "xyz", state_dict[state_name].states.keys(), state_name
        print "xyz", sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path()).states.keys(), state_name
        state_dict[state_name].add_state(state4)
        print sm_model.state_machine.get_state_by_path(state4.get_path()).get_path()
        assert len(sm_history.changes.single_trail_history()) == 3
        state_dict[state_name].add_state(state5)
        print sm_model.state_machine.get_state_by_path(state5.get_path()).get_path()
        assert len(sm_history.changes.single_trail_history()) == 4
        print state_dict[state_name].states
        # store_state_machine(sm_model, test_history_path1)
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        # store_state_machine(sm_model, test_history_path2)

        # resolve reference
        print state4.get_path()
        print "\n\n\n"
        print sm_model.state_machine.get_state_by_path(state4.get_path())
        print sm_model.state_machine.get_state_by_path(state_dict['Nested'].get_path()).states
        print "\n\n\n"
        print sm_model.state_machine.get_state_by_path(state4.get_path()).get_path(), "\n", state4.get_path()
        state4 = sm_model.get_state_model_by_path(state4.get_path()).state
        state5 = sm_model.get_state_model_by_path(state5.get_path()).state
        state_dict[state_name] = sm_model.get_state_model_by_path(state_dict[state_name].get_path()).state

        outcome_state4 = state4.add_outcome('UsedHere')
        assert len(sm_history.changes.single_trail_history()) == 5
        outcome_state5 = state5.add_outcome('UsedHere')
        assert len(sm_history.changes.single_trail_history()) == 6

        ################
        # add transition from_state_id, from_outcome, to_state_id=None, to_outcome=None, transition_id
        new_transition_id1 = state_dict[state_name].add_transition(from_state_id=state4.state_id, from_outcome=outcome_state4,
                                                                   to_state_id=state5.state_id, to_outcome=None)
        state_m = sm_model.get_state_model_by_path(state_dict[state_name].get_path())
        state = sm_model.state_machine.get_state_by_path(state_dict[state_name].get_path())
        check_state_for_all_models(state, state_m)
        assert len(sm_history.changes.single_trail_history()) == 7
        state_dict[state_name].add_transition(from_state_id=state5.state_id, from_outcome=outcome_state5,
                                              to_state_id=state_dict[state_name].state_id, to_outcome=-1)
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        assert len(sm_history.changes.single_trail_history()) == 8
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)

        # resolve reference
        state4 = sm_model.get_state_model_by_path(state4.get_path()).state
        state5 = sm_model.get_state_model_by_path(state5.get_path()).state
        state_dict[state_name] = sm_model.get_state_model_by_path(state_dict[state_name].get_path()).state

        ###################
        # remove transition
        state_dict[state_name].remove_transition(new_transition_id1)  # new outcome should be the third one
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        assert len(sm_model.history.changes.single_trail_history()) == 9
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)

        # resolve reference
        state4 = sm_model.get_state_model_by_path(state4.get_path()).state
        state5 = sm_model.get_state_model_by_path(state5.get_path()).state
        state_dict[state_name] = sm_model.get_state_model_by_path(state_dict[state_name].get_path()).state

        #############
        # remove state
        state_dict[state_name].remove_state(state5.state_id)
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        assert len(sm_history.changes.single_trail_history()) == 10
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)

        # resolve reference
        state4 = sm_model.get_state_model_by_path(state4.get_path()).state
        state_dict[state_name] = sm_model.get_state_model_by_path(state_dict[state_name].get_path()).state

        #############
        # add input_data_port
        input_state4 = state4.add_input_data_port("input", "str", "zero")
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        assert len(sm_history.changes.single_trail_history()) == 11
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)

        #############
        # remove input_data_port
        state4.remove_input_data_port(input_state4)
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        assert len(sm_history.changes.single_trail_history()) == 12
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)

        #############
        # add output_data_port
        output_state4 = state4.add_output_data_port("output", "int")
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        assert len(sm_history.changes.single_trail_history()) == 13
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)

        #############
        # remove output_data_port
        state4.remove_output_data_port(output_state4)
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        assert len(sm_history.changes.single_trail_history()) == 14
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)

        # resolve reference
        state4 = sm_model.get_state_model_by_path(state4.get_path()).state
        state_dict[state_name] = sm_model.get_state_model_by_path(state_dict[state_name].get_path()).state

        # prepare again state4
        output_state4 = state4.add_output_data_port("output", "int")
        input_state4 = state4.add_input_data_port("input", "str", "zero")
        assert len(sm_history.changes.single_trail_history()) == 16
        output_state4 = state4.add_output_data_port("output", "int")
        assert len(sm_history.changes.single_trail_history()) == 17

        state5 = ExecutionState('State5')
        state_dict[state_name].add_state(state5)
        assert len(sm_history.changes.single_trail_history()) == 18
        input_par_state5 = state5.add_input_data_port("par", "int", 0)
        assert len(sm_history.changes.single_trail_history()) == 19
        output_res_state5 = state5.add_output_data_port("res", "int")
        assert len(sm_history.changes.single_trail_history()) == 20


        #####################
        # add scoped_variable
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        scoped_buffer_nested = state_dict[state_name].add_scoped_variable("buffer", "int")
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        assert len(sm_history.changes.single_trail_history()) == 21
        sm_history.undo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        sm_history.redo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)

        #####################
        # remove scoped_variable
        state_dict[state_name].remove_scoped_variable(scoped_buffer_nested)
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        assert len(sm_model.history.changes.single_trail_history()) == 22
        sm_model.history.undo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        sm_model.history.redo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)

        # resolve reference
        state4 = sm_model.get_state_model_by_path(state4.get_path()).state
        state5 = sm_model.get_state_model_by_path(state5.get_path()).state
        state_dict[state_name] = sm_model.get_state_model_by_path(state_dict[state_name].get_path()).state

        #############
        # add data_flow
        new_df_id = state_dict[state_name].add_data_flow(from_state_id=state4.state_id, from_data_port_id=output_state4,
                                                         to_state_id=state5.state_id, to_data_port_id=input_par_state5)
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        assert len(sm_model.history.changes.single_trail_history()) == 23
        sm_model.history.undo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        sm_model.history.redo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)

        # resolve reference
        state4 = sm_model.get_state_model_by_path(state4.get_path()).state
        state5 = sm_model.get_state_model_by_path(state5.get_path()).state
        state_dict[state_name] = sm_model.get_state_model_by_path(state_dict[state_name].get_path()).state

        ################
        # remove data_flow
        state_dict[state_name].remove_data_flow(new_df_id)
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        assert len(sm_model.history.changes.single_trail_history()) == 24
        sm_model.history.undo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)
        sm_model.history.redo()
        check_models_for_state_with_name(state_name, state_dict, sm_model)

    # state_check_dict1 = print_all_states_with_path_and_name(state_dict['Container'])
    # do_check_for_state(state_dict, state_name='state1')
    # do_check_for_state(state_dict, state_name='state2')
    do_check_for_state(state_dict, state_name='Nested')
    sm_model.history.changes.reset()
    save_state_machine(sm_model, "/tmp/DFC_test/history_add_remove_child_hierarchical_state", logger, with_gui=False)
    # assert check_if_all_states_there(state_dict['Container'], state_check_dict1)
    # state_check_dict2 = print_all_states_with_path_and_name(state_dict['Container'])
    do_check_for_state(state_dict, state_name='Container')
    save_state_machine(sm_model, "/tmp/DFC_test/history_add_remove_root_state", logger, with_gui=False)
    # assert check_if_all_states_there(state_dict['Container'], state_check_dict1)
    # assert check_if_all_states_there(state_dict['Container'], state_check_dict2)


def test_state_property_changes_history(with_print=False):
    ##################
    # state properties

    # change name

    # change parent

    # change states

    # change outcomes

    # change transitions

    # change input_data_ports

    # change output_data_ports

    # change scoped_variables

    # change data_flows

    # change script

    # change state_type

    # change description

    # change active

    # set_start_state

    # change start_state_id

    # change child_execution

    # create testbed
    [logger, state, sm_model, state_dict] = create_models()

    state1 = ExecutionState('State1')
    input_state1 = state1.add_input_data_port("input", "str", "zero")
    output_state1 = state1.add_output_data_port("output", "int")
    output_count_state1 = state1.add_output_data_port("count", "int")

    state2 = ExecutionState('State2')
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    input_number_state2 = state2.add_input_data_port("number", "int", 5)
    output_res_state2 = state2.add_output_data_port("res", "int")

    state_dict['Nested'].add_state(state1)
    assert len(sm_model.history.changes.single_trail_history()) == 1
    state_dict['Nested'].add_state(state2)
    assert len(sm_model.history.changes.single_trail_history()) == 2
    output_res_nested = state_dict['Nested'].add_output_data_port("res", "int")
    assert len(sm_model.history.changes.single_trail_history()) == 3

    oc_again_state1 = state1.add_outcome("again")
    assert len(sm_model.history.changes.single_trail_history()) == 4
    oc_counted_state1 = state1.add_outcome("counted")
    assert len(sm_model.history.changes.single_trail_history()) == 5

    oc_done_state2 = state2.add_outcome("done")
    oc_best_state2 = state2.add_outcome("best")
    oc_full_state2 = state2.add_outcome("full")
    assert len(sm_model.history.changes.single_trail_history()) == 8

    oc_great_nested = state_dict['Nested'].add_outcome("great")
    assert len(sm_model.history.changes.single_trail_history()) == 9

    #######################################
    ######## Properties of State ##########

    # name(self, name)
    state_dict['Nested'].name = 'nested'
    sm_model.history.undo()
    sm_model.history.redo()

    # parent(self, parent) State
    # state_dict['Nested'].parent = state_dict['State3']
    # sm_model.history.undo()
    # sm_model.history.redo()

    # input_data_ports(self, input_data_ports) None or dict
    state_dict['Nested'].input_data_ports = None
    sm_model.history.undo()
    sm_model.history.redo()

    # output_data_ports(self, output_data_ports) None or dict
    state_dict['Nested'].output_data_ports = None
    sm_model.history.undo()
    sm_model.history.redo()

    # outcomes(self, outcomes) None or dict
    state_dict['Nested'].outcomes = state_dict['Nested'].outcomes
    sm_model.history.undo()
    sm_model.history.redo()
    state_dict['Nested'].outcomes = None
    sm_model.history.undo()
    sm_model.history.redo()

    # script(self, script) Script
    state_dict['Nested'].script = Script(script_type=ScriptType.CONTAINER, state=state_dict['Nested'])
    state_dict['Nested'].script = Script(script_type=ScriptType.EXECUTION, state=state_dict['Nested'])
    sm_model.history.undo()
    sm_model.history.redo()

    # state_type(self, state_type) StateType
    state_dict['Nested'].state_type = StateType.PREEMPTION_CONCURRENCY
    sm_model.history.undo()
    sm_model.history.redo()
    state_dict['Nested'].state_type = StateType.HIERARCHY
    sm_model.history.undo()
    sm_model.history.redo()

    # description(self, description) str
    state_dict['Nested'].description = "awesome"
    sm_model.history.undo()
    sm_model.history.redo()

    # # active(self, active) bool
    # state_dict['Nested'].active = True
    # sm_model.history.undo()
    # sm_model.history.redo()
    #
    # # child_execution(self, child_execution) bool
    # state_dict['Nested'].child_execution = True
    # sm_model.history.undo()
    # sm_model.history.redo()
    # TODO state_execution_status has to be checked

    ############################################
    ###### Properties of ContainerState ########

    # TODO May SOLVED - check where all this shit comes from may a observed capsuled set_start_state function in ContainerState will help
    # set_start_state(self, state) State or state_id
    state1_m = sm_model.get_state_model_by_path(state1.get_path())
    state_dict['Nested'].set_start_state(state1_m.state.state_id)
    sm_model.history.undo()
    sm_model.history.redo()
    # set_start_state(self, start_state)
    state2_m = sm_model.get_state_model_by_path(state2.get_path())
    state_dict['Nested'].set_start_state(state2_m.state)
    sm_model.history.undo()
    sm_model.history.redo()

    # TODO all dict setter have to use remove and add functionalities or need better implementation
    # # states(self, states) None or dict
    # state_dict['Nested'].states = None
    # # print "\n\n\n", state_dict['Nested'].states
    # # state_nested_m = sm_model.get_state_model_by_path(state_dict['Nested'].get_path())
    # # print "\n\n\n", state_nested_m.state.states
    # # print "\n\n\n", state_nested_m.states
    # # exit(1)
    # # assert state_nested_m.states
    # sm_model.history.undo()
    # sm_model.history.redo()

    # # transitions(self, transitions) None or dict
    # state_dict['Nested'].transitions = None
    # sm_model.history.undo()
    # sm_model.history.redo()
    #
    # # data_flows(self, data_flows) None or dict
    # state_dict['Nested'].data_flows = None
    # sm_model.history.undo()
    # sm_model.history.redo()

    # scoped_variables(self, scoped_variables) None or dict
    state_dict['Nested'].scoped_variables = None
    sm_model.history.undo()
    sm_model.history.redo()

    save_state_machine(sm_model, "/tmp/DFC_test/history_state_properties", logger, with_gui=False)


def test_outcome_property_changes_history(with_print=False):
    ##################
    # outcome properties

    # change name

    # change outcome_id

    # create testbed
    [logger, state, sm_model, state_dict] = create_models()

    def do_check_for_state(state_dict, state_name='Nested'):
        ####################################################
        # modify outcome and generate in previous a observer
        outcome_models_observer_dict = {}
        for outcome_id, outcome in state_dict['Nested2'].outcomes.iteritems():
            if not outcome_id < 0:
                outcome.name = "new_name_" + str(outcome_id)
                sm_model.history.undo()
                sm_model.history.redo()
                # resolve reference
                state_dict['Nested2'] = sm_model.get_state_model_by_path(state_dict['Nested2'].get_path()).state

        ##########################
        # check for ContainerState -> should be unnecessary
        state_model = sm_model.get_state_model_by_path(state_dict['Nested'].get_path())

        ####################################################
        # modify outcome
        outcome_models_observer_dict = {}
        for outcome_id, outcome in state_dict['Nested'].outcomes.iteritems():
            outcome.name = "new_name_" + str(outcome_id)
            sm_model.history.undo()
            sm_model.history.redo()
            # resolve reference
            state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

        print "\n\n\n\n\n\n\n\n\n\n\n\n"
        state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state
        state_dict['Nested'].outcomes.values()[0].outcome_id += 10
        sm_model.history.undo()
        # TODO MAY DONE
        sm_model.history.redo()

    # do_check_for_state(state_dict, history_ctrl, state_name='Nested')
    do_check_for_state(state_dict, state_name='Container')
    save_state_machine(sm_model, "/tmp/DFC_test/history_outcome_properties", logger, with_gui=False)


def wait_for_states_editor(main_window_controller, tab_key, max_time=5.0):
    assert tab_key in main_window_controller.get_controller('states_editor_ctrl').tabs
    time_waited = 0.0
    state_editor_ctrl = None
    while state_editor_ctrl is None:
        state_editor_ctrl = main_window_controller.get_controller('states_editor_ctrl').tabs[tab_key]['controller']
        time.sleep(0.1)
        time_waited += 0.1
        assert time_waited < max_time

    return state_editor_ctrl, time_waited


def test_transition_property_changes_history(with_print=False):
    ##################
    # transition properties

    # change modify_origin

    # change from_outcome

    # change to_state

    # change to_outcome

    # change transition_id

    # modify_transition_from_state

    # modify_transition_from_outcome

    # modify_transition_to_outcome

    # modify_transition_to_state

    # create testbed
    [logger, state, sm_model, state_dict] = create_models()

    state1 = ExecutionState('State1')
    outcome_again_state1 = state1.add_outcome("again")
    state2 = ExecutionState('State2')
    oc_done_state2 = state2.add_outcome("done")
    oc_best_state2 = state2.add_outcome("best")
    state_dict['Nested'].add_state(state1)
    state_dict['Nested'].add_state(state2)
    oc_great_nested = state_dict['Nested'].add_outcome("great")
    outcome_counted_state1 = state1.add_outcome("counted")
    oc_full_state2 = state2.add_outcome("full")
    # assert False

    new_trans_id = state_dict['Nested'].add_transition(from_state_id=state1.state_id,
                                                       from_outcome=outcome_again_state1,
                                                       to_state_id=state1.state_id,
                                                       to_outcome=None)

    # modify_origin(self, from_state, from_outcome)
    state_dict['Nested'].transitions[new_trans_id].modify_origin(from_state=state2.state_id,
                                                                 from_outcome=oc_full_state2)
    sm_model.history.undo()
    sm_model.history.redo()

    # from_outcome(self, from_outcome)
    state_dict['Nested'].transitions[new_trans_id].from_outcome = oc_done_state2
    sm_model.history.undo()
    sm_model.history.redo()

    # to_state(self, to_state)
    state_dict['Nested'].transitions[new_trans_id].to_state = state2.state_id
    sm_model.history.undo()
    sm_model.history.redo()

    # to_outcome(self, to_outcome)
    # TODO do an example what does not violate the roles of transitions
    # TODO test the respective function in transition
    # state_dict['Nested'].transitions[new_trans_id].to_outcome = oc_great_nested
    # sm_model.history.undo()
    # sm_model.history.redo()

    # # transition_id(self, transition_id)
    # state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state
    # state_dict['Nested'].transitions[new_trans_id].transition_id += 1
    # sm_model.history.undo()
    # # TODO Container state needs a modify transition_id function for proper handling
    # sm_model.history.redo()

    # reset observer and testbed
    state_dict['Nested'].remove_transition(new_trans_id)
    new_df_id = state_dict['Nested'].add_transition(from_state_id=state1.state_id,
                                                    from_outcome=outcome_again_state1,
                                                    to_state_id=state1.state_id,
                                                    to_outcome=None)
    sm_model.history.undo()
    sm_model.history.redo()

    ##### modify from parent state #######
    # modify_transition_from_state(self, transition_id, from_state, from_outcome)
    # state_dict['Nested'].modify_transition_from_state(new_df_id, from_state=state2.state_id,
    #                                                   from_outcome=oc_full_state2)
    state_dict['Nested'].transitions[new_df_id].modify_origin(state2.state_id, oc_full_state2)
    sm_model.history.undo()
    sm_model.history.redo()

    # modify_transition_from_outcome(self, transition_id, from_outcome)
    # state_dict['Nested'].modify_transition_from_outcome(new_df_id, from_outcome=oc_done_state2)
    state_dict['Nested'].transitions[new_df_id].from_outcome = oc_done_state2
    sm_model.history.undo()
    sm_model.history.redo()

    # TODO test the respective function in transition
    # modify_transition_to_outcome(self, transition_id, to_outcome)
    # Invalid test: to_outcome must be None as transition goes to child state
    # state_dict['Nested'].modify_transition_to_outcome(new_df_id, to_outcome=oc_great_nested)
    # sm_model.history.undo()
    # sm_model.history.redo()

    # modify_transition_to_state(self, transition_id, to_state, to_outcome)
    # state_dict['Nested'].modify_transition_to_state(new_df_id, to_state=state1.state_id)
    state_dict['Nested'].transitions[new_df_id].to_state = state1.state_id
    sm_model.history.undo()
    sm_model.history.redo()
    save_state_machine(sm_model, "/tmp/DFC_test/history_transition_properties", logger, with_gui=False)


def test_input_port_modify_notification(with_print=False):
    ##################
    # input_data_port properties

    # change name

    # change data_type

    # change default_value

    # change datatype

    # create testbed
    [logger, state, sm_model, state_dict] = create_models()
    new_input_data_port_id = state_dict['Nested2'].add_input_data_port(name='new_input', data_type='str')
    sm_model.history.undo()
    sm_model.history.redo()

    ################################
    # check for modification of name
    state_dict['Nested2'].input_data_ports[new_input_data_port_id].name = 'changed_new_input_name'
    sm_model.history.undo()
    sm_model.history.redo()

    #####################################
    # check for modification of data_type
    state_dict['Nested2'].input_data_ports[new_input_data_port_id].data_type = 'int'
    sm_model.history.undo()
    sm_model.history.redo()

    #########################################
    # check for modification of default_value
    state_dict['Nested2'].input_data_ports[new_input_data_port_id].default_value = 5
    sm_model.history.undo()
    sm_model.history.redo()

    ###########################################
    # check for modification of change_datatype
    state_dict['Nested2'].input_data_ports[new_input_data_port_id].change_data_type(data_type='str',
                                                                                    default_value='awesome_tool')
    sm_model.history.undo()
    sm_model.history.redo()
    save_state_machine(sm_model, "/tmp/DFC_test/history_input_port_properties", logger, with_gui=False)


def test_output_port_modify_notification(with_print=False):

    ##################
    # output_data_port properties

    # change name

    # change data_type

    # change default_value

    # change datatype

    # create testbed
    [logger, state, sm_model, state_dict] = create_models()
    new_output_data_port_id = state_dict['Nested2'].add_output_data_port(name='new_output', data_type='str')

    ################################
    # check for modification of name
    state_dict['Nested2'].output_data_ports[new_output_data_port_id].name = 'changed_new_output_name'
    sm_model.history.undo()
    sm_model.history.redo()

    #####################################
    # check for modification of data_type
    state_dict['Nested2'].output_data_ports[new_output_data_port_id].data_type = 'int'
    sm_model.history.undo()
    sm_model.history.redo()

    #########################################
    # check for modification of default_value
    state_dict['Nested2'].output_data_ports[new_output_data_port_id].default_value = 5
    sm_model.history.undo()
    sm_model.history.redo()

    ###########################################
    # check for modification of change_datatype
    state_dict['Nested2'].output_data_ports[new_output_data_port_id].change_data_type(data_type='str',
                                                                                      default_value='awesome_tool')
    sm_model.history.undo()
    sm_model.history.redo()
    save_state_machine(sm_model, "/tmp/DFC_test/history_output_port_properties", logger, with_gui=False)


def test_scoped_variable_modify_notification(with_print=False):
    ##################
    # scoped_variable properties

    # change name

    # change data_type

    # change default_value

    # change datatype

    # create testbed
    [logger, state, sm_model, state_dict] = create_models()
    new_scoped_variable_id = state_dict['Nested'].add_scoped_variable(name='new_output', data_type='str')

    ################################
    # check for modification of name
    # state_dict['Nested'].modify_scoped_variable_name('changed_new_scoped_var_name', new_scoped_variable_id)
    state_dict['Nested'].scoped_variables[new_scoped_variable_id].name = 'changed_new_scoped_var_name'
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    #####################################
    # check for modification of data_type
    state_dict['Nested'].scoped_variables[new_scoped_variable_id].data_type = 'int'
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    #########################################
    # check for modification of default_value
    state_dict['Nested'].scoped_variables[new_scoped_variable_id].default_value = 5
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    ###########################################
    # check for modification of change_datatype
    state_dict['Nested'].scoped_variables[new_scoped_variable_id].change_data_type(data_type='str',
                                                                                   default_value='awesome_tool')
    sm_model.history.undo()
    sm_model.history.redo()
    save_state_machine(sm_model, "/tmp/DFC_test/history_scoped_variable_properties", logger, with_gui=False)


def test_data_flow_property_changes_history(with_print=False):
    ##################
    # data_flow properties

    # change modify_origin

    # change from_key

    # change modify_target

    # change to_key

    # change data_flow_id

    # modify_transition_from_state

    # modify_transition_from_key

    # modify_transition_to_key

    # modify_transition_to_state

    # create testbed
    [logger, ctr_state, sm_model, state_dict] = create_models()

    state1 = ExecutionState('State1')
    output_state1 = state1.add_output_data_port("output", "int")
    input_state1 = state1.add_input_data_port("input", "str", "zero")
    state2 = ExecutionState('State2')
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    output_res_state2 = state2.add_output_data_port("res", "int")
    state_dict['Nested'].add_state(state1)
    state_dict['Nested'].add_state(state2)
    output_res_nested = state_dict['Nested'].add_output_data_port("res", "int")
    output_count_state1 = state1.add_output_data_port("count", "int")
    input_number_state2 = state2.add_input_data_port("number", "int", 5)

    new_df_id = state_dict['Nested'].add_data_flow(from_state_id=state2.state_id,
                                                   from_data_port_id=output_res_state2,
                                                   to_state_id=state_dict['Nested'].state_id,
                                                   to_data_port_id=output_res_nested)

    ##### modify from data_flow #######
    # modify_origin(self, from_state, from_key)
    state_dict['Nested'].data_flows[new_df_id].modify_origin(from_state=state1.state_id, from_key=output_state1)
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # from_key(self, from_key)
    state_dict['Nested'].data_flows[new_df_id].from_key = output_count_state1
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # modify_target(self, to_state, to_key)
    state_dict['Nested'].data_flows[new_df_id].modify_target(to_state=state2.state_id, to_key=input_par_state2)
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # to_key(self, to_key)
    state_dict['Nested'].data_flows[new_df_id].to_key = input_number_state2
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # data_flow_id(self, data_flow_id)
    # state_dict['Nested'].data_flows[new_df_id].data_flow_id += 1
    # # TODO ContainerState needs a modify data_flow_id for proper handling
    # sm_model.history.undo()
    # #sm_model.history.redo()

    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state
    new_df_id = state_dict['Nested'].data_flows[new_df_id].data_flow_id  # only if histroy.redo was not run

    # reset observer and testbed
    state_dict['Nested'].remove_data_flow(new_df_id)
    sm_model.history.undo()
    sm_model.history.redo()
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state
    new_df_id = state_dict['Nested'].add_data_flow(from_state_id=state2.state_id,
                                                   from_data_port_id=output_res_state2,
                                                   to_state_id=state_dict['Nested'].state_id,
                                                   to_data_port_id=output_res_nested)
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    ##### modify from parent state #######
    # modify_data_flow_from_state(self, data_flow_id, from_state, from_key)
    # state_dict['Nested'].modify_data_flow_from_state(new_df_id, from_state=state1.state_id, from_key=output_state1)
    state_dict['Nested'].data_flows[new_df_id].modify_origin(state1.state_id, output_state1)
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # modify_data_flow_from_key(self, data_flow_id, from_key)
    # state_dict['Nested'].modify_data_flow_from_key(new_df_id, from_key=output_count_state1)
    state_dict['Nested'].data_flows[new_df_id].from_key = output_count_state1
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # modify_data_flow_to_state(self, data_flow_id, to_state, to_key)
    # state_dict['Nested'].modify_data_flow_to_state(new_df_id, to_state=state2.state_id, to_key=input_par_state2)
    state_dict['Nested'].data_flows[new_df_id].modify_target(state2.state_id, input_par_state2)
    sm_model.history.undo()
    sm_model.history.redo()
    # resolve reference
    state_dict['Nested'] = sm_model.get_state_model_by_path(state_dict['Nested'].get_path()).state

    # modify_data_flow_to_key(self, data_flow_id, to_key)
    # state_dict['Nested'].modify_data_flow_to_key(new_df_id, to_key=input_number_state2)
    state_dict['Nested'].data_flows[new_df_id].to_key = input_number_state2
    sm_model.history.undo()
    sm_model.history.redo()

    save_state_machine(sm_model, "/tmp/DFC_test/history_data_flow_properties", logger, with_gui=False)


def setup_logger(logging_view):
    log.debug_filter.set_logging_test_view(logging_view)
    log.error_filter.set_logging_test_view(logging_view)


def test_type_changes_without_gui():

    with_gui = False

    rafcon.statemachine.singleton.state_machine_manager.delete_all_state_machines()
    os.chdir(rafcon.__path__[0] + "/mvc")
    gtk.rc_parse(rafcon.__path__[0] + "/mvc/themes/black/gtk-2.0/gtkrc")
    signal.signal(signal.SIGINT, rafcon.statemachine.singleton.signal_handler)
    global_config.load()  # load the default config
    global_gui_config.load()  # load the default config
    logging_view = LoggingView()
    setup_logger(logging_view)
    # time.sleep(1)
    print "create model"
    [logger, state, sm_m, state_dict] = create_models()
    print "init libs"
    rafcon.statemachine.singleton.library_manager.initialize()

    sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model

    # load the meta data for the state machine
    sm_manager_model.get_selected_state_machine_model().root_state.load_meta_data_for_state()

    # thread = threading.Thread(target=test_add_remove_history,
    #                           args=[True, variables_for_pytest.sm_manager_model, main_window_controller,
    #                                 sm_m, state_dict, with_gui])
    # time.sleep(1)
    print "start thread"
    trigger_state_type_change_tests(sm_manager_model, None, sm_m, state_dict, with_gui, logger)
    os.chdir(rafcon.__path__[0] + "/../test")


def test_state_machine_changes_with_gui(with_gui=True):

    test_multithrading_lock.acquire()
    rafcon.statemachine.singleton.state_machine_manager.delete_all_state_machines()
    os.chdir(rafcon.__path__[0] + "/mvc")
    gtk.rc_parse("./themes/black/gtk-2.0/gtkrc")
    signal.signal(signal.SIGINT, rafcon.statemachine.singleton.signal_handler)
    global_config.load()  # load the default config
    global_gui_config.load()  # load the default config
    logging_view = LoggingView()
    setup_logger(logging_view)
    time.sleep(1)
    print "create model"
    [logger, state, sm_m, state_dict] = create_models()
    print "init libs"
    rafcon.statemachine.singleton.library_manager.initialize()

    if variables_for_pytest.sm_manager_model is None:
            variables_for_pytest.sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model

    print "initialize MainWindow"
    main_window_view = MainWindowView(logging_view)

    # load the meta data for the state machine
    variables_for_pytest.sm_manager_model.get_selected_state_machine_model().root_state.load_meta_data_for_state()

    main_window_controller = MainWindowController(variables_for_pytest.sm_manager_model, main_window_view,
                                                  editor_type='LogicDataGrouped')

    # thread = threading.Thread(target=test_add_remove_history,
    #                           args=[True, variables_for_pytest.sm_manager_model, main_window_controller,
    #                                 sm_m, state_dict, with_gui])
    # time.sleep(1)
    print "start thread"
    thread = threading.Thread(target=trigger_state_type_change_tests,
                              args=[variables_for_pytest.sm_manager_model, main_window_controller,
                                    sm_m, state_dict, with_gui, logger])

    thread.start()

    if with_gui:
        gtk.main()
        logger.debug("Gtk main loop exited!")
        sm = rafcon.statemachine.singleton.state_machine_manager.get_active_state_machine()
        if sm:
            # sm.root_state.join()
            # logger.debug("Joined currently executing state machine!")
            thread.join()
            logger.debug("Joined test triggering thread!")
        os.chdir(rafcon.__path__[0] + "/../test")
        test_multithrading_lock.release()
    else:
        os.chdir(rafcon.__path__[0] + "/../test")
        thread.join()


def trigger_state_type_change_tests(*args):
    print "Wait for the gui to initialize"
    time.sleep(1.0)
    sm_manager_model = args[0]
    main_window_controller = args[1]
    sm_m = args[2]
    state_dict = args[3]
    with_gui = args[4]
    logger = args[5]

    sleep_time_short = 3
    sleep_time_max = 5  # 0.5

    time.sleep(sleep_time_short)

    ####### General Type Change inside of a state machine (NO ROOT STATE) ############
    state_of_type_change = 'State3'
    parent_of_type_change = 'Container'

    # do state_type_change with gui
    # - find state machine id
    my_sm_id = None
    for sm_id, state_machine in sm_manager_model.state_machine_manager.state_machines.iteritems():
        if state_machine is sm_m.state_machine:
            my_sm_id = sm_id
    assert my_sm_id is not None

    sm_model = sm_m  # sm_manager_model.state_machines[my_sm_id]
    sm_model.history.changes.reset()

    state_parent_m = sm_m.get_state_model_by_path(state_dict[parent_of_type_change].get_path())
    state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements, stored_state_m_elements] = store_state_elements(state_dict[state_of_type_change], state_m)
    print "\n\n %s \n\n" % state_m.state.name
    # call_gui_callback(sm_m.selection.set, [state_m])
    sm_m.selection.set([state_m])
    # time.sleep(sleep_time_short)

    # adjust the transition test
    # TODO finish it

    list_store_id_from_state_type_dict = {}
    state_editor_ctrl = None
    state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements, stored_state_m_elements] = store_state_elements(state_dict[state_of_type_change], state_m)

    if with_gui:
        state_machine_path = '/tmp/dfc_test_state_type_change_history_with_gui'
        # TODO the next lines should not to be necessary to save the statemachine at the end
        menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
        save_state_machine(sm_model, state_machine_path, logger, with_gui, menubar_ctrl)

        # do state_type_change with gui
        # - get state-editor controller and find right row in combo box
        [state_editor_ctrl, list_store_id_from_state_type_dict] = \
            get_state_editor_ctrl_and_store_id_dict(sm_m, state_m, main_window_controller, sleep_time_max, logger)
    else:
        menubar_ctrl = None
        state_machine_path = '/tmp/dfc_test_state_type_change_history_without_gui'
        save_state_machine(sm_model, state_machine_path, logger, with_gui, menubar_ctrl)
        pass
    if with_gui:
        check_state_editor_models(sm_m, state_m, main_window_controller, logger)

    # HS -> BCS
    save_state_machine(sm_model, state_machine_path + '_before1', logger, with_gui, menubar_ctrl)
    # - do state type change
    if with_gui:
        state_type_row_id = list_store_id_from_state_type_dict['BARRIER_CONCURRENCY']
        call_gui_callback(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    else:
        state_dict[parent_of_type_change].change_state_type(state_m, BarrierConcurrencyState)

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    new_state = sm_model.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_model.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements_after, stored_state_m_elements_after] = store_state_elements(new_state, new_state_m)

    save_state_machine(sm_model, state_machine_path + '_after1', logger, with_gui, menubar_ctrl)

    assert len(sm_model.history.changes.single_trail_history()) == 1
    if with_gui:
        call_gui_callback(sm_model.history.undo)
    else:
        sm_model.history.undo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    save_state_machine(sm_model, state_machine_path + '_undo1', logger, with_gui, menubar_ctrl)

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_HS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)
    if with_gui:
        check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    if with_gui:
        call_gui_callback(sm_model.history.redo)
    else:
        sm_model.history.redo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    save_state_machine(sm_model, state_machine_path + '_redo1', logger, with_gui, menubar_ctrl)

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_BCS, new_state, new_state_m, stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    # BCS -> HS
    [stored_state_elements, stored_state_m_elements] = store_state_elements(new_state, new_state_m)
    save_state_machine(sm_model, state_machine_path + '_before2', logger, with_gui, menubar_ctrl)
    sm_model.state_machine.file_system_path = state_machine_path
    if with_gui:
        # do state_type_change with gui
        # - get state-editor controller and find right row in combo box
        [state_editor_ctrl, list_store_id_from_state_type_dict] = \
            get_state_editor_ctrl_and_store_id_dict(sm_m, new_state_m, main_window_controller, sleep_time_max, logger)

    # - do state type change
        state_type_row_id = list_store_id_from_state_type_dict['HIERARCHY']
        call_gui_callback(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    else:
        state_dict[parent_of_type_change].change_state_type(new_state_m, HierarchyState)

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements_after, stored_state_m_elements_after] = store_state_elements(new_state, new_state_m)

    print "\n\n ###### State: %s" % state_dict[state_of_type_change]
    from test_z_storage import check_that_all_files_are_there
    check_that_all_files_are_there(sm_model, state_machine_path + '_before2', False, True)
    check_that_all_files_are_there(sm_model, state_machine_path + '_after2', False, True)

    save_state_machine(sm_model, state_machine_path + '_after2', logger, with_gui, menubar_ctrl)

    assert len(sm_model.history.changes.single_trail_history()) == 2
    if with_gui:
        call_gui_callback(sm_model.history.undo)
    else:
        sm_model.history.undo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())
    if with_gui:
        check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    save_state_machine(sm_model, state_machine_path + '_undo2', logger, with_gui, menubar_ctrl)

    if with_gui:
        call_gui_callback(sm_model.history.redo)
    else:
        sm_model.history.redo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    save_state_machine(sm_model, state_machine_path + '_redo2', logger, with_gui, menubar_ctrl)

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_HS, new_state, new_state_m, stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    # HS -> PCS
    [stored_state_elements, stored_state_m_elements] = store_state_elements(new_state, new_state_m)
    save_state_machine(sm_model, state_machine_path + '_before3', logger, with_gui, menubar_ctrl)
    if with_gui:
        # do state_type_change with gui
        # - get state-editor controller and find right row in combo box
        [state_editor_ctrl, list_store_id_from_state_type_dict] = \
            get_state_editor_ctrl_and_store_id_dict(sm_m, new_state_m, main_window_controller, sleep_time_max, logger)

    # - do state type change
        state_type_row_id = list_store_id_from_state_type_dict['PREEMPTION_CONCURRENCY']
        call_gui_callback(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    else:
        state_dict[parent_of_type_change].change_state_type(new_state_m, PreemptiveConcurrencyState)

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    new_state = sm_model.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_model.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements_after, stored_state_m_elements_after] = store_state_elements(new_state, new_state_m)

    save_state_machine(sm_model, state_machine_path + '_after3', logger, with_gui, menubar_ctrl)

    # assert len(sm_model.history.changes.single_trail_history()) == 3
    if with_gui:
        call_gui_callback(sm_model.history.undo)
    else:
        sm_model.history.undo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())
    if with_gui:
        check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    save_state_machine(sm_model, state_machine_path + '_undo3', logger, with_gui, menubar_ctrl)

    new_state = sm_model.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_model.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_HS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)

    if with_gui:
        call_gui_callback(sm_model.history.redo)
    else:
        sm_model.history.redo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    save_state_machine(sm_model, state_machine_path + '_redo3', logger, with_gui, menubar_ctrl)

    new_state = sm_model.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_model.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_PCS, new_state, new_state_m, stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    # PCS -> ES
    [stored_state_elements, stored_state_m_elements] = store_state_elements(new_state, new_state_m)
    save_state_machine(sm_model, state_machine_path + '_before4', logger, with_gui, menubar_ctrl)
    if with_gui:
        # do state_type_change with gui
        # - get state-editor controller and find right row in combo box
        [state_editor_ctrl, list_store_id_from_state_type_dict] = \
            get_state_editor_ctrl_and_store_id_dict(sm_m, new_state_m, main_window_controller, sleep_time_max, logger)

    # - do state type change
        state_type_row_id = list_store_id_from_state_type_dict['EXECUTION']
        call_gui_callback(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    else:
        state_dict[parent_of_type_change].change_state_type(new_state_m, ExecutionState)

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    new_state = sm_model.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_model.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements_after, stored_state_m_elements_after] = store_state_elements(new_state, new_state_m)

    save_state_machine(sm_model, state_machine_path + '_after4', logger, with_gui, menubar_ctrl)

    # assert len(sm_model.history.changes.single_trail_history()) == 4
    if with_gui:
        call_gui_callback(sm_model.history.undo)
    else:
        sm_model.history.undo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())
    if with_gui:
        check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    save_state_machine(sm_model, state_machine_path + '_undo4', logger, with_gui, menubar_ctrl)

    new_state = sm_model.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_model.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_PCS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)

    if with_gui:
        call_gui_callback(sm_model.history.redo)
    else:
        sm_model.history.redo()

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    state_dict[parent_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[parent_of_type_change].get_path())

    save_state_machine(sm_model, state_machine_path + '_redo4', logger, with_gui, menubar_ctrl)

    new_state = sm_model.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_model.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_ES, new_state, new_state_m, stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    # # TODO do the check for the root-state too!!!
    # save_and_quit(sm_model, '/tmp/dfc_test_state_type_change_history', menubar_ctrl, with_gui)
    # # return

    ####### General Type Change as ROOT STATE ############
    state_of_type_change = 'Container'
    state_m = sm_model.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements, stored_state_m_elements] = store_state_elements(state_dict[state_of_type_change], state_m)
    if with_gui:
        check_state_editor_models(sm_m, state_m, main_window_controller, logger)

    # HS -> BCS
    if with_gui:
        # do state_type_change with gui
        # - do state selection to generate state editor widget
        call_gui_callback(sm_m.selection.set, [state_m])
        call_gui_callback(sm_m.selection.clear)
        call_gui_callback(sm_m.selection.set, [state_m])

        # - get state-editor controller and find right row in combo box
        [state_editor_ctrl, list_store_id_from_state_type_dict] = \
            get_state_editor_ctrl_and_store_id_dict(sm_m, state_m, main_window_controller, sleep_time_max, logger)

    # - do state type change
        state_type_row_id = list_store_id_from_state_type_dict['BARRIER_CONCURRENCY']
        call_gui_callback(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    else:
        sm_m.state_machine.change_root_state_type(state_m, BarrierConcurrencyState)
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements_after, stored_state_m_elements_after] = store_state_elements(new_state, new_state_m)

    # assert len(sm_model.history.changes.single_trail_history()) == 1
    if with_gui:
        call_gui_callback(sm_model.history.undo)
    else:
        sm_model.history.undo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_HS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)
    if with_gui:
        check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    if with_gui:
        call_gui_callback(sm_model.history.redo)
    else:
        sm_model.history.redo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_BCS, new_state, new_state_m,
                         stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    # BCS -> HS
    [stored_state_elements, stored_state_m_elements] = store_state_elements(new_state, new_state_m)
    if with_gui:
        # do state_type_change with gui
        # - do state selection to generate state editor widget
        call_gui_callback(sm_m.selection.set, [new_state_m])
        call_gui_callback(sm_m.selection.clear)
        call_gui_callback(sm_m.selection.set, [new_state_m])

        # - get state-editor controller and find right row in combo box
        [state_editor_ctrl, list_store_id_from_state_type_dict] = \
            get_state_editor_ctrl_and_store_id_dict(sm_m, new_state_m, main_window_controller, sleep_time_max, logger)

    # - do state type change
        state_type_row_id = list_store_id_from_state_type_dict['HIERARCHY']
        call_gui_callback(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    else:
        sm_m.state_machine.change_root_state_type(new_state_m, HierarchyState)

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    # assert len(sm_model.history.changes.single_trail_history()) == 2
    if with_gui:
        call_gui_callback(sm_model.history.undo)
    else:
        sm_model.history.undo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_BCS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)
    if with_gui:
        check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    if with_gui:
        call_gui_callback(sm_model.history.redo)
    else:
        sm_model.history.redo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_HS, new_state, new_state_m,
                         stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    # HS -> PCS
    [stored_state_elements, stored_state_m_elements] = store_state_elements(new_state, new_state_m)
    if with_gui:
        # do state_type_change with gui
        # - do state selection to generate state editor widget
        call_gui_callback(sm_m.selection.set, [new_state_m])
        call_gui_callback(sm_m.selection.clear)
        call_gui_callback(sm_m.selection.set, [new_state_m])

        # - get state-editor controller and find right row in combo box
        [state_editor_ctrl, list_store_id_from_state_type_dict] = \
            get_state_editor_ctrl_and_store_id_dict(sm_m, new_state_m, main_window_controller, sleep_time_max, logger)

    # - do state type change
        state_type_row_id = list_store_id_from_state_type_dict['PREEMPTION_CONCURRENCY']
        call_gui_callback(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    else:
        sm_m.state_machine.change_root_state_type(new_state_m, PreemptiveConcurrencyState)
        state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    # assert len(sm_model.history.changes.single_trail_history()) == 3
    if with_gui:
        call_gui_callback(sm_model.history.undo)
    else:
        sm_model.history.undo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_PCS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)
    if with_gui:
        check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    if with_gui:
        call_gui_callback(sm_model.history.redo)
    else:
        sm_model.history.redo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_HS, new_state, new_state_m,
                         stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    # PCS -> ES
    [stored_state_elements, stored_state_m_elements] = store_state_elements(new_state, new_state_m)
    if with_gui:
        # do state_type_change with gui
        # - do state selection to generate state editor widget
        call_gui_callback(sm_m.selection.set, [new_state_m])
        call_gui_callback(sm_m.selection.clear)
        call_gui_callback(sm_m.selection.set, [new_state_m])

        # - get state-editor controller and find right row in combo box
        [state_editor_ctrl, list_store_id_from_state_type_dict] = \
            get_state_editor_ctrl_and_store_id_dict(sm_m, new_state_m, main_window_controller, sleep_time_max, logger)

    # - do state type change
        state_type_row_id = list_store_id_from_state_type_dict['EXECUTION']
        call_gui_callback(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    else:
        sm_m.state_machine.change_root_state_type(new_state_m, ExecutionState)

    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    [stored_state_elements_after, stored_state_m_elements_after] = store_state_elements(new_state, new_state_m)

    # assert len(sm_model.history.changes.single_trail_history()) == 4
    if with_gui:
        call_gui_callback(sm_model.history.undo)
    else:
        sm_model.history.undo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_PCS, new_state, new_state_m, stored_state_elements, stored_state_m_elements)
    if with_gui:
        check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    if with_gui:
        call_gui_callback(sm_model.history.redo)
    else:
        sm_model.history.redo()
    state_dict[state_of_type_change] = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())

    new_state = sm_m.state_machine.get_state_by_path(state_dict[state_of_type_change].get_path())
    new_state_m = sm_m.get_state_model_by_path(state_dict[state_of_type_change].get_path())
    check_state_elements(check_list_root_ES, new_state, new_state_m,
                         stored_state_elements_after, stored_state_m_elements_after)
    if with_gui:
        check_state_editor_models(sm_m, new_state_m, main_window_controller, logger)

    save_and_quit(sm_model, '/tmp/dfc_test_state_type_change_history', menubar_ctrl, with_gui)


if __name__ == '__main__':
    # pytest.main()
    with_prints = False
    test_add_remove_history(with_prints)
    test_state_property_changes_history(with_prints)

    test_outcome_property_changes_history(with_prints)
    test_transition_property_changes_history(with_prints)

    test_input_port_modify_notification(with_prints)
    test_output_port_modify_notification(with_prints)
    test_scoped_variable_modify_notification(with_prints)

    test_data_flow_property_changes_history(with_prints)

    test_type_changes_without_gui()

    test_state_machine_changes_with_gui()
