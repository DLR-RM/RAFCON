import sys
import logging
import gtk
import threading
import time
import glib
import os
from os.path import dirname, join

# general tool elements
from rafcon.utils import log

# core elements
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.state_machine import StateMachine

# mvc elements
from rafcon.mvc.models import GlobalVariableManagerModel
from rafcon.mvc.controllers import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView

# singleton elements
import rafcon.mvc.singleton

# test environment elements
import test_utils
from test_utils import call_gui_callback
import pytest


def setup_module(module):
    # set the test_libraries path temporarily to the correct value
    test_utils.remove_all_libraries()
    library_paths = rafcon.statemachine.config.global_config.get_config_value("LIBRARY_PATHS")
    print "File: ", dirname(__file__), dirname(dirname(__file__))

    library_paths["ros"] = join(rafcon.__path__[0] + "/..", "test_scripts", "ros_libraries")
    library_paths["turtle_libraries"] = join(rafcon.__path__[0] + "/..", "test_scripts", "turtle_libraries")


def create_models(*args, **kargs):
    logger = log.get_logger(__name__)
    logger.setLevel(logging.DEBUG)
    #logging.getLogger('gtkmvc').setLevel(logging.DEBUG)
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)
    stdout = logging.StreamHandler(sys.stdout)
    stdout.setFormatter(logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s"))
    stdout.setLevel(logging.DEBUG)
    logging.getLogger('gtkmvc').addHandler(stdout)
    logging.getLogger('statemachine.state').setLevel(logging.DEBUG)
    logging.getLogger('controllers.state_properties').setLevel(logging.DEBUG)

    state1 = ExecutionState('State1')
    output_state1 = state1.add_output_data_port("output", "int")
    input_state1 = state1.add_input_data_port("input", "str", "zero")
    state2 = ExecutionState('State2')
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    output_res_state2 = state2.add_output_data_port("res", "int")
    state4 = ExecutionState('Nested')
    output_state4 = state4.add_output_data_port("out", "int")
    state5 = ExecutionState('Nested2')
    input_state5 = state5.add_input_data_port("in", "int", 0)
    state3 = HierarchyState(name='State3')
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

    ctr_state = HierarchyState(name="Container")
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
    # this is not allowed as the output port is already connected
    ctr_state.add_data_flow(state1.state_id, output_state1, ctr_state.state_id, scoped_variable3_ctr_state)

    global_var_manager_model = GlobalVariableManagerModel()
    global_var_manager_model.global_variable_manager.set_variable("global_variable_1", "value1")
    global_var_manager_model.global_variable_manager.set_variable("global_variable_2", "value2")

    return state5, logger, ctr_state, global_var_manager_model


def wait_for_values_identical_number_state_machines(sm_manager_model, val2):
    values_identical = len(sm_manager_model.state_machines) == val2
    counter = 0
    while not values_identical:
        time.sleep(0.1)
        values_identical = len(sm_manager_model.state_machines) == val2
        counter += 1
        if counter == 50:
            break


def trigger_gui_signals(*args):
    """ The function triggers and test basic functions of the menu bar.
    At the moment those functions are tested:
    - New State Machine
    - Open State Machine
    - Copy State/HierarchyState -> via GraphicalEditor
    - Cut State/HierarchyState -> via GraphicalEditor
    - Paste State/HierarchyState -> via GraphicalEditor
    - Refresh Libraries
    - Refresh All
    - Save as
    - Stop State Machine
    - Quit GUI
    """

    print "Wait for the gui to initialize"
    time.sleep(2.0)
    sm_manager_model = args[0]
    main_window_controller = args[1]
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')

    current_sm_length = len(sm_manager_model.state_machines)
    # print "1:", sm_manager_model.state_machines.keys()
    first_sm_id = sm_manager_model.state_machines.keys()[0]
    call_gui_callback(menubar_ctrl.on_new_activate, None)
    # print "2:", sm_manager_model.state_machines.keys()

    # wait_for_values_identical_number_state_machines(sm_manager_model, current_sm_length+1)
    assert len(sm_manager_model.state_machines) == current_sm_length+1

    call_gui_callback(menubar_ctrl.on_open_activate, None, None, rafcon.__path__[0] + "/../test_scripts/tutorials/basic_turtle_demo_sm")
    # wait_for_values_identical_number_state_machines(sm_manager_model, current_sm_length+2)
    assert len(sm_manager_model.state_machines) == current_sm_length+2

    # print "3:", sm_manager_model.state_machines.keys(), first_sm_id+2
    sleep_time_short = 1.0
    sm_m = sm_manager_model.state_machines[first_sm_id+2]
    sm_m.history.fake = True
    time.sleep(sleep_time_short)
    # print "focus"
    # time.sleep(5)
    # MAIN_WINDOW NEEDS TO BE FOCUSED (for global input focus) TO OPERATE PASTE IN GRAPHICAL VIEWER
    main_window_controller.view['main_window'].grab_focus()
    sm_manager_model.selected_state_machine_id = first_sm_id+2
    state_machines_ctrl = main_window_controller.get_controller('state_machines_editor_ctrl')
    page_id = state_machines_ctrl.get_page_id(first_sm_id+2)
    page = state_machines_ctrl.view.notebook.get_nth_page(page_id)
    page.children()[0].grab_focus()

    time.sleep(sleep_time_short)
    #########################################################
    # select a execution state -> and paste it some where

    state_m = sm_m.get_state_model_by_path('CDMJPK/RMKGEW/KYENSZ/UEPNNW')
    print "\n\n %s \n\n" % state_m.state.name
    # print "set"
    call_gui_callback(sm_m.selection.set, [state_m])
    # time.sleep(3)
    # print "copy"
    # copy the state to clipboard
    call_gui_callback(menubar_ctrl.on_copy_selection_activate, None, None)
    # time.sleep(3)
    # print "set"
    # select other state
    state_m = sm_m.get_state_model_by_path('CDMJPK/RMKGEW')
    print state_m.state.states.keys()
    print "\n\n %s \n\n" % state_m.state.name
    call_gui_callback(sm_m.selection.set, [state_m])
    # time.sleep(3)
    old_child_state_count = len(state_m.state.states)
    # print "focus"
    # paste clipboard element into the new state
    main_window_controller.view['main_window'].grab_focus()  # refresh focus
    page.children()[0].grab_focus()
    # time.sleep(3)
    # print "paste"
    # print dir(page.children()[0]), "\n\n", page.children()[0], "\n\n", page.children()[0].has_focus()
    call_gui_callback(menubar_ctrl.on_paste_clipboard_activate, None, None)
    state_m = sm_m.get_state_model_by_path('CDMJPK/RMKGEW')
    print state_m.state.states.keys()
    # IN CASE OF ASSERTION SECURE FOCUS FOR MAIN MAIN WINDOW !!!!
    assert len(state_m.state.states) == old_child_state_count + 1

    ###########################################################
    # select a hierarchy state -> and paste it some where
    sm_m = sm_manager_model.state_machines[first_sm_id+2]
    state_m = sm_m.get_state_model_by_path('CDMJPK/RMKGEW/KYENSZ/VCWTIY')
    print "\n\n %s \n\n" % state_m.state.name
    # glib.idle_add(sm_m.selection.set, [state_m])
    call_gui_callback(sm_m.selection.set, [state_m])
    # time.sleep(sleep_time_short)

    # copy the state to clipboard
    # glib.idle_add(menubar_ctrl.on_copy_selection_activate, None, None)
    call_gui_callback(menubar_ctrl.on_copy_selection_activate, None, None)
    # global_clipboard.copy(sm_m.selection)
    # time.sleep(sleep_time_short)

    # select other state
    state_m = sm_m.get_state_model_by_path('CDMJPK')
    old_child_state_count = len(state_m.state.states)
    print "\n\n %s \n\n" % state_m.state.name
    # glib.idle_add(sm_m.selection.set, [state_m])
    call_gui_callback(sm_m.selection.set, [state_m])
    # time.sleep(sleep_time_short)

    # paste clipboard element into the new state
    main_window_controller.view['main_window'].grab_focus()  # refresh focus
    page.children()[0].grab_focus()
    glib.idle_add(menubar_ctrl.on_paste_clipboard_activate, None, None)
    # global_clipboard.paste(state_m)  # sm_m.selection)
    time.sleep(sleep_time_short)

    # verify
    state_m = sm_m.get_state_model_by_path('CDMJPK')
    print state_m.state.states.keys()
    # IN CASE OF ASSERTION SECURE FOCUS FOR MAIN MAIN WINDOW !!!!
    assert len(state_m.state.states) == old_child_state_count + 1

    ##########################################################
    # select a library state -> and paste it some where WITH CUT !!!
    sm_m = sm_manager_model.state_machines[first_sm_id+2]
    state_m = sm_m.get_state_model_by_path('CDMJPK/RMKGEW/KYENSZ/VCWTIY')
    print "\n\n %s \n\n" % state_m.state.name
    # glib.idle_add(sm_m.selection.set, [state_m])
    call_gui_callback(sm_m.selection.set, [state_m])
    # time.sleep(sleep_time_short)

    # cut the state to clipboard
    # glib.idle_add(menubar_ctrl.on_copy_selection_activate, None, None)
    # glib.idle_add(menubar_ctrl.on_cut_selection_activate, None, None)
    call_gui_callback(menubar_ctrl.on_cut_selection_activate, None, None)

    # select other state
    state_m = sm_m.get_state_model_by_path('CDMJPK')
    old_child_state_count = len(state_m.state.states)
    print "\n\n %s \n\n" % state_m.state.name
    # glib.idle_add(sm_m.selection.set, [state_m])
    call_gui_callback(sm_m.selection.set, [state_m])
    # time.sleep(sleep_time_short)

    # paste clipboard element into the new state
    main_window_controller.view['main_window'].grab_focus()  # refresh focus
    page.children()[0].grab_focus()
    # glib.idle_add(menubar_ctrl.on_paste_clipboard_activate, None, None)
    call_gui_callback(menubar_ctrl.on_paste_clipboard_activate, None, None)
    # global_clipboard.paste(state_m)  # sm_m.selection)
    # time.sleep(sleep_time_short)

    # verify
    state_m = sm_m.get_state_model_by_path('CDMJPK')
    print state_m.state.states.keys()
    # IN CASE OF ASSERTION SECURE FOCUS FOR MAIN MAIN WINDOW !!!!
    assert len(state_m.state.states) == old_child_state_count + 1
    ##########################################################

    call_gui_callback(menubar_ctrl.on_refresh_libraries_activate, None)
    call_gui_callback(menubar_ctrl.on_refresh_all_activate, None, None, True)

    # wait_for_values_identical_number_state_machines(sm_manager_model, 1)
    assert len(sm_manager_model.state_machines) == 1

    call_gui_callback(menubar_ctrl.on_save_as_activate, None, None, "/tmp")

    call_gui_callback(menubar_ctrl.on_stop_activate, None)
    call_gui_callback(menubar_ctrl.on_quit_activate, None)


def test_gui(caplog):
    test_utils.test_multithrading_lock.acquire()
    # delete all old state machines
    rafcon.statemachine.singleton.state_machine_manager.delete_all_state_machines()
    os.chdir(test_utils.RAFCON_PATH + "/mvc/")
    gtk.rc_parse("./themes/black/gtk-2.0/gtkrc")
    rafcon.statemachine.singleton.library_manager.initialize()
    [execution_state, logger, ctr_state, gvm_model] = create_models()

    state_machine = StateMachine(ctr_state)
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    test_utils.sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model
    main_window_view = MainWindowView()
    main_window_controller = MainWindowController(test_utils.sm_manager_model, main_window_view,
                                                  editor_type='LogicDataGrouped')

    thread = threading.Thread(target=trigger_gui_signals, args=[test_utils.sm_manager_model,
                                                                main_window_controller])
    thread.start()

    gtk.main()
    logger.debug("after gtk main")
    os.chdir(test_utils.RAFCON_PATH + "/../test/common")
    test_utils.assert_logger_warnings_and_errors(caplog)
    test_utils.test_multithrading_lock.release()


if __name__ == '__main__':
    pytest.main([__file__])
