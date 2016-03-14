import os
import gtk
import signal
import time
import logging

# general tool elements
from rafcon.utils import log

# core elements
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.container_state import ContainerState
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.state_machine import StateMachine

from rafcon.mvc.controllers.main_window import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView

# singleton elements
import rafcon.statemachine.singleton
import rafcon.mvc.singleton

# test environment elements
import testing_utils
from testing_utils import test_multithrading_lock, call_gui_callback, get_unique_temp_path
import pytest


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

    # return ctr_state, sm_m, state_dict
    return logger, ctr_state, sm_m, state_dict


def on_save_activate(state_machine_m, logger):
        if state_machine_m is None:
            return
        save_path = state_machine_m.state_machine.base_path
        if save_path is None:
            return

        logger.debug("Saving state machine to {0}".format(save_path))
        rafcon.statemachine.singleton.global_storage.save_statemachine_to_path(
            state_machine_m.state_machine,
            state_machine_m.state_machine.base_path, delete_old_state_machine=False)

        state_machine_m.root_state.store_meta_data()
        logger.debug("Successfully saved graphics meta data.")


def save_state_machine(sm_model, path, logger, with_gui, menubar_ctrl):
    # sleep_time_short = 0.5
    if with_gui:
        sm_model.state_machine.base_path = path
        # time.sleep(sleep_time_short)
        call_gui_callback(menubar_ctrl.on_save_as_activate, None, None, sm_model.state_machine.base_path)
        # time.sleep(sleep_time_short)
        call_gui_callback(menubar_ctrl.on_quit_activate, None)
    else:
        sm_model.state_machine.base_path = path
        # time.sleep(sleep_time_short)
        on_save_activate(sm_model, logger)
        # time.sleep(sleep_time_short)


def check_file(file_path, kind, missing_elements=None, actual_exists=None):
    if os.path.isfile(file_path):
        print "%s: '%s' exists" % (kind, file_path)
        if actual_exists is not None:
            actual_exists.append(file_path)
        return True
    else:
        print "%s: '%s' DON'T exists" % (kind, file_path)
        if missing_elements is not None:
            missing_elements.append(file_path)
        return False


def check_folder(folder_path, kind, missing_elements=None, actual_exists=None):
    if os.path.exists(folder_path):
        print "%s: '%s' exists" % (kind, folder_path)
        if actual_exists is not None:
            actual_exists.append(folder_path)
        return True
    else:
        print "%s: '%s' DON'T exists" % (kind, folder_path)
        if missing_elements is not None:
            missing_elements.append(folder_path)
        return False


def check_state_storage(state, parent_path, missing_elements, check_gui_meta_data=False, actual_exists=None):

    # check state folder exists
    folder_path = parent_path + "/" + state.state_id
    check_folder(folder_path, "state_path", missing_elements, actual_exists)

    # check state script exists
    if isinstance(state, ExecutionState):
        file_path = parent_path + "/" + state.state_id + "/" + "script.py"
        check_file(file_path, "script", missing_elements, actual_exists)

    # check state-meta data exists (transitions and so on)
    file_path = parent_path + "/" + state.state_id + "/" + "meta.yaml"
    check_file(file_path, "state_meta.yaml", missing_elements, actual_exists)

    # check if optional gui-meta-data exists
    if check_gui_meta_data:
        # gtk gui meta data
        file_path = parent_path + "/" + state.state_id + "/" + "gtk_meta.yaml"
        check_file(file_path, "gtk_gui_meta.yaml", missing_elements, actual_exists)

    if isinstance(state, ContainerState):
        for key, child_state in state.states.iteritems():
            check_state_storage(child_state, folder_path, missing_elements, check_gui_meta_data, actual_exists)


def check_that_all_files_are_there(sm_m, base_path=None, check_gui_meta_data=False, with_print=False, old_exists=None, old_base_path=None):
    root_state = sm_m.state_machine.root_state
    base_path = sm_m.state_machine.file_system_path
    missing_elements = []
    actual_exists = []
    check_folder(base_path, "statemachine_root_state_base_path", missing_elements, actual_exists)

    check_state_storage(root_state, base_path, missing_elements, check_gui_meta_data, actual_exists)
    if old_exists is not None and old_base_path:
        old_without_base = [old_path.replace(old_base_path, "") for old_path in old_exists]
    else:
        old_without_base = None
    if with_print and missing_elements:
        print 30*"#"
        print "statemachine %s with root_state.state_id %s MISSING the following FILES" % (sm_m.state_machine.state_machine_id, root_state.state_id)
        print 30*"#"
        for path in missing_elements:
            if old_without_base is not None and path.replace(base_path, "") in old_without_base:
                print path, " ... but exists in ", old_base_path
            else:
                print path, " ... but does not exist before"
    else:
        print 30*"#"
        print "All Files and Folders where Found of statemachine %s with root_state.state_id %s" % (sm_m.state_machine.state_machine_id, root_state.state_id)
        print 30*"#"

    return missing_elements, actual_exists


def test_storage_without_gui(caplog):
    with_gui=False

    print "create model"
    [logger, state, sm_m, state_dict] = create_models()
    print "init libs"
    testing_utils.remove_all_libraries()
    rafcon.statemachine.singleton.library_manager.initialize()
    save_state_machine(sm_model=sm_m, path=get_unique_temp_path(), logger=logger, with_gui=with_gui,
                       menubar_ctrl=None)

    missing_elements = check_that_all_files_are_there(sm_m, with_print=True)
    testing_utils.reload_config()
    testing_utils.assert_logger_warnings_and_errors(caplog)


def test_storage_with_gui(caplog):
    with_gui = True

    test_multithrading_lock.acquire()
    rafcon.statemachine.singleton.state_machine_manager.delete_all_state_machines()
    os.chdir(rafcon.__path__[0] + "/mvc")
    gtk.rc_parse("./themes/dark/gtk-2.0/gtkrc")
    signal.signal(signal.SIGINT, rafcon.statemachine.singleton.signal_handler)
    print "create model"
    [logger, state, sm_m, state_dict] = create_models()
    print "init libs"
    testing_utils.remove_all_libraries()
    rafcon.statemachine.singleton.library_manager.initialize()

    if testing_utils.sm_manager_model is None:
        testing_utils.sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model
    print "initialize MainWindow"
    main_window_view = MainWindowView()

    main_window_controller = MainWindowController(testing_utils.sm_manager_model, main_window_view,
                                                  editor_type='LogicDataGrouped')

    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
    print "start thread"
    import threading
    thread = threading.Thread(target=save_state_machine,
                              args=[sm_m, get_unique_temp_path(), logger, with_gui, menubar_ctrl])
    thread.start()

    if with_gui:
        gtk.main()
        logger.debug("Gtk main loop exited!")
        test_multithrading_lock.release()

    thread.join()

    missing_elements = check_that_all_files_are_there(sm_m, with_print=True)
    os.chdir(rafcon.__path__[0] + "/../test/common")
    testing_utils.assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    pytest.main([__file__])