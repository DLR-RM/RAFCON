import pytest
import sys
import logging
import gtk
import threading
import time
import os
import datetime

# mvc elements
import rafcon.mvc.config as gui_config
import rafcon.mvc.singleton
from rafcon.mvc.models.global_variable_manager import GlobalVariableManagerModel
from rafcon.mvc.controllers.main_window import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView

# statemachine elements
from rafcon.statemachine.singleton import state_machine_execution_engine
from rafcon.statemachine.storage import storage
from rafcon.statemachine.execution.state_machine_status import StateMachineExecutionStatus

# general tool elements
from rafcon.utils import log

# test environment elements
import testing_utils
from testing_utils import call_gui_callback


def create_models():
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

    global_var_manager_model = GlobalVariableManagerModel()
    global_var_manager_model.global_variable_manager.set_variable("global_variable_1", "value1")
    global_var_manager_model.global_variable_manager.set_variable("global_variable_2", "value2")

    return logger, global_var_manager_model


def wait_for_execution_engine_sync_counter(target_value, logger, timeout=5):
    logger.debug("++++++++++ waiting for execution engine sync for " + str(target_value) + " steps ++++++++++")
    current_time = datetime.datetime.now()
    # time.sleep(0.3)
    while True:
        state_machine_execution_engine.synchronization_lock.acquire()
        if state_machine_execution_engine.synchronization_counter == target_value:
            state_machine_execution_engine.synchronization_counter = 0
            state_machine_execution_engine.synchronization_lock.release()
            break
        state_machine_execution_engine.synchronization_lock.release()
        if (datetime.datetime.now() - current_time).seconds > timeout:
            raise RuntimeError("Something went wrong")
        time.sleep(0.1)


@log.log_exceptions(None, gtk_quit=True)
def trigger_gui_signals_library_state(*args):
    main_window_controller = args[0]
    logger = args[1]
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')

    # reset the synchronization counter; although the tests run in different processes they share their memory
    # as the import statements are at the top of the file and not inside the parallel called functions
    state_machine_execution_engine.synchronization_lock.acquire()
    state_machine_execution_engine.synchronization_counter = 0
    state_machine_execution_engine.synchronization_lock.release()

    call_gui_callback(menubar_ctrl.on_step_mode_activate, None, None)
    # Wait for GUI to initialize
    while gtk.events_pending():
        gtk.main_iteration(False)
    wait_for_execution_engine_sync_counter(1, logger)

    # forward
    for i in range(5):
        call_gui_callback(menubar_ctrl.on_step_into_activate, None, None)
        wait_for_execution_engine_sync_counter(1, logger)

    # # backward
    for i in range(4):
        call_gui_callback(menubar_ctrl.on_backward_step_activate, None, None)
        wait_for_execution_engine_sync_counter(1, logger)

    call_gui_callback(menubar_ctrl.on_backward_step_activate, None, None)

    sm = rafcon.statemachine.singleton.state_machine_manager.get_active_state_machine()
    while state_machine_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STOPPED:
        time.sleep(0.1)
    for key, sd in sm.root_state.scoped_data.iteritems():
        if sd.name == "beer_count":
            assert sd.value == 100

    call_gui_callback(menubar_ctrl.on_stop_activate, None)
    call_gui_callback(menubar_ctrl.on_save_as_activate, None, None, testing_utils.get_unique_temp_path())
    call_gui_callback(menubar_ctrl.on_quit_activate, None)


def test_backward_stepping_library_state(caplog):
    sys.setrecursionlimit(3000)
    testing_utils.start_rafcon()
    testing_utils.remove_all_libraries()
    # Load test library
    config_path = rafcon.__path__[0] + "/../test/common/configs_for_start_script_test/valid_config/"
    testing_utils.global_config.load('config.yaml', config_path)
    rafcon.statemachine.singleton.library_manager.initialize()

    gui_config.global_gui_config.set_config_value('HISTORY_ENABLED', False)
    gui_config.global_gui_config.set_config_value('AUTO_BACKUP_ENABLED', False)
    logger, gvm_model = create_models()
    state_machine = storage.load_state_machine_from_path(testing_utils.get_test_sm_path("unit_test_state_machines"
                                                                                       "/backward_step_library_execution_test"))
    main_window_view = MainWindowView()

    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    if testing_utils.sm_manager_model is None:
        testing_utils.sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model

    main_window_controller = MainWindowController(testing_utils.sm_manager_model, main_window_view)

    # Wait for GUI to initialize
    while gtk.events_pending():
        gtk.main_iteration(False)
    thread = threading.Thread(target=trigger_gui_signals_library_state, args=[main_window_controller, logger])
    thread.start()
    gtk.main()
    logger.debug("Gtk main loop exited!")
    thread.join()
    logger.debug("Joined test triggering thread!")
    os.chdir(testing_utils.RAFCON_PATH + "/../test/common")
    testing_utils.reload_config()
    testing_utils.test_multithreading_lock.release()
    testing_utils.assert_logger_warnings_and_errors(caplog)


@log.log_exceptions(None, gtk_quit=True)
def trigger_gui_signals_preemptive_state(*args):
    main_window_controller = args[0]
    logger = args[1]
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
    gvm = GlobalVariableManagerModel()

    # reset the synchronization counter; although the tests run in different processes they share their memory
    # as the import statements are at the top of the file and not inside the parallel called functions
    state_machine_execution_engine.synchronization_lock.acquire()
    state_machine_execution_engine.synchronization_counter = 0
    state_machine_execution_engine.synchronization_lock.release()

    call_gui_callback(menubar_ctrl.on_step_mode_activate, None, None)
    # Wait for GUI to initialize
    while gtk.events_pending():
        gtk.main_iteration(False)

    wait_for_execution_engine_sync_counter(1, logger)

    # forward
    for i in range(3):
        call_gui_callback(menubar_ctrl.on_step_into_activate, None, None)
        wait_for_execution_engine_sync_counter(2, logger)

    for i in range(2):
        call_gui_callback(menubar_ctrl.on_step_into_activate, None, None)
        wait_for_execution_engine_sync_counter(1, logger)

    # "take turn" state reached

    # backward
    for i in range(1):
        call_gui_callback(menubar_ctrl.on_backward_step_activate, None, None)
        wait_for_execution_engine_sync_counter(1, logger)

    for i in range(3):
        call_gui_callback(menubar_ctrl.on_backward_step_activate, None, None)
        wait_for_execution_engine_sync_counter(2, logger)

    call_gui_callback(menubar_ctrl.on_backward_step_activate, None, None)

    sm = rafcon.statemachine.singleton.state_machine_manager.get_active_state_machine()

    while state_machine_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STOPPED:
        time.sleep(0.1)
    beers = gvm.global_variable_manager.get_variable('beers')
    whiskey = gvm.global_variable_manager.get_variable('whiskey')
    assert beers == 0
    assert whiskey == 0

    call_gui_callback(menubar_ctrl.on_stop_activate, None)
    call_gui_callback(menubar_ctrl.on_save_as_activate, None, None, testing_utils.get_unique_temp_path())
    call_gui_callback(menubar_ctrl.on_quit_activate, None)


def test_backward_stepping_preemptive_state(caplog):
    sys.setrecursionlimit(3000)
    testing_utils.start_rafcon()
    #testing_utils.remove_all_libraries()
    gui_config.global_gui_config.set_config_value('HISTORY_ENABLED', False)
    gui_config.global_gui_config.set_config_value('AUTO_BACKUP_ENABLED', False)
    logger, gvm_model = create_models()
    state_machine = storage.load_state_machine_from_path(testing_utils.get_test_sm_path("unit_test_state_machines"
                                                                                       "/backward_step_preemtive_test"))
    main_window_view = MainWindowView()
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    if testing_utils.sm_manager_model is None:
        testing_utils.sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model

    main_window_controller = MainWindowController(testing_utils.sm_manager_model, main_window_view)

    # Wait for GUI to initialize
    while gtk.events_pending():
        gtk.main_iteration(False)
    thread = threading.Thread(target=trigger_gui_signals_preemptive_state, args=[main_window_controller, logger])
    thread.start()
    gtk.main()
    logger.debug("Gtk main loop exited!")
    thread.join()
    logger.debug("Joined test triggering thread!")
    testing_utils.reload_config()
    testing_utils.test_multithreading_lock.release()
    testing_utils.assert_logger_warnings_and_errors(caplog)


@log.log_exceptions(None, gtk_quit=True)
def trigger_gui_signals_barrier_state(*args):
    main_window_controller = args[0]
    logger = args[1]
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')

    # reset the synchronization counter; although the tests run in different processes they share their memory
    # as the import statements are at the top of the file and not inside the parallel called functions
    state_machine_execution_engine.synchronization_lock.acquire()
    state_machine_execution_engine.synchronization_counter = 0
    state_machine_execution_engine.synchronization_lock.release()

    call_gui_callback(menubar_ctrl.on_step_mode_activate, None, None)
    # Wait for GUI to initialize
    while gtk.events_pending():
        gtk.main_iteration(False)
    wait_for_execution_engine_sync_counter(1, logger)

    # forward
    for i in range(4):
        call_gui_callback(menubar_ctrl.on_step_into_activate, None, None)
        wait_for_execution_engine_sync_counter(3, logger)

    for i in range(4):
        call_gui_callback(menubar_ctrl.on_step_into_activate, None, None)
        wait_for_execution_engine_sync_counter(1, logger)

    # # backward
    for i in range(3):
        call_gui_callback(menubar_ctrl.on_backward_step_activate, None, None)
        wait_for_execution_engine_sync_counter(1, logger)

    for i in range(4):
        call_gui_callback(menubar_ctrl.on_backward_step_activate, None, None)
        wait_for_execution_engine_sync_counter(3, logger)

    call_gui_callback(menubar_ctrl.on_backward_step_activate, None, None)

    sm = rafcon.statemachine.singleton.state_machine_manager.get_active_state_machine()
    while state_machine_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STOPPED:
        time.sleep(0.1)
    for key, sd in sm.root_state.scoped_data.iteritems():
        if sd.name == "beer_number":
            assert sd.value == 100
        elif sd.name == "wine_number":
            assert sd.value == 40
        elif sd.name == "whiskey_number":
            assert sd.value == 20

    call_gui_callback(menubar_ctrl.on_stop_activate, None)
    call_gui_callback(menubar_ctrl.on_save_as_activate, None, None, testing_utils.get_unique_temp_path())
    call_gui_callback(menubar_ctrl.on_quit_activate, None)


def test_backward_stepping_barrier_state(caplog):
    sys.setrecursionlimit(3000)
    testing_utils.start_rafcon()
    testing_utils.remove_all_libraries()
    gui_config.global_gui_config.set_config_value('HISTORY_ENABLED', False)
    gui_config.global_gui_config.set_config_value('AUTO_BACKUP_ENABLED', False)
    logger, gvm_model = create_models()
    state_machine = storage.load_state_machine_from_path(testing_utils.get_test_sm_path("unit_test_state_machines"
                                                                                       "/backward_step_barrier_test"))
    main_window_view = MainWindowView()
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    if testing_utils.sm_manager_model is None:
        testing_utils.sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model

    main_window_controller = MainWindowController(testing_utils.sm_manager_model, main_window_view)

    # Wait for GUI to initialize
    while gtk.events_pending():
        gtk.main_iteration(False)
    thread = threading.Thread(target=trigger_gui_signals_barrier_state, args=[main_window_controller, logger])
    thread.start()
    gtk.main()
    logger.debug("Gtk main loop exited!")
    thread.join()
    logger.debug("Joined test triggering thread!")
    testing_utils.reload_config()
    testing_utils.test_multithreading_lock.release()
    testing_utils.assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    test_backward_stepping_barrier_state(None)
    test_backward_stepping_preemptive_state(None)
    test_backward_stepping_library_state(None)
    # pytest.main([__file__])
