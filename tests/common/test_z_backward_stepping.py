import sys
import logging
import gtk
import time
import os
import datetime

# gui elements
import rafcon.core.singleton
import rafcon.gui.singleton
from rafcon.gui.models.global_variable_manager import GlobalVariableManagerModel

# state machine elements
from rafcon.core.singleton import state_machine_execution_engine
from rafcon.core.storage import storage

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

    sm = rafcon.core.singleton.state_machine_manager.get_active_state_machine()
    while not state_machine_execution_engine.finished_or_stopped():
        time.sleep(0.1)
    for key, sd in sm.root_state.scoped_data.iteritems():
        if sd.name == "beer_count":
            assert sd.value == 100

    call_gui_callback(menubar_ctrl.on_stop_activate, None)


def test_backward_stepping_library_state(caplog):

    # config_path = os.path.join(testing_utils.TESTS_PATH, 'common', 'configs_for_start_script_test' ,'valid_config')
    # testing_utils.run_gui(core_config=('config.yaml', config_path),
    #                       gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
    testing_utils.run_gui(core_config={'PROFILER_RESULT_PATH': "/tmp/rafcon_profiler_result.prf",
                                       'PROFILER_RUN': False,
                                       'PROFILER_VIEWER': True
                                       },
                          gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False},
                          libraries={'generic': "${RAFCON_LIB_PATH}/generic",
                                     'unit_test': os.path.join(testing_utils.TESTS_PATH, 'assets',
                                                               'unit_test_state_machines',
                                                               'backward_step_library_execution_test', 'test_library')
                                     }
                          )
    logger, gvm_model = create_models()
    state_machine = storage.load_state_machine_from_path(testing_utils.get_test_sm_path("unit_test_state_machines"
                                                                                       "/backward_step_library_execution_test"))
    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
    try:
        trigger_gui_signals_library_state(rafcon.gui.singleton.main_window_controller, logger)
    finally:
        menubar_ctrl = rafcon.gui.singleton.main_window_controller.get_controller('menu_bar_controller')
        call_gui_callback(menubar_ctrl.on_quit_activate, None, None, True)

        testing_utils.wait_for_gui_quit()
        logger.debug("after gtk main")

        testing_utils.shutdown_environment(caplog=caplog)


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

    sm = rafcon.core.singleton.state_machine_manager.get_active_state_machine()

    while not state_machine_execution_engine.finished_or_stopped():
        time.sleep(0.1)
    beers = gvm.global_variable_manager.get_variable('beers')
    whiskey = gvm.global_variable_manager.get_variable('whiskey')
    assert beers == 0
    assert whiskey == 0

    call_gui_callback(menubar_ctrl.on_stop_activate, None)


def test_backward_stepping_preemptive_state(caplog):

    testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
    logger, gvm_model = create_models()
    state_machine = storage.load_state_machine_from_path(testing_utils.get_test_sm_path("unit_test_state_machines"
                                                                                       "/backward_step_preemtive_test"))
    rafcon.gui.singleton.state_machine_manager.add_state_machine(state_machine)
    try:
        trigger_gui_signals_preemptive_state(rafcon.gui.singleton.main_window_controller, logger)
    finally:
        menubar_ctrl = rafcon.gui.singleton.main_window_controller.get_controller('menu_bar_controller')
        call_gui_callback(menubar_ctrl.on_quit_activate, None, None, True)

        testing_utils.wait_for_gui_quit()
        logger.debug("after gtk main")

        testing_utils.shutdown_environment(caplog=caplog)


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

    sm = rafcon.gui.singleton.state_machine_manager.get_active_state_machine()
    while not state_machine_execution_engine.finished_or_stopped():
        time.sleep(0.1)
    for key, sd in sm.root_state.scoped_data.iteritems():
        if sd.name == "beer_number":
            assert sd.value == 100
        elif sd.name == "wine_number":
            assert sd.value == 40
        elif sd.name == "whiskey_number":
            assert sd.value == 20

    call_gui_callback(menubar_ctrl.on_stop_activate, None)


def test_backward_stepping_barrier_state(caplog):

    testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
    logger, gvm_model = create_models()
    state_machine = storage.load_state_machine_from_path(testing_utils.get_test_sm_path("unit_test_state_machines"
                                                                                       "/backward_step_barrier_test"))
    rafcon.gui.singleton.state_machine_manager.add_state_machine(state_machine)
    try:
        trigger_gui_signals_barrier_state(rafcon.gui.singleton.main_window_controller, logger)
    finally:
        menubar_ctrl = rafcon.gui.singleton.main_window_controller.get_controller('menu_bar_controller')
        call_gui_callback(menubar_ctrl.on_quit_activate, None, None, True)

        testing_utils.wait_for_gui_quit()
        logger.debug("after gtk main")

        testing_utils.shutdown_environment(caplog=caplog)


if __name__ == '__main__':
    test_backward_stepping_barrier_state(None)
    test_backward_stepping_preemptive_state(None)
    test_backward_stepping_library_state(None)
    # pytest.main([__file__])
