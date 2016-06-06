import sys
import logging
import gtk
import threading
import time
import os
import datetime

# general tool elements
from rafcon.utils import log

# mvc elements
from rafcon.mvc.models import GlobalVariableManagerModel
from rafcon.mvc.controllers.main_window import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView

from rafcon.statemachine.storage import storage
import rafcon.mvc.singleton
import rafcon.mvc.config as gui_config
from rafcon.statemachine.singleton import state_machine_execution_engine
from rafcon.statemachine.execution.state_machine_status import StateMachineExecutionStatus

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
def trigger_gui_signals(*args):
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
        wait_for_execution_engine_sync_counter(1, logger)

    # # backward
    for i in range(3):
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


def test_backward_stepping(caplog):
    testing_utils.start_rafcon()
    testing_utils.remove_all_libraries()
    gui_config.global_gui_config.set_config_value('HISTORY_ENABLED', False)
    gui_config.global_gui_config.set_config_value('AUTO_BACKUP_ENABLED', False)
    logger, gvm_model = create_models()
    state_machine = storage.load_state_machine_from_path(testing_utils.get_test_sm_path("unit_test_state_machines"
                                                                                       "/backward_stepping_execution_state_test"))
    main_window_view = MainWindowView()
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    if testing_utils.sm_manager_model is None:
        testing_utils.sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model

    main_window_controller = MainWindowController(testing_utils.sm_manager_model, main_window_view,
                                                  editor_type="LogicDataGrouped")

    # Wait for GUI to initialize
    while gtk.events_pending():
        gtk.main_iteration(False)
    thread = threading.Thread(target=trigger_gui_signals, args=[main_window_controller, logger])
    thread.start()
    gtk.main()
    logger.debug("Gtk main loop exited!")
    thread.join()
    logger.debug("Joined test triggering thread!")
    os.chdir(testing_utils.RAFCON_PATH + "/../test/common")
    testing_utils.reload_config()
    testing_utils.test_multithrading_lock.release()
    testing_utils.assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    test_backward_stepping(None)
    # pytest.main([__file__])