import sys
import logging
import gtk
import threading
import time
import os
import signal

# general tool elements
from rafcon.utils import log

# mvc elements
from rafcon.mvc.models import GlobalVariableManagerModel
from rafcon.mvc.controllers.main_window import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView

from rafcon.statemachine.storage import storage
import rafcon.mvc.singleton

# test environment elements
import testing_utils
from testing_utils import call_gui_callback
import pytest


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


def wait_for_execution_engine_sync_counter(target_value, timeout=2):
    print "++++++++++ waiting for execution engine sync for ", str(target_value), " steps ++++++++++"
    execution_engine = rafcon.statemachine.singleton.state_machine_execution_engine
    import datetime
    current_time = datetime.datetime.now()
    while True:
        execution_engine.synchronization_lock.acquire()
        if execution_engine.synchronization_counter == target_value:
            execution_engine.synchronization_counter = 0
            execution_engine.synchronization_lock.release()
            break
        execution_engine.synchronization_lock.release()
        if (datetime.datetime.now() - current_time).seconds > timeout:
            raise RuntimeError("Something went wrong")
        time.sleep(0.01)


@log.log_exceptions(None, gtk_quit=True)
def trigger_gui_signals(*args):
    main_window_controller = args[0]
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')

    call_gui_callback(menubar_ctrl.on_step_mode_activate, None, None)
    wait_for_execution_engine_sync_counter(1)

    # forward
    for i in range(4):
        call_gui_callback(menubar_ctrl.on_step_into_activate, None, None)
        wait_for_execution_engine_sync_counter(3)

    for i in range(4):
        call_gui_callback(menubar_ctrl.on_step_into_activate, None, None)
        wait_for_execution_engine_sync_counter(1)

    # # backward
    for i in range(3):
        call_gui_callback(menubar_ctrl.on_backward_step_activate, None, None)
        wait_for_execution_engine_sync_counter(1)

    for i in range(4):
        call_gui_callback(menubar_ctrl.on_backward_step_activate, None, None)
        wait_for_execution_engine_sync_counter(3)

    call_gui_callback(menubar_ctrl.on_backward_step_activate, None, None)

    sm = rafcon.statemachine.singleton.state_machine_manager.get_active_state_machine()
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


def test_backward_stepping(caplog):
    testing_utils.start_rafcon()
    logger, gvm_model = create_models()
    state_machine = storage.load_statemachine_from_path(testing_utils.get_test_sm_path("unit_test_state_machines"
                                                                                       "/backward_step_barrier_test"))
    main_window_view = MainWindowView()
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    if testing_utils.sm_manager_model is None:
        testing_utils.sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model

    main_window_controller = MainWindowController(testing_utils.sm_manager_model, main_window_view,
                                                  editor_type="LogicDataGrouped")

    # Wait for GUI to initialize
    while gtk.events_pending():
        gtk.main_iteration(False)
    thread = threading.Thread(target=trigger_gui_signals, args=[main_window_controller])
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
    # test_backward_stepping(None)
    pytest.main([__file__])
