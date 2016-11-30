import gtk
import threading
import pytest
import time
import os
import sys

# gui elements
import rafcon.gui.config as gui_config
import rafcon.gui.singleton
from rafcon.gui.controllers.main_window import MainWindowController
from rafcon.gui.views.main_window import MainWindowView

# core elements
import rafcon.core.config
import rafcon.core.singleton
from rafcon.core.singleton import state_machine_execution_engine

# general tool elements
from rafcon.utils import log
import testing_utils
from testing_utils import call_gui_callback

logger = log.get_logger(__name__)


def trigger_gvm_signals(main_window_controller):
    gvm = rafcon.core.singleton.global_variable_manager
    gvm_controller = main_window_controller.get_controller('global_variable_manager_ctrl')
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')

    # TODO: replace these at some point with released state machines
    call_gui_callback(menubar_ctrl.on_open_activate, None, None, os.path.join(
        testing_utils.RAFCON_PATH, "../test_scripts/unit_test_state_machines/0.7.13.backward_compatibility_storage_test"))

    state_machine_execution_engine.start()

    while not state_machine_execution_engine.finished_or_stopped():
        time.sleep(0.01)

    assert gvm.get_variable("b1") == 1
    assert gvm.get_variable("b2") == 1
    assert gvm.get_variable("h1") == 1
    assert gvm.get_variable("e1") == 1
    assert gvm.get_variable("l1") == 1

    call_gui_callback(menubar_ctrl.on_quit_activate, None)


def test_backward_compatibility_storage(caplog):
    sys.setrecursionlimit(3000)
    testing_utils.start_rafcon()

    gui_config.global_gui_config.set_config_value('HISTORY_ENABLED', False)
    gui_config.global_gui_config.set_config_value('AUTO_BACKUP_ENABLED', False)

    testing_utils.remove_all_libraries()
    library_paths = rafcon.core.config.global_config.get_config_value("LIBRARY_PATHS")
    library_paths["unit_test_state_machines"] = testing_utils.RAFCON_PATH + "/../test_scripts/unit_test_state_machines"
    rafcon.core.singleton.library_manager.refresh_libraries()

    testing_utils.remove_all_gvm_variables()

    testing_utils.sm_manager_model = rafcon.gui.singleton.state_machine_manager_model

    main_window_view = MainWindowView()
    main_window_controller = MainWindowController(testing_utils.sm_manager_model, main_window_view)

    # Wait for GUI to initialize
    testing_utils.wait_for_gui()

    thread = threading.Thread(target=trigger_gvm_signals, args=[main_window_controller])
    thread.start()
    gtk.main()
    logger.debug("after gtk main")

    thread.join()

    testing_utils.remove_all_libraries()

    testing_utils.test_multithreading_lock.release()

    # expected_errors=1 because global_variable_is_editable throws an error
    testing_utils.assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    test_backward_compatibility_storage(None)
    # pytest.main(['-s', __file__])