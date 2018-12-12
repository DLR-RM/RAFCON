from __future__ import print_function
from builtins import range
import os
import time

# test environment elements
import testing_utils
from testing_utils import call_gui_callback, wait_for_execution_engine_sync_counter

# general tool elements
from rafcon.utils import log
logger = log.get_logger(__name__)


def initialize_global_variables():
    import rafcon.gui.singleton as gui_singleton
    gui_singleton.global_variable_manager_model.global_variable_manager.set_variable("global_variable_1", "value1")
    gui_singleton.global_variable_manager_model.global_variable_manager.set_variable("global_variable_2", "value2")


def execute_preemption_of_all_state_machines_at_once():
    from rafcon.core.singleton import state_machine_execution_engine, state_machine_manager
    import rafcon.gui.singleton as gui_singleton

    menubar_ctrl = gui_singleton.main_window_controller.get_controller('menu_bar_controller')
    call_gui_callback(
        menubar_ctrl.on_open_activate, None, None,
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "all_dialogs_parallel_preempted"))
    )
    testing_utils.wait_for_gui()
    # reset the synchronization counter; although the tests run in different processes they share their memory
    # as the import statements are at the top of the file and not inside the parallel called functions
    state_machine_execution_engine.synchronization_lock.acquire()
    state_machine_execution_engine.synchronization_counter = 0
    state_machine_execution_engine.synchronization_lock.release()

    call_gui_callback(menubar_ctrl.on_start_activate, None, None)
    wait_for_execution_engine_sync_counter(1, logger)


def test_preemption_of_all_state_machines_at_once(caplog):
    testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
    call_gui_callback(initialize_global_variables)
    try:
        execute_preemption_of_all_state_machines_at_once()
    except Exception:
        raise
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=4)


if __name__ == '__main__':
    import pytest
    test_preemption_of_all_state_machines_at_once(None)
    # pytest.main(['-s', __file__])
