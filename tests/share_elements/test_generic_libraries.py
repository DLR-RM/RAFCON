from __future__ import print_function
from builtins import range
import os
import time

# test environment elements
import pytest
from tests import utils as testing_utils
from tests.utils import call_gui_callback

# general tool elements
from rafcon.utils import log
logger = log.get_logger(__name__)


def initialize_global_variables():
    import rafcon.gui.singleton as gui_singleton
    gui_singleton.global_variable_manager_model.global_variable_manager.set_variable("global_variable_1", "value1")
    gui_singleton.global_variable_manager_model.global_variable_manager.set_variable("global_variable_2", "value2")


def execute_all_generic_libraries_with_keyboard_only():
    from rafcon.core.singleton import state_machine_execution_engine, state_machine_manager
    import rafcon.gui.singleton as gui_singleton

    menubar_ctrl = gui_singleton.main_window_controller.get_controller('menu_bar_controller')
    call_gui_callback(
        menubar_ctrl.on_open_activate, None, None,
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "all_generic_libraries"))
    )
    testing_utils.wait_for_gui()

    call_gui_callback(menubar_ctrl.on_start_activate, None, None)
    import time
    from pykeyboard import PyKeyboard
    time.sleep(0.5)
    k = PyKeyboard()
    k.tap_key('Return', 7, 0.5)
    k.tap_key('Tab', 2, 0.5)
    k.tap_key('Return', 5, 0.5)
    call_gui_callback(menubar_ctrl.on_stop_activate, None, None)


@pytest.mark.timeout(30)
def test_all_generic_libraries_in_a_row(caplog):
    testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False},
                          libraries={'generic': os.path.join(testing_utils.LIBRARY_SM_PATH, 'generic')}
    )
    call_gui_callback(initialize_global_variables)
    try:

        execute_all_generic_libraries_with_keyboard_only()
    except Exception:
        raise
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=4)


def execute_preemption_of_all_state_machines_at_once():
    from rafcon.core.singleton import state_machine_execution_engine, state_machine_manager
    import rafcon.gui.singleton as gui_singleton

    menubar_ctrl = gui_singleton.main_window_controller.get_controller('menu_bar_controller')
    call_gui_callback(
        menubar_ctrl.on_open_activate, None, None,
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "all_dialogs_parallel_preempted"))
    )
    testing_utils.wait_for_gui()

    call_gui_callback(menubar_ctrl.on_start_activate, None, None)
    duration_waited = 0.
    period = 0.1
    while not state_machine_execution_engine.finished_or_stopped():
        time.sleep(period)
        duration_waited += period
        if duration_waited > 3.:
            call_gui_callback(menubar_ctrl.on_stop_activate, None, None)
            raise RuntimeError("The state machine should finish in less then {0}".format(duration_waited))
    print("Run duration of execute_preemption_of_all_state_machines_at_once was: {0}".format(duration_waited))


def test_preemption_of_all_state_machines_at_once(caplog):
    testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False},
                          libraries={'generic': os.path.join(testing_utils.LIBRARY_SM_PATH, 'generic')}
    )
    call_gui_callback(initialize_global_variables)
    try:
        execute_preemption_of_all_state_machines_at_once()
    except Exception:
        raise
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=4)

if __name__ == '__main__':
    # test_preemption_of_all_state_machines_at_once(None)
    test_all_generic_libraries_in_a_row(None)
    # import pytest
    # pytest.main(['-s', __file__])
