import os
import time
import datetime
import pytest

# test environment elements
import testing_utils
from testing_utils import call_gui_callback

# general tool elements
from rafcon.utils import log
logger = log.get_logger(__name__)


def execute_dynamic_state_insertion():
    from rafcon.core.singleton import state_machine_execution_engine, state_machine_manager
    import rafcon.gui.singleton as gui_singleton

    menubar_ctrl = gui_singleton.main_window_controller.get_controller('menu_bar_controller')

    call_gui_callback(
        menubar_ctrl.on_open_activate, None, None,
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "dynamic_library_insertion"))
    )
    testing_utils.wait_for_gui()

    call_gui_callback(menubar_ctrl.on_start_activate, None, None)

    testing_utils.wait_for_gui()

    sm = state_machine_manager.get_active_state_machine()
    while not state_machine_execution_engine.finished_or_stopped():
        time.sleep(0.1)

    call_gui_callback(menubar_ctrl.on_stop_activate, None)


def test_dynamic_state_insertion(caplog):
    testing_utils.run_gui(
        gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False},
        libraries={'unit_test_state_machines': testing_utils.get_test_sm_path("unit_test_state_machines")}
    )
    try:
        execute_dynamic_state_insertion()
    except Exception:
        raise
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog)


if __name__ == '__main__':
    test_dynamic_state_insertion(None)
    # pytest.main(['-s', __file__])
