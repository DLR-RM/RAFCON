import os
import time
import datetime
import pytest

# test environment elements
from tests import utils as testing_utils
from tests.utils import call_gui_callback
from tests.gui.execution import state_machines_editor_tab_status_check

# general tool elements
from rafcon.utils import log
logger = log.get_logger(__name__)


@pytest.mark.parametrize("state_machine_name,expected_warnings", [
    ("dynamic_library_insertion", 0),
    ("dynamic_library_insertion_and_deletion", 0),
    ("dynamic_state_deletion_inside_library", 23),
])
def test_dynamic_state_insertion(gui, state_machine_name, expected_warnings):

    menubar_ctrl = gui.singletons.main_window_controller.get_controller('menu_bar_controller')

    sm = gui(
        menubar_ctrl.on_open_activate, None, None,
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", state_machine_name))
    )

    gui(menubar_ctrl.on_start_activate, None, None)

    while not gui.core_singletons.state_machine_execution_engine.finished_or_stopped():
        time.sleep(0.1)
    # stop or finished are asynchronous but the call_gui_callback makes the check synchronous
    gui(state_machines_editor_tab_status_check, sm.state_machine_id, False)
    gui.expected_warnings = expected_warnings


if __name__ == '__main__':
    test_dynamic_state_insertion(None)
    test_dynamic_state_insertion_and_deletion(None)
    test_dynamic_state_insertion_and_deletion_inside_library(None)
    # pytest.main(['-s', __file__])
