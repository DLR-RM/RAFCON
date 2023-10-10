import os
import time
import pytest

# general tool elements
from rafcon.utils import log
logger = log.get_logger(__name__)


@pytest.mark.unstable
@pytest.mark.parametrize("state_machine_name,expected_warnings", [
    ("dynamic_library_insertion", 2),
    ("dynamic_library_insertion_and_deletion", 0),
    ("dynamic_state_deletion_inside_library", 23),
])
def test_dynamic_state_insertion(gui, state_machine_name, expected_warnings):
    from tests import utils as testing_utils
    from tests.gui.execution import state_machines_editor_tab_status_check

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
    # this snippet shows how to run a unit test without pytest
    from tests import utils
    from copy import deepcopy
    from tests.gui.conftest import GUITester

    parameters = {}
    with_gui = parameters.get("with_gui", True)
    config = {parameter_name: parameters.get(parameter_name) for parameter_name in
              ["core_config", "gui_config", "runtime_config", "libraries"]}

    utils.dummy_gui(None)
    if with_gui:
        utils.run_gui(**deepcopy(config))
    else:
        utils.initialize_environment(gui_already_started=False, **deepcopy(config))

    gui_tester = GUITester(with_gui, config)

    test_dynamic_state_insertion(gui_tester, "dynamic_library_insertion_and_deletion", 0)

    try:
        if with_gui:
            utils.close_gui()
    finally:
        utils.shutdown_environment(caplog=None, unpatch_threading=with_gui,
                                   expected_warnings=gui_tester.expected_warnings,
                                   expected_errors=gui_tester.expected_errors)
        gui_tester.post_test and gui_tester.post_test()
