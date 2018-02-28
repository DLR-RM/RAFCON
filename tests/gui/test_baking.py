
# test environment elements
import os
import testing_utils
from testing_utils import call_gui_callback

# general tool elements
from rafcon.utils import log
logger = log.get_logger(__name__)


def trigger_baking_commands():
    # core elements
    from rafcon.core.singleton import state_machine_manager
    # gui elements
    import rafcon.gui.singleton as gui_singleton
    from rafcon.gui.helpers import state_machine as gui_helper_state_machine

    menu_bar_controller = gui_singleton.main_window_controller.get_controller("menu_bar_controller")

    call_gui_callback(
        menu_bar_controller.on_open_activate, None, None,
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "bake_sm"))
    )

    state_machine1 = state_machine_manager.state_machines[1]
    # print state_machine1
    # print state_machine1.mutable_hash().hexdigest()

    baking_path = os.path.join(testing_utils.RAFCON_TEMP_PATH_TEST_BASE, "baking")
    # baking_path = os.path.join(testing_utils.RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE, "baking")

    call_gui_callback(gui_helper_state_machine.bake_selected_state_machine, baking_path)

    call_gui_callback(
        menu_bar_controller.on_open_activate, None, None, os.path.join(baking_path, "__generated__state_machine")
    )

    state_machine2 = state_machine_manager.state_machines[4]
    # print state_machine2
    # print state_machine1.mutable_hash().hexdigest()

    assert state_machine1.mutable_hash().hexdigest() == state_machine2.mutable_hash().hexdigest()


def test_state_machine_baking(caplog):
    testing_utils.run_gui(
        gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False},
        libraries={'unit_test_state_machines': testing_utils.get_test_sm_path("unit_test_state_machines")}
    )
    try:
        trigger_baking_commands()
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog)


if __name__ == '__main__':
    test_state_machine_baking(None)
    # import pytest
    # pytest.main(['-s', __file__])
