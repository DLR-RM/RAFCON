
# test environment elements
import os
import testing_utils
from testing_utils import call_gui_callback

# general tool elements
from rafcon.utils import log
logger = log.get_logger(__name__)


def trigger_baking_commands():
    # core elements
    import rafcon.core.id_generator
    from rafcon.core.singleton import state_machine_manager
    # gui elements
    import rafcon.gui.singleton as gui_singleton
    from rafcon.gui.helpers import state_machine as gui_helper_state_machine

    sm_id_0 = rafcon.core.id_generator.state_machine_id_counter

    menu_bar_controller = gui_singleton.main_window_controller.get_controller("menu_bar_controller")

    call_gui_callback(
        menu_bar_controller.on_open_activate, None, None,
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "bake_sm"))
    )

    state_machine1 = state_machine_manager.state_machines[sm_id_0 + 1]
    print "#1 ", state_machine1
    print "#2 ", call_gui_callback(state_machine1.mutable_hash).hexdigest()

    baking_path = os.path.join(testing_utils.RAFCON_TEMP_PATH_TEST_BASE, "baking")
    # baking_path = os.path.join(testing_utils.RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE, "baking")

    call_gui_callback(gui_helper_state_machine.bake_selected_state_machine, baking_path)

    call_gui_callback(
        menu_bar_controller.on_open_activate, None, None, os.path.join(baking_path, "__generated__state_machine")
    )

    from rafcon.core.id_generator import generate_state_machine_id
    state_machine2 = state_machine_manager.state_machines[sm_id_0 + 4]
    print "#3 ", state_machine2
    print "#4 ", call_gui_callback(state_machine1.mutable_hash).hexdigest()

    mutable_hash_sm1 = call_gui_callback(state_machine1.mutable_hash)
    mutable_hash_sm2 = call_gui_callback(state_machine2.mutable_hash)
    assert mutable_hash_sm1.hexdigest() == mutable_hash_sm2.hexdigest()


def test_state_machine_baking(caplog):
    testing_utils.dummy_gui(caplog)

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
