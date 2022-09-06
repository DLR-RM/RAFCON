# test environment elements
import os
from tests import utils as testing_utils
from tests.utils import call_gui_callback

# general tool elements
from rafcon.utils import log
logger = log.get_logger(__name__)


def test_state_machine_baking(gui):
    import rafcon.core.id_generator
    from rafcon.gui.helpers import state_machine as gui_helper_state_machine

    state_machine_manager = gui.core_singletons.state_machine_manager
    sm_id_0 = rafcon.core.id_generator.state_machine_id_counter

    menu_bar_controller = gui.singletons.main_window_controller.menu_bar_controller

    gui(
        menu_bar_controller.on_open_activate, None, None,
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "bake_sm"))
    )

    state_machine1 = state_machine_manager.state_machines[sm_id_0 + 1]
    print("#1 ", state_machine1)
    print("#2 ", gui(state_machine1.mutable_hash).hexdigest())

    baking_path = os.path.join(testing_utils.RAFCON_TEMP_PATH_TEST_BASE, "baking")
    # baking_path = os.path.join(testing_utils.RAFCON_TEMP_PATH_TEST_BASE_ONLY_USER_SAVE, "baking")

    gui(gui_helper_state_machine.bake_selected_state_machine, baking_path)

    gui(menu_bar_controller.on_open_activate, None, None, os.path.join(baking_path, "__generated__state_machine"))

    from rafcon.core.id_generator import generate_state_machine_id
    state_machine2 = state_machine_manager.state_machines[sm_id_0 + 4]
    print("#3 ", state_machine2)
    print("#4 ", gui(state_machine1.mutable_hash).hexdigest())

    mutable_hash_sm1 = gui(state_machine1.mutable_hash)
    mutable_hash_sm2 = gui(state_machine2.mutable_hash)
    assert mutable_hash_sm1.hexdigest() == mutable_hash_sm2.hexdigest()


if __name__ == '__main__':
    test_state_machine_baking(None)
    # import pytest
    # pytest.main(['-s', __file__])
