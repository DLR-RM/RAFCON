import os
import time

from tests import utils as testing_utils
from tests.utils import wait_for_gui

sm_path = os.path.join(testing_utils.TEST_ASSETS_PATH, "unit_test_state_machines", "99_bottles_of_beer_monitoring")


def open_test_state_machine(gui):
    import rafcon.gui.singleton

    smm_m = rafcon.gui.singleton.state_machine_manager_model
    main_window_controller = rafcon.gui.singleton.main_window_controller
    menubar_ctrl = main_window_controller.menu_bar_controller
    state_machines_ctrl = main_window_controller.state_machines_editor_ctrl

    gui(menubar_ctrl.on_open_activate, None, None, sm_path)
    time.sleep(0.5)
    gui(wait_for_gui)  # Wait for gaphas view

    sm_m = smm_m.state_machines[smm_m.selected_state_machine_id]
    sm_id = sm_m.state_machine.state_machine_id
    sm_gaphas_ctrl = state_machines_ctrl.get_controller(sm_id)
    canvas = sm_gaphas_ctrl.canvas
    gaphas_view = sm_gaphas_ctrl.view.editor

    return sm_m, canvas, gaphas_view


def test_auto_layout(gui):
    from rafcon.gui.layouter import StateMachineLayouter
    sm_m, canvas, view = open_test_state_machine(gui)
    initial_size = sm_m.root_state.meta['gui']['editor_gaphas']['size']
    print(sm_m.root_state.meta['gui']['editor_gaphas']['size'])
    StateMachineLayouter(sm_m.root_state).layout_state_machine()
    new_size = sm_m.root_state.meta['gui']['editor_gaphas']['size']
    print(sm_m.root_state.meta['gui']['editor_gaphas']['size'])
    assert initial_size != new_size
    assert (int(new_size[0]), int(new_size[1])) == (356, 151)
