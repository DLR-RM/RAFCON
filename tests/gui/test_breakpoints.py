import os

import rafcon as _rafcon_pkg

from tests import utils as testing_utils
from rafcon.utils import log

logger = log.get_logger(__name__)

_TUTORIALS = os.path.join(os.path.dirname(_rafcon_pkg.__file__), "share", "rafcon", "examples", "tutorials")
BOTTLES_PATH = os.path.join(_TUTORIALS, "99_bottles_of_beer")
COUNT_BOTTLES = "SFZGMH"
DECIMATE_BOTTLES = "NDIVLD"


def test_breakpoints_panel(gui):
    """Tests the breakpoints panel GUI — setting, toggling, and removing breakpoints."""
    import rafcon.gui.singleton as gui_singleton
    from rafcon.core.storage import storage
    from rafcon.core.singleton import state_machine_execution_engine, state_machine_manager

    # load from disk so states have a file_system_path (breakpoints need it)
    sm = gui(storage.load_state_machine_from_path, BOTTLES_PATH)
    sm_id = gui(state_machine_manager.add_state_machine, sm)
    testing_utils.wait_for_gui()

    sm_m = gui_singleton.state_machine_manager_model.state_machines[sm_id]
    count_state_m = sm_m.root_state.states[COUNT_BOTTLES]
    decimate_state_m = sm_m.root_state.states[DECIMATE_BOTTLES]
    count_state = count_state_m.state
    decimate_state = decimate_state_m.state

    bm = state_machine_execution_engine.breakpoint_manager
    breakpoints_ctrl = gui_singleton.main_window_controller.get_controller('breakpoints_ctrl')
    states_editor_ctrl = gui_singleton.main_window_controller.get_controller('states_editor_ctrl')

    # select the state so the editor opens a tab for it, then check the breakpoint checkbox
    # this is the same path the right-click "Set Breakpoint" menu item takes
    gui(sm_m.selection.set, [count_state_m])
    testing_utils.wait_for_gui()
    count_tab_key = states_editor_ctrl.get_state_identifier(count_state_m)
    props_ctrl = states_editor_ctrl.tabs[count_tab_key]['controller'].get_controller('properties_ctrl')
    gui(props_ctrl.view['breakpoint_checkbox'].set_active, True)

    gui(sm_m.selection.set, [decimate_state_m])
    testing_utils.wait_for_gui()
    decimate_tab_key = states_editor_ctrl.get_state_identifier(decimate_state_m)
    props_ctrl = states_editor_ctrl.tabs[decimate_tab_key]['controller'].get_controller('properties_ctrl')
    gui(props_ctrl.view['breakpoint_checkbox'].set_active, True)

    assert bm.should_pause(count_state)
    assert bm.should_pause(decimate_state)
    assert len(list(breakpoints_ctrl.breakpoints_store)) == 2

    # disable all, then re-enable all
    toggle_btn = breakpoints_ctrl.view['toggle_all_button']
    gui(toggle_btn.set_active, True)
    assert not bm.should_pause(count_state)
    assert not bm.should_pause(decimate_state)

    gui(toggle_btn.set_active, False)
    assert bm.should_pause(count_state)
    assert bm.should_pause(decimate_state)

    # test individual remove from breakpoints panel
    # select the first breakpoint in the tree and remove it
    gui(breakpoints_ctrl.breakpoints_tree.get_selection().select_path, 0)
    gui(breakpoints_ctrl.on_remove_selected, None)
    testing_utils.wait_for_gui()

    # verify one breakpoint was removed and one remains
    assert len(list(breakpoints_ctrl.breakpoints_store)) == 1
    remaining_breakpoints = bm.get_all_breakpoints()
    assert len(remaining_breakpoints) == 1

    # remove all remaining breakpoints
    gui(breakpoints_ctrl.on_remove_all, None)
    assert not bm.get_all_breakpoints()
    assert len(list(breakpoints_ctrl.breakpoints_store)) == 0
