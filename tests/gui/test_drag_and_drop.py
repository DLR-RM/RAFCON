import pytest

import time
from rafcon.utils import log

# test environment elements
from tests import utils as testing_utils
from tests.utils import call_gui_callback

logger = log.get_logger(__name__)


def create_models(*args, **kargs):
    import rafcon.core.singleton
    import rafcon.gui.singleton
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.state_machine import StateMachine

    state1 = HierarchyState('State1', state_id="State1")

    ctr_state = HierarchyState(name="Root", state_id="Root")
    ctr_state.add_state(state1)
    ctr_state.name = "Container"

    sm = StateMachine(ctr_state)

    # add new state machine
    rafcon.core.singleton.state_machine_manager.add_state_machine(sm)
    testing_utils.wait_for_gui()
    # select state machine
    rafcon.gui.singleton.state_machine_manager_model.selected_state_machine_id = sm.state_machine_id


@pytest.mark.unstable
@pytest.mark.parametrize('gui', [{"runtime_config": {
    'MAIN_WINDOW_MAXIMIZED': False,
    'MAIN_WINDOW_SIZE': (1500, 800),
    'MAIN_WINDOW_POS': (0, 0),
    'LEFT_BAR_WINDOW_UNDOCKED': False,
    'RIGHT_BAR_WINDOW_UNDOCKED': False,
    'CONSOLE_WINDOW_UNDOCKED': False,
    'LEFT_BAR_HIDDEN': True,
    'RIGHT_BAR_HIDDEN': True,
    'CONSOLE_HIDDEN': True,
}}], indirect=True, ids=["with fixed runtime config values"])
def test_drag_and_drop_test(gui):
    import rafcon.core.singleton

    gui(create_models)

    # TODO test should use real SelectionData objects and motion to provide selection
    # -> currently very limited test scenario
    # TODO test needs check on position -> is the state drawn where it was dropped?

    sm_manager_model = gui.singletons.state_machine_manager_model
    main_window_controller = gui.singletons.main_window_controller

    states_machines_editor_controller = main_window_controller.state_machines_editor_ctrl
    library_tree_controller = main_window_controller.library_controller
    state_icon_controller = main_window_controller.state_icon_controller
    graphical_editor_controller = states_machines_editor_controller.get_child_controllers()[0]

    # the view is required here; as it is created asynchronously we explicitly wait for its creation
    while not graphical_editor_controller.view:
        testing_utils.wait_for_gui()

    # wait for root state to be focused
    time.sleep(.5)

    gui(library_tree_controller.view.expand_all)
    # generic and unit_test_state_machines in library tree index 1 is unit_test_state_machines
    gui(library_tree_controller.view.get_selection().select_path, (1, 0))

    state_machine_m = sm_manager_model.get_selected_state_machine_model()

    # GTK4 drag & drop: the drag source's "prepare" handler registers a payload provider,
    # the editor's DropTarget "drop" handler (on_drag_data_received) takes it and inserts
    # the state into the container state selected during the drag motion

    # insert state in root_state
    print("insert state in root_state")
    gui(graphical_editor_controller.on_drag_motion, None, 200, 200)
    # Override selection
    state_m = state_machine_m.root_state
    gui(state_machine_m.selection.set, [state_m])
    assert gui(library_tree_controller._on_drag_prepare, None, 0, 0) is not None
    gui(graphical_editor_controller.on_drag_data_received, None, "rafcon-library-state", 200, 200)
    assert len(sm_manager_model.get_selected_state_machine_model().root_state.state.states) == 2

    # insert state from IconView
    print("insert state from IconView")
    gui(graphical_editor_controller.on_drag_motion, None, 300, 300)
    # Override selection
    state_m = state_machine_m.root_state
    gui(state_machine_m.selection.set, [state_m])
    assert gui(state_icon_controller._on_drag_prepare, None, 30, 15) is not None
    gui(graphical_editor_controller.on_drag_data_received, None, "rafcon-new-state", 300, 300)
    assert len(sm_manager_model.get_selected_state_machine_model().root_state.state.states) == 3

    # try to insert state next to root state -> the drop is rejected
    print("insert state next to root state")
    # Position (0, 0) is left above the root state, so no container state gets selected
    gui(state_machine_m.selection.clear)
    gui(graphical_editor_controller.on_drag_motion, None, 0, 0)
    assert gui(state_icon_controller._on_drag_prepare, None, 30, 15) is not None
    assert gui(graphical_editor_controller.on_drag_data_received, None, "rafcon-new-state", 0, 0) is False
    assert len(sm_manager_model.get_selected_state_machine_model().root_state.state.states) == 3

    # insert state in state1
    print("insert state in state1")
    state_m = state_machine_m.root_state.states['State1']
    gui(state_machine_m.selection.set, [state_m])
    assert gui(library_tree_controller._on_drag_prepare, None, 0, 0) is not None
    gui(graphical_editor_controller.on_drag_data_received, None, "rafcon-library-state", 20, 20)
    assert len(sm_manager_model.get_selected_state_machine_model().root_state.state.states['State1'].states) == 1


if __name__ == '__main__':
    test_drag_and_drop_test(None)
    # pytest.main([__file__, '-xs'])
