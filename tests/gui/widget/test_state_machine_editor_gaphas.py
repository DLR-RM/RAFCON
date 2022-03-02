# general tool elements
from os.path import join

import pytest

from rafcon.utils import log

# test environment elements
from tests import utils as testing_utils
from tests.utils import call_gui_callback

logger = log.get_logger(__name__)


@pytest.mark.parametrize('gui', [{"libraries": {
    "ros": join(testing_utils.EXAMPLES_PATH, "libraries", "ros_libraries"),
    "turtle_libraries": join(testing_utils.EXAMPLES_PATH, "libraries", "turtle_libraries")
}}], indirect=True, ids=["with ros and turtle libraries"])
def test_copy_delete_bug(gui):
    """The function triggers multiple actions that result into a gaphas bug.

    The test should be secured that the bug 'Gaphas removing wrong view if delete state #155'
    do not occur again.

    Procedure for the bug:
    - root state (by new state machine 1. action)
      - child hierarchy state (by add state and by type change 2. action)
        - child child execution state (by add state 3. action)
      - child execution state (by copy paste of child child execution state 4. action)

    -> 5. action is to delete child execution state whereby it is proved that not the view of child child execution
    state is removed.
    """
    from rafcon.core.states.hierarchy_state import HierarchyState
    from tests.gui.widget.test_menu_bar import select_and_paste_state
    import rafcon.gui.config as gui_config
    import rafcon.gui.controllers.graphical_editor_gaphas as graphical_editor_gaphas

    sm_manager_model = gui.singletons.state_machine_manager_model
    menubar_ctrl = gui.singletons.main_window_controller.menu_bar_controller
    state_machines_ctrl = gui.singletons.main_window_controller.state_machines_editor_ctrl

    current_sm_length = len(sm_manager_model.state_machines)
    gui(menubar_ctrl.on_new_activate, None)
    first_sm_id = list(sm_manager_model.state_machines.keys())[0]

    # 2. action
    gui(menubar_ctrl.on_add_state_activate, None)
    assert len(sm_manager_model.state_machines) == current_sm_length + 1
    sm_m = sm_manager_model.state_machines[first_sm_id]
    root_state_m = sm_m.root_state
    logger.info("States of root state: {}".format(list(root_state_m.states.values())))
    new_state_m = list(root_state_m.states.values())[0]  # becomes the hstate

    gui(sm_m.selection.set, new_state_m)
    import rafcon.gui.helpers.state as gui_helpers_state
    gui(gui_helpers_state.change_state_type, new_state_m, HierarchyState)
    logger.info("States of root state after type change: {}".format([str(state_m.state) for state_m in list(root_state_m.states.values())]))
    new_hstate_m = list(root_state_m.states.values())[0]

    # 3. action
    gui(sm_m.selection.set, new_hstate_m)
    gui(menubar_ctrl.on_add_state_activate, None)
    logger.info("States of hierarchy state: {}".format(list(new_hstate_m.states.values())))
    new_state_m = list(new_hstate_m.states.values())[0]

    # 4. action copy-paste
    page_id = state_machines_ctrl.get_page_num(first_sm_id)
    page = state_machines_ctrl.view.notebook.get_nth_page(page_id)
    select_and_paste_state(gui, sm_m, new_state_m, root_state_m, menubar_ctrl, 'copy',
                           gui.singletons.main_window_controller, page)

    graphical_editor_ctrl = state_machines_ctrl.get_controller(first_sm_id)

    assert isinstance(graphical_editor_ctrl, graphical_editor_gaphas.GraphicalEditorController)
    assert graphical_editor_ctrl.canvas.get_view_for_model(new_state_m)

    new_estate_m = list(root_state_m.states.values())[0]
    if new_estate_m.state.get_path() == new_hstate_m.state.get_path():
        new_estate_m = list(root_state_m.states.values())[1]

    gui(sm_m.selection.set, new_estate_m)
    gui(menubar_ctrl.on_delete_activate, None)
    assert graphical_editor_ctrl.canvas.get_view_for_model(new_state_m)


if __name__ == '__main__':
    # testing_utils.dummy_gui(None)
    # test_copy_delete_bug(None)
    import pytest
    pytest.main(['-s', __file__])
