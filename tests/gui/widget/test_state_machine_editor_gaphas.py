# general tool elements
from rafcon.utils import log

# test environment elements
import testing_utils
from testing_utils import call_gui_callback

logger = log.get_logger(__name__)


@log.log_exceptions(None, gtk_quit=True)
def trigger_copy_delete_bug_signals():
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
    from gui.widget.test_menu_bar import select_and_paste_state
    import rafcon.gui.singleton
    import rafcon.gui.config as gui_config
    import rafcon.gui.controllers.graphical_editor_gaphas as graphical_editor_gaphas

    sm_manager_model = rafcon.gui.singleton.state_machine_manager_model
    menubar_ctrl = rafcon.gui.singleton.main_window_controller.get_controller('menu_bar_controller')
    state_machines_ctrl = rafcon.gui.singleton.main_window_controller.get_controller('state_machines_editor_ctrl')

    current_sm_length = len(sm_manager_model.state_machines)
    call_gui_callback(menubar_ctrl.on_new_activate, None)
    first_sm_id = sm_manager_model.state_machines.keys()[0]

    # 2. action
    call_gui_callback(menubar_ctrl.on_add_state_activate, None)
    assert len(sm_manager_model.state_machines) == current_sm_length + 1
    sm_m = sm_manager_model.state_machines[first_sm_id]
    root_state_m = sm_m.root_state
    logger.info("States of root state: {}".format(root_state_m.states.values()))
    new_state_m = root_state_m.states.values()[0]  # becomes the hstate

    call_gui_callback(sm_m.selection.set, new_state_m)
    import rafcon.gui.helpers.state as gui_helpers_state
    call_gui_callback(gui_helpers_state.change_state_type, new_state_m, HierarchyState)
    logger.info("States of root state after type change: {}".format([str(state_m.state) for state_m in root_state_m.states.values()]))
    new_hstate_m = root_state_m.states.values()[0]

    # 3. action
    call_gui_callback(sm_m.selection.set, new_hstate_m)
    call_gui_callback(menubar_ctrl.on_add_state_activate, None)
    logger.info("States of hierarchy state: {}".format(new_hstate_m.states.values()))
    new_state_m = new_hstate_m.states.values()[0]

    # 4. action copy-paste
    page_id = state_machines_ctrl.get_page_num(first_sm_id)
    page = state_machines_ctrl.view.notebook.get_nth_page(page_id)
    select_and_paste_state(sm_m, new_state_m, root_state_m, menubar_ctrl, 'copy',
                           rafcon.gui.singleton.main_window_controller, page)

    graphical_editor_ctrl = state_machines_ctrl.get_controller(first_sm_id)

    if gui_config.global_gui_config.get_config_value('GAPHAS_EDITOR'):
        assert isinstance(graphical_editor_ctrl, graphical_editor_gaphas.GraphicalEditorController)
        assert graphical_editor_ctrl.canvas.get_view_for_model(new_state_m)

    new_estate_m = root_state_m.states.values()[0]
    if new_estate_m.state.get_path() == new_hstate_m.state.get_path():
        new_estate_m = root_state_m.states.values()[1]

    call_gui_callback(sm_m.selection.set, new_estate_m)
    call_gui_callback(menubar_ctrl.on_delete_activate, None)
    if gui_config.global_gui_config.get_config_value('GAPHAS_EDITOR'):
        assert graphical_editor_ctrl.canvas.get_view_for_model(new_state_m)

    print "TEST FINISHED"


def test_copy_delete_bug(caplog):
    from os.path import join

    libraries = {"ros": join(testing_utils.EXAMPLES_PATH, "libraries", "ros_libraries"),
                 "turtle_libraries": join(testing_utils.EXAMPLES_PATH, "libraries", "turtle_libraries"),
                 "generic": join(testing_utils.LIBRARY_SM_PATH, "generic")}
    change_in_gui_config = {'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': False, 'GAPHAS_EDITOR': True}
    testing_utils.run_gui(gui_config=change_in_gui_config, libraries=libraries,)

    try:
        trigger_copy_delete_bug_signals()
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog)


if __name__ == '__main__':
    # testing_utils.dummy_gui(None)
    # test_copy_delete_bug(None)
    import pytest
    pytest.main(['-s', __file__])
