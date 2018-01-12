
from os.path import join

from rafcon.utils import log
# test environment elements
import testing_utils
from testing_utils import call_gui_callback

import pytest

logger = log.get_logger(__name__)


def set_selected_state_machine_id(id):
    import rafcon
    sm_manager_model = rafcon.gui.singleton.state_machine_manager_model
    sm_manager_model.selected_state_machine_id = id


def trigger_ungroup_bug_signals():
    import rafcon.core.singleton
    import rafcon.gui.singleton as gui_singleton
    from gui.widget.test_menu_bar import focus_graphical_editor_in_page, create_state_machine

    state_machine = create_state_machine()
    call_gui_callback(rafcon.core.singleton.state_machine_manager.add_state_machine, state_machine)

    sm_manager_model = rafcon.gui.singleton.state_machine_manager_model
    main_window_controller = gui_singleton.main_window_controller
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')

    current_sm_length = len(sm_manager_model.state_machines)
    first_sm_id = sm_manager_model.state_machines.keys()[0]
    call_gui_callback(menubar_ctrl.on_new_activate, None)
    main_window_controller.view['main_window'].grab_focus()
    call_gui_callback(set_selected_state_machine_id, first_sm_id)

    state_machines_ctrl = main_window_controller.get_controller('state_machines_editor_ctrl')
    page_id = state_machines_ctrl.get_page_num(first_sm_id)
    page = state_machines_ctrl.view.notebook.get_nth_page(page_id)
    focus_graphical_editor_in_page(page)
    sm_m = sm_manager_model.get_selected_state_machine_model()
    assert sm_m
    import rafcon.gui.helpers.state as gui_helper_state
    import rafcon.gui.helpers.state_machine as gui_helper_state_machine
    call_gui_callback(gui_helper_state.change_state_type, sm_m.get_state_model_by_path("ROOTSTATE/STATE3"),
                      gui_helper_state.BarrierConcurrencyState)

    call_gui_callback(sm_m.selection.set, sm_m.get_state_model_by_path("ROOTSTATE/STATE3"))
    call_gui_callback(gui_helper_state_machine.ungroup_selected_state)

    call_gui_callback(menubar_ctrl.on_save_as_activate, None, None, testing_utils.get_unique_temp_path())
    call_gui_callback(menubar_ctrl.on_stop_activate, None)


def test_ungroup_bug(caplog):
    testing_utils.run_gui(
        gui_config={'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': False},
        libraries={"ros": join(testing_utils.EXAMPLES_PATH, "libraries", "ros_libraries"),
                   "turtle_libraries": join(testing_utils.EXAMPLES_PATH, "libraries", "turtle_libraries"),
                   "generic": join(testing_utils.LIBRARY_SM_PATH, "generic")}
    )

    try:
        trigger_ungroup_bug_signals()
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog)


if __name__ == '__main__':
    test_ungroup_bug(None)
    # pytest.main(['-s', __file__])
