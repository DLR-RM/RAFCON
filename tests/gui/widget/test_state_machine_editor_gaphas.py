import threading
from os.path import join

import gtk
# gui elements
import rafcon.gui.config as gui_config
import rafcon.gui.singleton
import rafcon.gui
from rafcon.gui.controllers.main_window import MainWindowController
from rafcon.gui.views.main_window import MainWindowView
import rafcon.gui.controllers.graphical_editor_gaphas as graphical_editor_gaphas

# core elements
import rafcon.core.config
from rafcon.core.states.hierarchy_state import HierarchyState
import rafcon.core.singleton

# general tool elements
from rafcon.utils import log

# test environment elements
import testing_utils
from gui.widget.test_menu_bar import select_and_paste_state
from testing_utils import call_gui_callback

logger = log.get_logger(__name__)


@log.log_exceptions(None, gtk_quit=True)
def trigger_copy_delete_bug_signals(*args):
    """The function triggers multiple actions that result into a gaphas bug.

    The test should be secured that the bug 'Gaphas removing wrong view if delete state #155'
    do not occur again.
    """

    sm_manager_model = args[0]
    main_window_controller = args[1]
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')

    current_sm_length = len(sm_manager_model.state_machines)
    call_gui_callback(menubar_ctrl.on_new_activate, None)
    first_sm_id = sm_manager_model.state_machines.keys()[0]

    call_gui_callback(menubar_ctrl.on_add_state_activate, None)
    assert len(sm_manager_model.state_machines) == current_sm_length + 1
    sm_m = sm_manager_model.state_machines[first_sm_id]
    root_state_m = sm_m.root_state
    logger.info("States of root state: {}".format(root_state_m.states.values()))
    new_state_m = root_state_m.states.values()[0]
    call_gui_callback(sm_m.selection.set, new_state_m)
    import rafcon.gui.helpers.state as gui_helpers_state
    call_gui_callback(gui_helpers_state.change_state_type, new_state_m, HierarchyState)
    logger.info("States of root state after type change: {}".format([str(state_m.state) for state_m in root_state_m.states.values()]))
    new_hstate_m = root_state_m.states.values()[0]
    call_gui_callback(sm_m.selection.set, new_hstate_m)
    call_gui_callback(menubar_ctrl.on_add_state_activate, None)
    logger.info("States of hierarchy state: {}".format(new_hstate_m.states.values()))
    new_state_m = new_hstate_m.states.values()[0]

    state_machines_ctrl = main_window_controller.get_controller('state_machines_editor_ctrl')
    page_id = state_machines_ctrl.get_page_num(first_sm_id)
    page = state_machines_ctrl.view.notebook.get_nth_page(page_id)
    select_and_paste_state(sm_m, new_state_m, root_state_m, menubar_ctrl, 'copy', main_window_controller, page)

    graphical_editor_ctrl = state_machines_ctrl.get_controller(first_sm_id)

    if gui_config.global_gui_config.get_config_value('GAPHAS_EDITOR'):
        assert isinstance(graphical_editor_ctrl, graphical_editor_gaphas.GraphicalEditorController)
        assert graphical_editor_ctrl.canvas.get_view_for_model(new_state_m)

    new_estate_m = root_state_m.states.values()[0]
    if new_estate_m.state.get_path() == new_hstate_m.state.get_path():
        new_estate_m = root_state_m.states.values()[0]

    call_gui_callback(sm_m.selection.set, new_estate_m)
    call_gui_callback(menubar_ctrl.on_delete_activate, None)
    if gui_config.global_gui_config.get_config_value('GAPHAS_EDITOR'):
        assert graphical_editor_ctrl.canvas.get_view_for_model(new_state_m)

    call_gui_callback(menubar_ctrl.prepare_destruction)


def test_copy_delete_bug(caplog):
    # testing_utils.run_gui(gui_config={'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': False, 'GAPHAS_EDITOR': True},
    #                       libraries={"ros": join(testing_utils.EXAMPLES_PATH, "libraries", "ros_libraries"),
    #                                  "turtle_libraries": join(testing_utils.EXAMPLES_PATH, "libraries", "turtle_libraries"),
    #                                  "generic": join(testing_utils.LIBRARY_SM_PATH, "generic")}
    #                       )
    #
    # try:
    #     trigger_copy_delete_bug_signals(rafcon.gui.singleton.state_machine_manager_model,
    #                                     rafcon.gui.singleton.main_window_controller)
    # finally:
    #     menubar_ctrl = rafcon.gui.singleton.main_window_controller.get_controller('menu_bar_controller')
    #     call_gui_callback(menubar_ctrl.on_quit_activate, None, None, True)
    #
    # testing_utils.shutdown_environment()
    # testing_utils.assert_logger_warnings_and_errors(caplog)

    libraries = {"ros": join(testing_utils.EXAMPLES_PATH, "libraries", "ros_libraries"),
                 "turtle_libraries": join(testing_utils.EXAMPLES_PATH, "libraries", "turtle_libraries"),
                 "generic": join(testing_utils.LIBRARY_SM_PATH, "generic")}
    change_in_gui_config = {'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': False, 'GAPHAS_EDITOR': True}
    testing_utils.initialize_environment(gui_config=change_in_gui_config, libraries=libraries)

    main_window_controller = MainWindowController(rafcon.gui.singleton.state_machine_manager_model, MainWindowView())

    # Wait for GUI to initialize
    testing_utils.wait_for_gui()

    thread = threading.Thread(target=trigger_copy_delete_bug_signals,
                              args=[rafcon.gui.singleton.state_machine_manager_model, main_window_controller])
    thread.start()
    gtk.main()
    logger.debug("after gtk main")
    thread.join()
    testing_utils.shutdown_environment(caplog=caplog)


if __name__ == '__main__':
    test_copy_delete_bug(None)
    # pytest.main(['-s', __file__])
