import threading
import shutil
from os.path import join

# general tool elements
from rafcon.utils import log
# test environment elements
import testing_utils
from testing_utils import call_gui_callback

import pytest

logger = log.get_logger(__name__)


def create_state_machine(*args, **kargs):
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.core.state_machine import StateMachine

    state1 = ExecutionState('State1', state_id='STATE1')
    state2 = ExecutionState('State2')
    state4 = ExecutionState('Nested')
    output_state4 = state4.add_output_data_port("out", "int")
    state5 = ExecutionState('Nested2')
    input_state5 = state5.add_input_data_port("in", "int", 0)
    state3 = HierarchyState(name='State3', state_id='STATE3')
    state3.add_state(state4)
    state3.add_state(state5)
    state3.set_start_state(state4)
    state3.add_scoped_variable("share", "int", 3)
    state3.add_transition(state4.state_id, 0, state5.state_id, None)
    state3.add_transition(state5.state_id, 0, state3.state_id, 0)
    state3.add_data_flow(state4.state_id, output_state4, state5.state_id, input_state5)

    ctr_state = HierarchyState(name="Container", state_id='ROOTSTATE')
    ctr_state.add_state(state1)
    ctr_state.add_state(state2)
    ctr_state.add_state(state3)
    ctr_state.set_start_state(state1)
    ctr_state.add_transition(state1.state_id, 0, state2.state_id, None)
    ctr_state.add_transition(state2.state_id, 0, state3.state_id, None)
    ctr_state.add_transition(state3.state_id, 0, ctr_state.state_id, 0)
    ctr_state.name = "Container"

    return StateMachine(ctr_state)


def focus_graphical_editor_in_page(page):
    from rafcon.gui.views.graphical_editor import GraphicalEditor as OpenGLEditor
    from rafcon.gui.mygaphas.view import ExtendedGtkView as GaphasEditor
    graphical_controller = page.children()[0]
    if not isinstance(graphical_controller, (OpenGLEditor, GaphasEditor)):
        graphical_controller = graphical_controller.children()[0]
    graphical_controller.grab_focus()


def check_order_and_consistency_of_menu(menubar_ctrl):
    import rafcon.gui.singleton
    from rafcon.gui.controllers.main_window import MenuBarController
    assert isinstance(menubar_ctrl, MenuBarController)
    recently_opened = rafcon.gui.singleton.global_runtime_config.get_config_value('recently_opened_state_machines')
    for index, elem in enumerate(menubar_ctrl.view.sub_menu_open_recently):
        if index in [0, 1]:
            continue
        print elem.get_label()
        assert recently_opened[index - 2] in elem.get_label()


@log.log_exceptions(None, gtk_quit=True)
def trigger_gui_signals(*args):
    """The function triggers and test basic functions of the menu bar.

    At the moment those functions are TESTED, SHOULD or are THOUGHT about:
    - TESTED menu-bar: New State Machine and save state machine (check in list and in menu) -> after save list updated
    - TESTED menu-bar: Open State Machine no library (check in list and in menu)
    - TESTED lib-tree: Open library State Machine (check in list and in menu)
    - THOUGHT sub-dialog-lib-tree: sub with library State Machine (check in list and in menu) - no update
    - THOUGHT sub-dialog-lib-tree: sub with library State Machine as template (check in list and in menu) - no update
    - SHOULD menu-bar: Save state machine with existing state machine no changes (check in list and in menu) - with update
    - TESTED menu-bar: Save changed state machine with existing state machine (check in list and in menu)
    - menu-bar: Save state machine as with existing state machine (check in list and in menu)
    - THOUGHT tool-bar: Save state machine with existing state machine no changes (check in list and in menu) - with update
    - THOUGHT tool-bar: Save changed state machine with existing state machine (check in list and in menu)
    - TESTED tool-bar: Save state machine as with existing state machine (check in list and in menu)
    - THOUGHT right-click menu: Save state as (check in list and in menu)
    - TESTED auto backup: recover state machine from backup functionality (check in list and in menu) - no update
    - TESTED menu bar: open state machine by recent opened sub-menu
    - TESTED menu bar: try to open state machine that is not there by recent opened sub-menu without fatal failure
    - THOUGHT menu-bar: refresh
    - THOUGHT tool-bar: refresh
    - THOUGHT tool-bar: refresh selected
    """
    # gui elements
    import rafcon.gui.singleton
    from rafcon.gui.controllers.main_window import MenuBarController
    from rafcon.gui.models.state_machine_manager import StateMachineManagerModel
    import rafcon.gui.helpers.state_machine as gui_helper_state_machine
    import rafcon.core.config
    from rafcon.core.states.library_state import LibraryState

    print "WT: ", threading.currentThread()
    sm_manager_model = rafcon.gui.singleton.state_machine_manager_model
    main_window_controller = rafcon.gui.singleton.main_window_controller
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
    global_runtime_config = rafcon.gui.singleton.global_runtime_config
    call_gui_callback(rafcon.core.singleton.state_machine_manager.add_state_machine, create_state_machine())
    assert isinstance(menubar_ctrl, MenuBarController)
    assert isinstance(sm_manager_model, StateMachineManagerModel)

    ####################
    # POSITIVE EXAMPLES -> supposed to be added to the recently opened state machines list
    ####################

    # menu-bar: New State Machine and save state machine (check in list and in menu) -> after save list updated
    call_gui_callback(testing_utils.wait_for_gui)
    current_sm_length = len(sm_manager_model.state_machines)
    first_sm_id = sm_manager_model.state_machines.keys()[0]
    call_gui_callback(menubar_ctrl.on_new_activate, None)
    call_gui_callback(sm_manager_model.__setattr__, "selected_state_machine_id", first_sm_id)

    call_gui_callback(testing_utils.wait_for_gui)
    assert len(sm_manager_model.state_machines) == current_sm_length + 1

    call_gui_callback(menubar_ctrl.on_save_as_activate, None, None, testing_utils.get_unique_temp_path())
    recently_opened_state_machines_paths = global_runtime_config.get_config_value('recently_opened_state_machines')
    assert sm_manager_model.state_machines[first_sm_id].state_machine.file_system_path == recently_opened_state_machines_paths[0]
    check_order_and_consistency_of_menu(menubar_ctrl)
    call_gui_callback(sm_manager_model.__setattr__, "selected_state_machine_id", first_sm_id + 1)

    call_gui_callback(menubar_ctrl.on_save_as_activate, None, None, testing_utils.get_unique_temp_path())
    call_gui_callback(testing_utils.wait_for_gui)
    recently_opened_state_machines_paths = global_runtime_config.get_config_value('recently_opened_state_machines')
    assert sm_manager_model.state_machines[first_sm_id+1].state_machine.file_system_path == recently_opened_state_machines_paths[0]

    # menu-bar: Open State Machine no library (check in list and in menu)
    basic_turtle_sm_path = join(testing_utils.TUTORIAL_PATH, "basic_turtle_demo_sm")
    call_gui_callback(menubar_ctrl.on_open_activate, None, None, basic_turtle_sm_path)
    call_gui_callback(testing_utils.wait_for_gui)
    recently_opened_state_machines_paths = global_runtime_config.get_config_value('recently_opened_state_machines')
    assert basic_turtle_sm_path == recently_opened_state_machines_paths[0]
    check_order_and_consistency_of_menu(menubar_ctrl)

    # menu-bar: Open State Machine no library and re-save it somewhere (check in list and in menu)
    turtle_state_machine_m = sm_manager_model.get_selected_state_machine_model()
    assert turtle_state_machine_m.state_machine.file_system_path == basic_turtle_sm_path

    call_gui_callback(menubar_ctrl.on_save_as_activate, None, None, testing_utils.get_unique_temp_path())
    recently_opened_state_machines_paths = global_runtime_config.get_config_value('recently_opened_state_machines')
    assert turtle_state_machine_m.state_machine.file_system_path == recently_opened_state_machines_paths[0]
    check_order_and_consistency_of_menu(menubar_ctrl)

    # lib-tree: Open library State Machine (check in list and
    library_path = join("generic", "dialog")
    library_name = "Dialog [3 options]"
    library_os_path = rafcon.gui.singleton.library_manager.get_os_path_to_library(library_path, library_name)[0]
    call_gui_callback(menubar_ctrl.on_open_activate, None, None, library_os_path)
    recently_opened_state_machines_paths = global_runtime_config.get_config_value('recently_opened_state_machines')
    call_gui_callback(testing_utils.wait_for_gui)
    lib_sm_m = sm_manager_model.get_selected_state_machine_model()
    assert library_os_path == recently_opened_state_machines_paths[0]
    assert lib_sm_m.state_machine.file_system_path == library_os_path
    check_order_and_consistency_of_menu(menubar_ctrl)

    call_gui_callback(sm_manager_model.__setattr__, "selected_state_machine_id", first_sm_id)
    call_gui_callback(sm_manager_model.get_selected_state_machine_model().selection.set,
                      sm_manager_model.get_selected_state_machine_model().root_state)
    call_gui_callback(menubar_ctrl.on_add_state_activate, None, None)
    call_gui_callback(menubar_ctrl.on_save_activate, None, None)
    assert sm_manager_model.state_machines[first_sm_id].state_machine.file_system_path == recently_opened_state_machines_paths[0]
    assert library_os_path == recently_opened_state_machines_paths[1]
    check_order_and_consistency_of_menu(menubar_ctrl)

    # open state machine by recent opened sub-menu in menu bar
    call_gui_callback(global_runtime_config.update_recently_opened_state_machines_with,
                      sm_manager_model.state_machines[first_sm_id].state_machine)
    first_sm_path = sm_manager_model.state_machines[first_sm_id].state_machine.file_system_path
    call_gui_callback(testing_utils.wait_for_gui)
    assert first_sm_path in menubar_ctrl.view.sub_menu_open_recently.get_children()[2].get_label()
    call_gui_callback(sm_manager_model.state_machine_manager.remove_state_machine, first_sm_id)
    call_gui_callback(menubar_ctrl.view.sub_menu_open_recently.get_children()[2].activate)
    call_gui_callback(testing_utils.wait_for_gui)
    reopen_first_sm_id = sm_manager_model.selected_state_machine_id
    assert sm_manager_model.state_machines[reopen_first_sm_id].state_machine.file_system_path == first_sm_path
    check_order_and_consistency_of_menu(menubar_ctrl)
    # clean check after every update and re-save of library (both paths are in -> after enforcing by marked dirty flag)
    # TODO think about to remove this enforcement by marked dirty flag
    # change name to set marked dirty flag and enforce update of recent opened
    call_gui_callback(lib_sm_m.state_machine.root_state.__setattr__, "name", "Different")
    shutil.rmtree(sm_manager_model.state_machines[reopen_first_sm_id].state_machine.file_system_path)
    call_gui_callback(sm_manager_model.__setattr__, "selected_state_machine_id", lib_sm_m.state_machine.state_machine_id)
    call_gui_callback(testing_utils.wait_for_gui)
    call_gui_callback(menubar_ctrl.on_save_as_activate, None, None, testing_utils.get_unique_temp_path())
    print recently_opened_state_machines_paths
    assert lib_sm_m.state_machine.file_system_path == recently_opened_state_machines_paths[0]
    print recently_opened_state_machines_paths, library_os_path, sm_manager_model.state_machines[reopen_first_sm_id].state_machine.file_system_path
    assert sm_manager_model.state_machines[reopen_first_sm_id].state_machine.file_system_path == recently_opened_state_machines_paths[1]
    call_gui_callback(global_runtime_config.clean_recently_opened_state_machines)
    assert not sm_manager_model.state_machines[reopen_first_sm_id].state_machine.file_system_path == recently_opened_state_machines_paths[1]
    assert library_os_path == recently_opened_state_machines_paths[1]
    check_order_and_consistency_of_menu(menubar_ctrl)

    ####################
    # NEGATIVE EXAMPLES -> supposed to not been added to the recently opened state machines list
    ####################
    recently_opened_state_machines_paths = global_runtime_config.get_config_value('recently_opened_state_machines')

    # if a LibraryState is created and insert no change should be in the recently opened state machine list
    call_gui_callback(menubar_ctrl.on_new_activate, None)
    lib_state = LibraryState(join("generic", "dialog"), "Dialog [3 options]", "0.1", "Dialog [3 options]")
    call_gui_callback(gui_helper_state_machine.insert_state_into_selected_state, lib_state, True)
    assert recently_opened_state_machines_paths == global_runtime_config.get_config_value('recently_opened_state_machines')

    # try to open state machine that is not there -> no fatal failure
    print "OPEN FAILURE CASE"
    call_gui_callback(global_runtime_config.update_recently_opened_state_machines_with, lib_sm_m.state_machine)
    lib_sm_path = lib_sm_m.state_machine.file_system_path
    shutil.rmtree(lib_sm_m.state_machine.file_system_path)
    lib_sm_before_remove = lib_sm_m.state_machine
    call_gui_callback(sm_manager_model.state_machine_manager.remove_state_machine, lib_sm_m.state_machine.state_machine_id)
    call_gui_callback(testing_utils.wait_for_gui)
    call_gui_callback(menubar_ctrl.view.sub_menu_open_recently.get_children()[2].activate)
    # was not open
    selected_sm_id = sm_manager_model.selected_state_machine_id
    assert not sm_manager_model.state_machines[selected_sm_id].state_machine.file_system_path == lib_sm_path
    # is still in and after clean removed
    assert lib_sm_path in menubar_ctrl.view.sub_menu_open_recently.get_children()[2].get_label()
    call_gui_callback(global_runtime_config.clean_recently_opened_state_machines)
    assert lib_sm_path not in menubar_ctrl.view.sub_menu_open_recently.get_children()[2].get_label()
    call_gui_callback(global_runtime_config.update_recently_opened_state_machines_with, lib_sm_before_remove)
    assert 'NOT_ACCESSIBLE' in menubar_ctrl.view.sub_menu_open_recently.get_children()[2].get_label()

    # TODO maybe finally move this into the auto-backup or restore test module
    print "AUTO BACKUP TEST"
    number_of_open_sm = len(sm_manager_model.state_machines)
    backup_path = sm_manager_model.state_machines[reopen_first_sm_id].auto_backup.meta['last_backup']['file_system_path']
    from rafcon.gui.models import auto_backup
    call_gui_callback(auto_backup.recover_state_machine_from_backup, backup_path)
    assert recently_opened_state_machines_paths == global_runtime_config.get_config_value('recently_opened_state_machines')
    assert number_of_open_sm == len(sm_manager_model.state_machines)
    call_gui_callback(sm_manager_model.state_machine_manager.remove_state_machine, reopen_first_sm_id)
    call_gui_callback(testing_utils.wait_for_gui)
    assert recently_opened_state_machines_paths == global_runtime_config.get_config_value('recently_opened_state_machines')
    assert number_of_open_sm == len(sm_manager_model.state_machines) + 1
    call_gui_callback(auto_backup.recover_state_machine_from_backup, backup_path, None, None, True)
    assert recently_opened_state_machines_paths == global_runtime_config.get_config_value('recently_opened_state_machines')
    assert number_of_open_sm == len(sm_manager_model.state_machines)
    check_order_and_consistency_of_menu(menubar_ctrl)

original_auto_backup = None


def patch_backup_threading():
    import rafcon.gui.models.auto_backup
    from rafcon.gui.models.auto_backup import AutoBackupModel
    global original_auto_backup
    original_auto_backup = rafcon.gui.models.auto_backup.AutoBackupModel
    testing_utils.auto_backup_threads = []

    class PatchedAutoBackupModel(AutoBackupModel):

        def perform_temp_storage(self):
            testing_utils.auto_backup_threads.append(threading.currentThread())
            AutoBackupModel.perform_temp_storage(self)

    rafcon.gui.models.auto_backup.AutoBackupModel = PatchedAutoBackupModel


def unpatch_backup_threading():
    import rafcon.gui.models.auto_backup
    global original_auto_backup, auto_backup_threads
    rafcon.gui.models.auto_backup.AutoBackupModel = original_auto_backup
    testing_utils.auto_backup_threads = []


def test_recent_opened_state_machine_list(caplog):
    change_in_gui_config = {'AUTO_BACKUP_ENABLED': True, 'HISTORY_ENABLED': False}

    libraries = {"ros": join(testing_utils.EXAMPLES_PATH, "libraries", "ros_libraries"),
                 "turtle_libraries": join(testing_utils.EXAMPLES_PATH, "libraries", "turtle_libraries"),
                 "generic": join(testing_utils.LIBRARY_SM_PATH, "generic")}
    testing_utils.run_gui(gui_config=change_in_gui_config, libraries=libraries)  # , patch_threading=False)
    call_gui_callback(patch_backup_threading)
    try:
        trigger_gui_signals()
    except:
        raise
    finally:
        call_gui_callback(unpatch_backup_threading)
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=0, expected_errors=1)  # , unpatch_threading=False)


if __name__ == '__main__':
    test_recent_opened_state_machine_list(None)
    # pytest.main(['-s', __file__])
