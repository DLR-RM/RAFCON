import gtk
import threading
from os.path import join, exists

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
    import rafcon.core.singleton
    from rafcon.gui.controllers.main_window import MenuBarController
    assert isinstance(menubar_ctrl, MenuBarController)
    recently_opened = rafcon.gui.singleton.global_runtime_config.get_config_value('recently_opened_state_machines')
    for index, elem in enumerate(menubar_ctrl.view.sub_menu_open_recently):
        if index in [0, 1]:
            continue
        assert recently_opened[index - 2] in elem.get_label()

CORE_HASH_INDEX = 0
GUI_HASH_INDEX = 1
PATH_INDEX = 2
PAGE_NUMBER_INDEX = 3
MARKED_DIRTY_INDEX = 4


def prepare_tab_data_of_open_state_machines(main_window_controller, sm_manager_model, open_state_machines):
    state_machines_editor_ctrl = main_window_controller.get_controller('state_machines_editor_ctrl')
    number_of_pages = state_machines_editor_ctrl.view['notebook'].get_n_pages()
    for page_number in range(number_of_pages):
        page = state_machines_editor_ctrl.view['notebook'].get_nth_page(page_number)
        sm_id = state_machines_editor_ctrl.get_state_machine_id_for_page(page)
        if sm_id == sm_manager_model.selected_state_machine_id:
            open_state_machines['selected_sm_page_number'] = page_number
        sm_tuple = (sm_manager_model.state_machines[sm_id].state_machine.mutable_hash().hexdigest(),
                    sm_manager_model.state_machines[sm_id].mutable_hash().hexdigest(),
                    sm_manager_model.state_machines[sm_id].state_machine.file_system_path,
                    page_number,
                    sm_manager_model.state_machines[sm_id].state_machine.marked_dirty)
        open_state_machines['list_of_hash_path_tab_page_number_tuple'].append(sm_tuple)
    selected_sm = sm_manager_model.get_selected_state_machine_model()
    if selected_sm and selected_sm.selection.states:
        open_state_machines['selection_state_machine'] = selected_sm.selection.get_selected_state().state.get_path()
    else:
        open_state_machines['selection_state_machine'] = None


@log.log_exceptions(None, gtk_quit=True)
def trigger_gui_signals_first_run(*args):
    """The function triggers the creation of different state machines that should be backup-ed.
    In another run those are restored and checked onto correctness.

    At the moment TESTED, SHOULD or are THOUGHT about to generate the following state machines:
    - TESTED new state machine without storage
    - TESTED new state machine with storage and no changes
    - TESTED new state machine with storage and changes
    - TESTED state machine loaded and no changes
    - TESTED state machine loaded and changes
    - TESTED library not changed
    - TESTED library changed
    - TESTED change tab position
    - SHOULD not stored state machine that was removed/moved before restart
    - SHOULD stored state machine and no changes that was removed/moved before restart
    - SHOULD stored state machine and no changes that was removed/moved before restart
    """
    import rafcon.gui.singleton
    from rafcon.gui.controllers.main_window import MenuBarController
    from rafcon.gui.models.state_machine_manager import StateMachineManagerModel
    from gui.widget.test_state_type_change import get_state_editor_ctrl_and_store_id_dict

    testing_utils.wait_for_gui()
    main_window_controller = rafcon.gui.singleton.main_window_controller
    sm_manager_model = rafcon.gui.singleton.state_machine_manager_model
    open_state_machines = args[0]
    library_manager = rafcon.gui.singleton.library_manager
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
    assert isinstance(menubar_ctrl, MenuBarController)
    assert isinstance(sm_manager_model, StateMachineManagerModel)

    def add_two_states_to_root_state_of_selected_state_machine():
        sm_m = sm_manager_model.get_selected_state_machine_model()
        current_number_states = len(sm_m.root_state.states)
        call_gui_callback(sm_m.selection.set, sm_m.root_state)
        call_gui_callback(menubar_ctrl.on_add_state_activate, None)
        call_gui_callback(menubar_ctrl.on_add_state_activate, None)
        assert len(sm_m.root_state.states) == current_number_states + 2
        assert sm_manager_model.get_selected_state_machine_model().state_machine.marked_dirty

    ####################
    # POSITIVE EXAMPLES -> supposed to be added to the open tabs list
    ####################

    # new state machine without storage
    state_machine = create_state_machine()
    call_gui_callback(sm_manager_model.state_machine_manager.add_state_machine, state_machine)
    call_gui_callback(testing_utils.wait_for_gui)
    print sm_manager_model.state_machines.keys()
    current_sm_id = sm_manager_model.state_machines.keys()[0]
    current_number_of_sm = len(sm_manager_model.state_machines)

    # new state machine with storage and no changes
    current_number_of_sm += 1
    current_sm_id += 1
    call_gui_callback(menubar_ctrl.on_new_activate, None)
    call_gui_callback(sm_manager_model.__setattr__, 'selected_state_machine_id', current_sm_id)
    assert len(sm_manager_model.state_machines) == current_number_of_sm
    call_gui_callback(menubar_ctrl.on_save_as_activate, None, None, testing_utils.get_unique_temp_path())

    # new state machine with storage and with changes
    current_number_of_sm += 1
    current_sm_id += 1
    call_gui_callback(menubar_ctrl.on_new_activate, None)
    call_gui_callback(sm_manager_model.__setattr__, 'selected_state_machine_id', current_sm_id)
    assert len(sm_manager_model.state_machines) == current_number_of_sm
    call_gui_callback(menubar_ctrl.on_save_as_activate, None, None, testing_utils.get_unique_temp_path())
    add_two_states_to_root_state_of_selected_state_machine()

    # state machine loaded and no changes
    current_number_of_sm += 1
    current_sm_id += 1
    basic_turtle_sm_path = join(testing_utils.TUTORIAL_PATH, "basic_turtle_demo_sm")
    call_gui_callback(menubar_ctrl.on_open_activate, None, None, basic_turtle_sm_path)
    call_gui_callback(sm_manager_model.__setattr__, 'selected_state_machine_id', current_sm_id)
    move_this_sm_id = sm_manager_model.selected_state_machine_id
    assert len(sm_manager_model.state_machines) == current_number_of_sm

    # state machine loaded and changes
    current_number_of_sm += 1
    current_sm_id += 1
    print "BUGS"
    basic_turtle_sm_path = join(testing_utils.TUTORIAL_PATH, "99_bugs")
    call_gui_callback(menubar_ctrl.on_open_activate, None, None, basic_turtle_sm_path)
    call_gui_callback(testing_utils.wait_for_gui)
    assert len(sm_manager_model.state_machines) == current_number_of_sm
    assert sm_manager_model.get_selected_state_machine_model().state_machine.file_system_path == basic_turtle_sm_path
    add_two_states_to_root_state_of_selected_state_machine()

    # library not changed (needs state machine that has meta data already -> that should not be changed by opening)
    print "LIB no changes"
    library_os_path = library_manager.get_os_path_to_library("turtle_libraries", "clear_field")[0]
    call_gui_callback(menubar_ctrl.on_open_activate, None, None, library_os_path)
    call_gui_callback(testing_utils.wait_for_gui)
    assert sm_manager_model.get_selected_state_machine_model().state_machine.marked_dirty

    # library with changes
    print "LIB with changes"
    library_os_path = library_manager.get_os_path_to_library("turtle_libraries", "teleport_turtle")[0]
    call_gui_callback(menubar_ctrl.on_open_activate, None, None, library_os_path)
    call_gui_callback(testing_utils.wait_for_gui)
    lib_sm_m = sm_manager_model.get_selected_state_machine_model()

    def do_type_change():
        [state_editor_ctrl, list_store_id_from_state_type_dict] = \
            get_state_editor_ctrl_and_store_id_dict(lib_sm_m, lib_sm_m.root_state, main_window_controller, 5., logger)
        # - do state type change
        state_type_row_id = list_store_id_from_state_type_dict['HIERARCHY']
        state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active(state_type_row_id)

    call_gui_callback(lib_sm_m.selection.set, [lib_sm_m.root_state])
    print lib_sm_m.root_state
    call_gui_callback(do_type_change)
    print lib_sm_m.root_state
    add_two_states_to_root_state_of_selected_state_machine()
    print lib_sm_m.root_state

    # change tab position
    state_machines_editor_ctrl = main_window_controller.get_controller('state_machines_editor_ctrl')
    call_gui_callback(state_machines_editor_ctrl.rearrange_state_machines, {move_this_sm_id: 0})

    # defined selection
    sm_m = sm_manager_model.state_machines[move_this_sm_id]
    call_gui_callback(sm_manager_model.__setattr__, 'selected_state_machine_id', move_this_sm_id)
    call_gui_callback(sm_m.selection.set, sm_m.root_state.states.values()[0])
    print "last state machine:", sm_m.state_machine.file_system_path

    ####################
    # NEGATIVE EXAMPLES -> supposed to not been added to the recently opened state machines list
    ####################

    # state machine that was removed/moved before restart -> result in second run

    ####################
    # collect open state machine data
    ####################
    call_gui_callback(prepare_tab_data_of_open_state_machines, main_window_controller, sm_manager_model, open_state_machines)

    ####################
    # shout down gui
    ####################
    call_gui_callback(menubar_ctrl.on_stop_activate, None)  # TODO why this is some how important for correct restore


@log.log_exceptions(None, gtk_quit=True)
def trigger_gui_signals_second_run(*args):
    import rafcon.gui.singleton
    main_window_controller = rafcon.gui.singleton.main_window_controller
    sm_manager_model = rafcon.gui.singleton.state_machine_manager_model
    open_state_machines = args[0]
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
    import rafcon.gui.backup.session as backup_session
    if rafcon.gui.singleton.global_gui_config.get_config_value("SESSION_RESTORE_ENABLED"):
        call_gui_callback(backup_session.restore_session_from_runtime_config)
    print "restore config", rafcon.gui.singleton.global_runtime_config.config_file_path
    with open(rafcon.gui.singleton.global_runtime_config.config_file_path, 'r') as f:
        found_flag = False
        print "\n"*5, "#"*20
        for line in f:
            if "open_tabs" in line:
                found_flag = True
            if found_flag:
                print line
        print "#"*20, "\n"*5
    call_gui_callback(testing_utils.wait_for_gui)
    call_gui_callback(prepare_tab_data_of_open_state_machines,
                      main_window_controller, sm_manager_model, open_state_machines)
    call_gui_callback(backup_session.reset_session)


def test_restore_session(caplog):
    from rafcon.core.storage import storage

    change_in_gui_config = {'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': False,
                            'SESSION_RESTORE_ENABLED': True, 'GAPHAS_EDITOR': True}

    # first run
    libraries = {"ros": join(testing_utils.EXAMPLES_PATH, "libraries", "ros_libraries"),
                 "turtle_libraries": join(testing_utils.EXAMPLES_PATH, "libraries", "turtle_libraries"),
                 "generic": join(testing_utils.LIBRARY_SM_PATH, "generic")}
    testing_utils.run_gui(gui_config=change_in_gui_config, libraries=libraries)
    try:
        open_state_machines = {'list_of_hash_path_tab_page_number_tuple': [], 'selected_sm_page_number': None}
        trigger_gui_signals_first_run(open_state_machines)
    finally:
        testing_utils.close_gui(force_quit=False)
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=0, expected_errors=0)

    # second run
    libraries = {"ros": join(testing_utils.EXAMPLES_PATH, "libraries", "ros_libraries"),
                 "turtle_libraries": join(testing_utils.EXAMPLES_PATH, "libraries", "turtle_libraries"),
                 "generic": join(testing_utils.LIBRARY_SM_PATH, "generic")}
    testing_utils.run_gui(gui_config=change_in_gui_config, libraries=libraries)

    try:
        final_open_state_machines = {'list_of_hash_path_tab_page_number_tuple': [], 'selected_sm_page_number': None}
        trigger_gui_signals_second_run(final_open_state_machines)
    except:
        raise
    finally:
        testing_utils.close_gui()

    print open_state_machines
    print final_open_state_machines

    # test selection, page number and path
    # TODO find if there is a proper hash value test
    # TODO find out why core and gui hashes are changing !!! not even fully deterministic !!!
    # TODO find out why dirty flag is once wrong when AUTO_BACKUP is enabled in parallel
    #      (is connected to direct storing while opening)
    assert open_state_machines['selected_sm_page_number'] == final_open_state_machines['selected_sm_page_number']

    final_tuple_list = final_open_state_machines['list_of_hash_path_tab_page_number_tuple']

    assert open_state_machines['selection_state_machine'] == final_open_state_machines['selection_state_machine']

    order_of_pages_to_be_dirty = [False, True, False, False, True, False, True]
    for index, sm_tuple in enumerate(open_state_machines['list_of_hash_path_tab_page_number_tuple']):
        assert index == sm_tuple[PAGE_NUMBER_INDEX]
        if not final_tuple_list[index][CORE_HASH_INDEX] == sm_tuple[CORE_HASH_INDEX]:
            print "CORE hashes page {4} are not equal: {0} != {1}, path: {2} {3}" \
                  "".format(final_tuple_list[index][CORE_HASH_INDEX], sm_tuple[CORE_HASH_INDEX],
                            sm_tuple[PATH_INDEX], sm_tuple[MARKED_DIRTY_INDEX], sm_tuple[PAGE_NUMBER_INDEX])
            if sm_tuple[PATH_INDEX]:
                sm_file_path = join(sm_tuple[PATH_INDEX], storage.STATEMACHINE_FILE)
                if exists(sm_tuple[PATH_INDEX]) and exists(sm_file_path):
                    print "sm_file_path: ", sm_file_path
                    print storage.load_data_file(join(sm_tuple[PATH_INDEX], storage.STATEMACHINE_FILE))
                else:
                    print "does not exist sm_file_path ", sm_file_path
            else:
                print "state machine was NOT stored"
        assert final_tuple_list[index][CORE_HASH_INDEX] == sm_tuple[CORE_HASH_INDEX]
        assert final_tuple_list[index][PATH_INDEX] == sm_tuple[PATH_INDEX]
        if not final_tuple_list[index][GUI_HASH_INDEX] == sm_tuple[GUI_HASH_INDEX]:
            print "GUI hashes page {4} are not equal: {0} != {1}, path: {2} {3}" \
                  "".format(final_tuple_list[index][GUI_HASH_INDEX], sm_tuple[GUI_HASH_INDEX],
                            sm_tuple[PATH_INDEX], sm_tuple[MARKED_DIRTY_INDEX], sm_tuple[PAGE_NUMBER_INDEX])
        assert final_tuple_list[index][GUI_HASH_INDEX] == sm_tuple[GUI_HASH_INDEX]
        assert final_tuple_list[index][PAGE_NUMBER_INDEX] == sm_tuple[PAGE_NUMBER_INDEX]
        # page dirty 0, 4, 6 and not dirty 1, 2, 3, 5
        if not final_tuple_list[index][MARKED_DIRTY_INDEX] == sm_tuple[MARKED_DIRTY_INDEX]:
            print "MARKED DIRTY page {4} is not equal: {0} != {1}, path: {2} {3}" \
                  "".format(final_tuple_list[index][MARKED_DIRTY_INDEX], sm_tuple[MARKED_DIRTY_INDEX],
                            sm_tuple[PATH_INDEX], sm_tuple[MARKED_DIRTY_INDEX], sm_tuple[PAGE_NUMBER_INDEX])
        assert final_tuple_list[index][MARKED_DIRTY_INDEX] == sm_tuple[MARKED_DIRTY_INDEX]
        if not final_tuple_list[index][MARKED_DIRTY_INDEX] == order_of_pages_to_be_dirty[index]:
            print "Aspect different dirty flag page {4} is not equal: {0} != {1}, path: {2} {3}" \
                  "".format(final_tuple_list[index][MARKED_DIRTY_INDEX], order_of_pages_to_be_dirty[index],
                            sm_tuple[PATH_INDEX], sm_tuple[MARKED_DIRTY_INDEX], sm_tuple[PAGE_NUMBER_INDEX])
        # TODO check to put here an assert, too, -> maybe the implementation of this check is bad (because tabs look OK)

    testing_utils.shutdown_environment(caplog=caplog, expected_warnings=0, expected_errors=0)


if __name__ == '__main__':
    # test_restore_session(None)
    pytest.main(['-s', __file__])
