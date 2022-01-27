from os.path import join

from rafcon.utils import log

from tests import utils as testing_utils
from tests.utils import focus_graphical_editor_in_page

import pytest

logger = log.get_logger(__name__)


def create_state_machine():
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


def select_and_paste_state(gui, state_machine_model, source_state_model, target_state_model, menu_bar_ctrl, operation,
                           main_window_controller, page):
    """Select a particular state and perform an operation on it (Copy or Cut) and paste it somewhere else. At the end,
    verify that the operation was completed successfully.

    :param state_machine_model: The state machine model where the operation will be conducted
    :param source_state_model: The state model, on which the operation will be performed
    :param target_state_model: The state model, where the source state will be pasted
    :param menu_bar_ctrl: The menu_bar controller, through which copy, cut & paste actions are triggered
    :param operation: String indicating the operation to be performed (Copy or Cut)
    :param main_window_controller: The MainWindow Controller
    :param page: The notebook page of the corresponding state machine in the state machines editor
    :return: The target state model, and the child state count before pasting
    """
    print("\n\n %s \n\n" % source_state_model.state.name)
    gui(state_machine_model.selection.set, [source_state_model])
    gui(getattr(menu_bar_ctrl, 'on_{}_selection_activate'.format(operation)), None, None)
    print("\n\n %s \n\n" % target_state_model.state.name)
    gui(state_machine_model.selection.set, [target_state_model])
    old_child_state_count = len(target_state_model.state.states)
    main_window_controller.view['main_window'].grab_focus()
    focus_graphical_editor_in_page(page)
    gui(menu_bar_ctrl.on_paste_clipboard_activate, None, None)
    testing_utils.wait_for_gui()
    print(list(target_state_model.state.states.keys()))
    assert len(target_state_model.state.states) == old_child_state_count + 1
    return target_state_model, old_child_state_count


def copy_and_paste_state_into_itself(gui, sm_m, state_m_to_copy, page, menu_bar_ctrl):
    gui(sm_m.selection.set, [state_m_to_copy])
    focus_graphical_editor_in_page(page)
    gui(menu_bar_ctrl.on_copy_selection_activate, None, None)
    old_child_state_count = len(state_m_to_copy.state.states)
    gui(sm_m.selection.set, [state_m_to_copy])
    focus_graphical_editor_in_page(page)
    gui(menu_bar_ctrl.on_paste_clipboard_activate, None, None)
    assert len(state_m_to_copy.state.states) == old_child_state_count + 1


def copy_port(gui, state_machine_model, port_m, menu_bar_ctrl):
    print("Copying {}".format(port_m))
    gui(state_machine_model.selection.set, [port_m])
    gui(getattr(menu_bar_ctrl, 'on_copy_selection_activate'), None, None)


def paste(gui, state_machine_model, state_m, main_window_controller, menu_bar_ctrl, page):
    print("Pasting")
    gui(state_machine_model.selection.set, [state_m])
    main_window_controller.view['main_window'].grab_focus()
    focus_graphical_editor_in_page(page)
    gui(menu_bar_ctrl.on_paste_clipboard_activate, None, None)
    testing_utils.wait_for_gui()


@pytest.mark.timeout(60)
@pytest.mark.unstable35
@pytest.mark.unstable37
@pytest.mark.parametrize('gui', [{"libraries": {
    "ros": join(testing_utils.EXAMPLES_PATH, "libraries", "ros_libraries"),
    "turtle_libraries": join(testing_utils.EXAMPLES_PATH, "libraries", "turtle_libraries")
}}], indirect=True, ids=["with ros and turtle libraries"])
def test_gui(gui):
    trigger_menu_bar_items(gui)


def trigger_menu_bar_items(gui, with_refresh=True, with_substitute_library=True):
    """The function triggers and test basic functions of the menu bar.

    At the moment those functions are tested:
    - New State Machine
    - Open State Machine
    - Copy State/HierarchyState -> via GraphicalEditor
    - Cut State/HierarchyState -> via GraphicalEditor
    - Paste State/HierarchyState -> via GraphicalEditor
    - Refresh Libraries
    - Refresh All
    - Save as
    - Stop State Machine
    - Quit GUI
    """
    from rafcon.core.states.library_state import LibraryState
    import rafcon.core.singleton
    import rafcon.gui.singleton
    import rafcon.gui.helpers.state as gui_helper_state
    import rafcon.gui.helpers.state_machine as gui_helper_state_machine
    from rafcon.gui.controllers.library_tree import LibraryTreeController
    from rafcon.core.states.barrier_concurrency_state import UNIQUE_DECIDER_STATE_ID
    from rafcon.core.states.state import StateType
    sm_manager_model = rafcon.gui.singleton.state_machine_manager_model
    main_window_controller = rafcon.gui.singleton.main_window_controller
    menubar_ctrl = main_window_controller.menu_bar_controller
    lib_tree_ctrl = main_window_controller.library_controller
    state_machines_ctrl = main_window_controller.state_machines_editor_ctrl
    assert isinstance(lib_tree_ctrl, LibraryTreeController)

    state_machine = create_state_machine()
    first_sm_id = state_machine.state_machine_id
    gui(rafcon.core.singleton.state_machine_manager.add_state_machine, state_machine)

    current_sm_length = len(sm_manager_model.state_machines)
    gui(menubar_ctrl.on_new_activate, None)

    # test decider state removal of barrier state
    sm_m = sm_manager_model.state_machines[first_sm_id + 1]
    page = state_machines_ctrl.get_page_for_state_machine_id(sm_m.state_machine_id)

    # Tests for issue #712
    # Copy an InputDataPort from a root state to an ExecutionState already having an InputDataPort
    gui(sm_m.selection.set, [sm_m.root_state])
    gui(gui_helper_state_machine.add_new_state, sm_m, StateType.EXECUTION)
    execution_state_m = list(sm_m.root_state.states.values())[0]
    gui(sm_m.root_state.state.add_input_data_port, "i1", int, data_port_id=0)
    gui(execution_state_m.state.add_input_data_port, "i1", int, data_port_id=0)
    root_input_port_m = sm_m.root_state.get_input_data_port_m(0)
    copy_port(gui, sm_m, root_input_port_m, menubar_ctrl)
    paste(gui, sm_m, execution_state_m, main_window_controller, menubar_ctrl, page)
    gui(sm_m.selection.set, execution_state_m)
    gui(menubar_ctrl.on_delete_activate, None, None)

    # TODO check why I have had to move this test above the bug tests below?!
    # Create BarrierConcurrencyState and try to delete DeciderState (should fail with exception)
    gui(sm_m.selection.set, [sm_m.root_state])
    gui(gui_helper_state_machine.add_new_state, sm_m, StateType.BARRIER_CONCURRENCY)
    barrier_state_m = list(sm_m.root_state.states.values())[0]
    decider_state_path = "/".join([barrier_state_m.state.get_path(), UNIQUE_DECIDER_STATE_ID])
    gui(sm_m.selection.set, sm_m.get_state_model_by_path(decider_state_path))
    gui(menubar_ctrl.on_delete_activate, None, None)
    gui.expected_errors += 1
    gui(sm_m.root_state.state.remove_state, barrier_state_m.state.state_id)

    # Tests for issue #717 and #726
    # create self transition and self data flow and perform state type change and substitute state
    gui(sm_m.selection.set, [sm_m.root_state])
    gui(gui_helper_state_machine.add_new_state, sm_m, StateType.EXECUTION)
    execution_state_m = list(sm_m.root_state.states.values())[0]
    new_state_id = execution_state_m.state.state_id
    idp_id = gui(execution_state_m.state.add_input_data_port, "i1", int, data_port_id=0)
    odp_id = gui(execution_state_m.state.add_output_data_port, "o1", int, data_port_id=1)
    # add self transition and self data flow
    gui(execution_state_m.parent.state.add_transition, new_state_id, 0, new_state_id, None)
    gui(execution_state_m.parent.state.add_data_flow, new_state_id, odp_id, new_state_id, idp_id)
    gui(sm_m.selection.set, execution_state_m)
    # test issue #717
    from tests.gui.widget.test_state_type_change import get_state_editor_ctrl_and_store_id_dict
    from rafcon.core.states.hierarchy_state import HierarchyState
    [state_editor_ctrl, list_store_id_from_state_type_dict] = gui(get_state_editor_ctrl_and_store_id_dict,
                                                                  sm_m, execution_state_m, main_window_controller, 5.,
                                                                  logger)
    state_type_row_id = list_store_id_from_state_type_dict[HierarchyState.__name__]
    gui(state_editor_ctrl.get_controller('properties_ctrl').view['type_combobox'].set_active, state_type_row_id)
    new_state = sm_m.root_state.states[new_state_id]
    assert len(new_state.parent.transitions) == 1 and len(new_state.parent.data_flows) == 1
    # test issue #726
    hierarchy_state_m = sm_m.root_state.states[new_state_id]
    gui(hierarchy_state_m.outcomes[0].__setattr__, "name", "end")  # rename to let the transition survive
    gui(sm_m.selection.set, hierarchy_state_m)
    library_path, library_name = ('generic', 'wait')
    gui(lib_tree_ctrl.select_library_tree_element_of_lib_tree_path, join(library_path, library_name))
    gui(lib_tree_ctrl.substitute_as_library_clicked, None, True)
    assert len(sm_m.root_state.states) == 1
    wait_state_m = list(sm_m.root_state.states.values())[0]
    assert len(new_state.parent.transitions) == 1
    t_m = list(new_state.parent.transitions)[0]
    assert t_m.transition.from_state == t_m.transition.to_state and \
           t_m.transition.from_state == wait_state_m.state.state_id
    gui(sm_m.root_state.state.remove_state, wait_state_m.state.state_id)
    first_sm_id += 1  # count state machine id once up because of library substitution (loads a state machine, too)

    assert len(sm_manager_model.state_machines) == current_sm_length + 1
    gui(menubar_ctrl.on_open_activate, None, None, join(testing_utils.TUTORIAL_PATH, "basic_turtle_demo_sm"))
    gui(testing_utils.wait_for_gui)
    assert len(sm_manager_model.state_machines) == current_sm_length + 2

    sm_m = sm_manager_model.state_machines[first_sm_id + 2]
    testing_utils.wait_for_gui()
    # MAIN_WINDOW NEEDS TO BE FOCUSED (for global input focus) TO OPERATE PASTE IN GRAPHICAL VIEWER
    main_window_controller.view['main_window'].grab_focus()
    gui(sm_manager_model.__setattr__, "selected_state_machine_id", first_sm_id + 2)
    page_id = state_machines_ctrl.get_page_num(first_sm_id + 2)
    page = state_machines_ctrl.view.notebook.get_nth_page(page_id)
    gui(focus_graphical_editor_in_page, page)
    print("#" * 30, "\n", '#### group states \n', "#" * 30, "\n")
    state_m_parent = sm_m.get_state_model_by_path('CDMJPK/RMKGEW/KYENSZ')
    state_ids_old = [state_id for state_id in state_m_parent.state.states]
    state_m_list = [state_m_parent.states[child_state_id] for child_state_id in ['PAYECU', 'UEPNNW', 'KQDJYS']]
    gui(gui_helper_state.group_states_and_scoped_variables, state_m_list, [])

    ##########################################################
    # ungroup new state
    print("#" * 30, "\n", '#### ungroup state \n', "#" * 30, "\n")
    new_state = None
    for state_id in state_m_parent.state.states:
        if state_id not in state_ids_old:
            new_state = state_m_parent.state.states[state_id]
    gui(gui_helper_state.ungroup_state, sm_m.get_state_model_by_path(new_state.get_path()))

    #########################################################
    print("select & copy an execution state -> and paste it somewhere")
    select_and_paste_state(gui, sm_m, sm_m.get_state_model_by_path('CDMJPK/RMKGEW/KYENSZ'),
                           sm_m.get_state_model_by_path(
                               'CDMJPK/RMKGEW'), menubar_ctrl, 'copy', main_window_controller, page)

    ###########################################################
    print("select & copy a hierarchy state -> and paste it some where")
    select_and_paste_state(gui, sm_m, sm_m.get_state_model_by_path('CDMJPK/RMKGEW/KYENSZ/VCWTIY'),
                           sm_m.get_state_model_by_path('CDMJPK'), menubar_ctrl, 'copy', main_window_controller, page)

    ##########################################################
    print("select a library state -> and paste it some where WITH CUT !!!")
    state_m, old_child_state_count = select_and_paste_state(gui, sm_m,
                                                            sm_m.get_state_model_by_path('CDMJPK/RMKGEW/KYENSZ/VCWTIY'),
                                                            sm_m.get_state_model_by_path('CDMJPK'), menubar_ctrl, 'cut',
                                                            main_window_controller, page)

    ##########################################################
    # create complex state with all elements
    gui(sm_m.selection.set, [sm_m.get_state_model_by_path('CDMJPK'), ])
    lib_state = LibraryState(join("generic", "dialog"), "Dialog [3 options]", "0.1", "Dialog [3 options]")
    gui(gui_helper_state_machine.insert_state_into_selected_state, lib_state, True)
    assert len(state_m.state.states) == old_child_state_count + 2

    state = None
    for state in state_m.state.states.values():
        if state.name == "Dialog [3 options]":
            break
    assert state is not None
    new_template_state = state
    gui(new_template_state.add_scoped_variable, 'scoopy', float, 0.3)
    state_m_to_copy = sm_m.get_state_model_by_path('CDMJPK/' + new_template_state.state_id)

    ##########################################################
    print("copy & paste complex state into itself")

    copy_and_paste_state_into_itself(gui, sm_m, state_m_to_copy, page, menubar_ctrl)
    print("increase complexity by doing it twice -> increase the hierarchy-level")
    copy_and_paste_state_into_itself(gui, sm_m, state_m_to_copy, page, menubar_ctrl)

    ##########################################################
    # substitute state with template
    old_keys = list(state_m_parent.state.states.keys())
    transitions_before, data_flows_before = state_m_parent.state.get_connections_for_state('RQXPAI')

    # lib_state = gui(rafcon.gui.singleton.library_manager.get_library_instance, 'generic', 'wait')
    # CORE LEVEL VERSION OF A SUBSTITUTE STATE EXAMPLE
    # gui(state_m_parent.state.substitute_state, 'RQXPAI', lib_state.state_copy)
    # SAME WITH THE GUI AND HELPER METHOD
    # gui(gui_helper_state.substitute_state_as, state_m_parent.states['RQXPAI'], lib_state, True)
    # GUI LEVEL SUBSTITUTE STATE EXAMPLE WITH CORRECT META DATA HANDLING TODO why those above produce wrong meta data?
    gui(sm_m.selection.set, [state_m_parent.states['RQXPAI'], ])
    library_path, library_name = ('generic', 'wait')
    gui(lib_tree_ctrl.select_library_tree_element_of_lib_tree_path, join(library_path, library_name))
    gui(lib_tree_ctrl.substitute_as_template_clicked, None, True)
    gui.expected_warnings += 1  # The name of a temporarily generated LibraryState is changed

    new_state_id = None
    for state_id in state_m_parent.state.states.keys():
        if state_id not in old_keys:
            new_state_id = state_id
    transitions_after, data_flows_after = state_m_parent.state.get_connections_for_state(new_state_id)
    # transition is not preserved because of unequal outcome naming
    assert len(transitions_before['external']['ingoing']) == 1
    assert len(transitions_after['external']['ingoing']) == 1
    assert len(transitions_before['external']['outgoing']) == 1
    assert len(transitions_after['external']['outgoing']) == 0
    gui(state_m_parent.state.add_transition, new_state_id, 0, 'MCOLIQ', None)
    print("XXX1")
    # modify the template with other data type and respective data flows to parent
    gui(list(state_m_parent.states[new_state_id].state.input_data_ports.items())[0][1].__setattr__, "data_type", "int")
    gui(state_m_parent.state.add_input_data_port, 'in_time', "int")
    gui(state_m_parent.state.add_data_flow,
        state_m_parent.state.state_id,
        list(state_m_parent.state.input_data_ports.items())[0][1].data_port_id,
        new_state_id,
        list(state_m_parent.states[new_state_id].state.input_data_ports.items())[0][1].data_port_id)

    ##########################################################
    # substitute state with library
    old_keys = list(state_m_parent.state.states.keys())
    transitions_before, data_flows_before = state_m_parent.state.get_connections_for_state(new_state_id)

    # lib_state = gui(rafcon.gui.singleton.library_manager.get_library_instance, 'generic', 'wait')
    # CORE LEVEL VERSION OF A SUBSTITUTE STATE EXAMPLE
    # gui(state_m_parent.state.substitute_state, new_state_id, lib_state)
    # SAME WITH THE GUI AND HELPER METHOD
    # gui(gui_helper_state.substitute_state_as, state_m_parent.states[new_state_id], lib_state, False)
    # GUI LEVEL SUBSTITUTE STATE EXAMPLE WITH CORRECT META DATA HANDLING TODO why those above produce wrong meta data?
    gui(sm_m.selection.set, [state_m_parent.states[new_state_id], ])
    library_path, library_name = ('generic', 'wait')
    gui(lib_tree_ctrl.select_library_tree_element_of_lib_tree_path, join(library_path, library_name))
    gui(lib_tree_ctrl.substitute_as_library_clicked, None, True)

    new_state_id = None
    for state_id in list(state_m_parent.state.states.keys()):
        if state_id not in old_keys:
            new_state_id = state_id
    transitions_after, data_flows_after = state_m_parent.state.get_connections_for_state(new_state_id)
    # test if data flow is ignored
    assert len(transitions_before['external']['ingoing']) == 1
    assert len(transitions_after['external']['ingoing']) == 1
    assert len(transitions_before['external']['outgoing']) == 1
    assert len(transitions_after['external']['outgoing']) == 1
    assert len(data_flows_before['external']['ingoing']) == 1
    assert len(data_flows_after['external']['ingoing']) == 0

    # data flow is preserved if right data type and name is used
    gui(list(state_m_parent.state.input_data_ports.items())[0][1].__setattr__, "data_type", "float")
    if isinstance(state_m_parent.state.states[new_state_id], LibraryState):
        data_port_id = list(state_m_parent.state.states[new_state_id].input_data_ports.items())[0][0]
        state_m_parent.state.states[new_state_id].use_runtime_value_input_data_ports[data_port_id] = True
        state_m_parent.state.states[new_state_id].input_data_port_runtime_values[data_port_id] = 2.0
        print()
    else:
        raise
        # state_m_parent.state.states[new_state_id].input_data_ports.items()[0][1].default_value = 2.0
    gui(state_m_parent.state.add_data_flow,
        state_m_parent.state.state_id,
        list(state_m_parent.state.input_data_ports.items())[0][1].data_port_id,
        new_state_id,
        list(state_m_parent.states[new_state_id].state.input_data_ports.items())[0][1].data_port_id)

    old_keys = list(state_m_parent.state.states.keys())
    transitions_before, data_flows_before = state_m_parent.state.get_connections_for_state(new_state_id)
    # CORE LEVEL VERSION OF A SUBSTITUTE STATE EXAMPLE
    # lib_state = rafcon.gui.singleton.library_manager.get_library_instance('generic', 'wait')
    # gui(state_m_parent.state.substitute_state, new_state_id, lib_state.state_copy)
    # GUI LEVEL SUBSTITUTE STATE EXAMPLE WITH CORRECT META DATA HANDLING TODO why those above produce wrong meta data?
    gui(sm_m.selection.set, [state_m_parent.states[new_state_id], ])
    library_path, library_name = ('generic', 'wait')
    gui(lib_tree_ctrl.select_library_tree_element_of_lib_tree_path, join(library_path, library_name))
    gui(lib_tree_ctrl.substitute_as_template_clicked, None, True)
    gui.expected_warnings += 1  # The name of a temporarily generated LibraryState is changed
    new_state_id = None
    for state_id in list(state_m_parent.state.states.keys()):
        if state_id not in old_keys:
            new_state_id = state_id
    transitions_after, data_flows_after = state_m_parent.state.get_connections_for_state(new_state_id)
    # test if data flow is ignored
    assert len(transitions_before['external']['ingoing']) == 1
    assert len(transitions_after['external']['ingoing']) == 1
    assert len(transitions_before['external']['outgoing']) == 1
    assert len(transitions_after['external']['outgoing']) == 1
    assert len(data_flows_before['external']['ingoing']) == 1
    assert len(data_flows_after['external']['ingoing']) == 1
    assert list(state_m_parent.state.states[new_state_id].input_data_ports.items())[0][1].default_value == 2.0

    ##########################################################
    # open separately
    gui(sm_m.selection.set, [sm_m.get_state_model_by_path('CDMJPK'), ])
    lib_state = LibraryState(join("generic", "dialog"), "Dialog [3 options]", "0.1", "Dialog [3 options]")
    gui(gui_helper_state_machine.insert_state_into_selected_state, lib_state, False)

    lib_hash = lib_state.state_copy.mutable_hash()
    # assert lib_state.mutable_hash().hexdigest() == lib_hash.hexdigest()
    sm_m = sm_manager_model.state_machines[lib_state.get_state_machine().state_machine_id]
    gui(sm_m.selection.set, [sm_m.get_state_model_by_path(lib_state.get_path())])
    gui(gui_helper_state_machine.open_library_state_separately)
    sm_m = sm_manager_model.get_selected_state_machine_model()
    assert sm_m.root_state.state.mutable_hash().hexdigest() == lib_hash.hexdigest()
    ##########################################################

    if with_substitute_library:
        ##########################################################
        # check substitute library state as template -> keep name
        old_parent = lib_state.parent
        state_ids = list(old_parent.states.keys())
        gui(lib_state.__setattr__, 'name', 'DIALOG_X')
        gui(sm_manager_model.__setattr__, 'selected_state_machine_id',
            lib_state.get_state_machine().state_machine_id)
        gui(gui_helper_state_machine.substitute_selected_library_state_with_template, True)  # keep_name=True
        gui.expected_warnings += 1  # The name of a temporarily generated LibraryState is changed
        new_states = [state for state in list(old_parent.states.values()) if state.state_id not in state_ids]
        assert new_states and len(new_states) == 1 and new_states[0].name == 'DIALOG_X'
        ##########################################################

    if with_refresh:
        gui(menubar_ctrl.on_refresh_libraries_activate)
        gui(testing_utils.wait_for_gui)
        gui(menubar_ctrl.on_refresh_all_activate, None, None, True)
        gui(testing_utils.wait_for_gui)
        assert len(sm_manager_model.state_machines) == 2


if __name__ == '__main__':
    # test_gui(None)
    pytest.main(['-s', __file__])
