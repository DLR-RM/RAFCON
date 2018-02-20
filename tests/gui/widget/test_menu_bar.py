# general tool elements
from rafcon.utils import log

# test environment elements
import testing_utils
from testing_utils import call_gui_callback

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


def focus_graphical_editor_in_page(page):
    from rafcon.gui.views.graphical_editor import GraphicalEditor as OpenGLEditor
    from rafcon.gui.mygaphas.view import ExtendedGtkView as GaphasEditor
    graphical_controller = page.children()[0]
    if not isinstance(graphical_controller, (OpenGLEditor, GaphasEditor)):
        graphical_controller = graphical_controller.children()[0]
    graphical_controller.grab_focus()


def select_and_paste_state(state_machine_model, source_state_model, target_state_model, menu_bar_ctrl, operation,
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
    print "\n\n %s \n\n" % source_state_model.state.name
    call_gui_callback(state_machine_model.selection.set, [source_state_model])
    call_gui_callback(getattr(menu_bar_ctrl, 'on_{}_selection_activate'.format(operation)), None, None)
    print "\n\n %s \n\n" % target_state_model.state.name
    call_gui_callback(state_machine_model.selection.set, [target_state_model])
    old_child_state_count = len(target_state_model.state.states)
    main_window_controller.view['main_window'].grab_focus()
    focus_graphical_editor_in_page(page)
    call_gui_callback(menu_bar_ctrl.on_paste_clipboard_activate, None, None)
    testing_utils.wait_for_gui()
    print target_state_model.state.states.keys()
    assert len(target_state_model.state.states) == old_child_state_count + 1
    return target_state_model, old_child_state_count


def copy_and_paste_state_into_itself(sm_m, state_m_to_copy, page, menu_bar_ctrl):
    call_gui_callback(sm_m.selection.set, [state_m_to_copy])
    focus_graphical_editor_in_page(page)
    call_gui_callback(menu_bar_ctrl.on_copy_selection_activate, None, None)
    old_child_state_count = len(state_m_to_copy.state.states)
    call_gui_callback(sm_m.selection.set, [state_m_to_copy])
    focus_graphical_editor_in_page(page)
    call_gui_callback(menu_bar_ctrl.on_paste_clipboard_activate, None, None)
    assert len(state_m_to_copy.state.states) == old_child_state_count + 1


@log.log_exceptions(None, gtk_quit=True)
def trigger_gui_signals(with_refresh=True):
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
    from os.path import join
    from rafcon.core.states.library_state import LibraryState
    import rafcon.core.singleton
    import rafcon.gui.singleton
    import rafcon.gui.helpers.state as gui_helper_state
    import rafcon.gui.helpers.state_machine as gui_helper_state_machine
    sm_manager_model = rafcon.gui.singleton.state_machine_manager_model
    main_window_controller = rafcon.gui.singleton.main_window_controller
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')

    state_machine = create_state_machine()
    first_sm_id = state_machine.state_machine_id
    call_gui_callback(rafcon.core.singleton.state_machine_manager.add_state_machine, state_machine)
    call_gui_callback(rafcon.core.singleton.state_machine_manager.__setattr__, "active_state_machine_id", first_sm_id)

    current_sm_length = len(sm_manager_model.state_machines)
    call_gui_callback(menubar_ctrl.on_new_activate, None)

    assert len(sm_manager_model.state_machines) == current_sm_length + 1
    call_gui_callback(menubar_ctrl.on_open_activate, None, None, join(testing_utils.TUTORIAL_PATH,
                                                                      "basic_turtle_demo_sm"))
    call_gui_callback(testing_utils.wait_for_gui)
    assert len(sm_manager_model.state_machines) == current_sm_length + 2

    sm_m = sm_manager_model.state_machines[first_sm_id + 2]
    testing_utils.wait_for_gui()
    # MAIN_WINDOW NEEDS TO BE FOCUSED (for global input focus) TO OPERATE PASTE IN GRAPHICAL VIEWER
    main_window_controller.view['main_window'].grab_focus()
    call_gui_callback(sm_manager_model.__setattr__, "selected_state_machine_id", first_sm_id + 2)
    state_machines_ctrl = main_window_controller.get_controller('state_machines_editor_ctrl')
    page_id = state_machines_ctrl.get_page_num(first_sm_id + 2)
    page = state_machines_ctrl.view.notebook.get_nth_page(page_id)
    call_gui_callback(focus_graphical_editor_in_page, page)

    # TODO keep core interface, too
    # ##########################################################
    # # group states
    # # TODO improve test to related data flows
    # state_m_parent = sm_m.get_state_model_by_path('CDMJPK/RMKGEW/KYENSZ')
    # state_ids_old = [state_id for state_id in state_m_parent.state.states]
    # call_gui_callback(state_m_parent.state.group_states, ['PAYECU', 'UEPNNW', 'KQDJYS'])
    #
    # ##########################################################
    # # ungroup new state
    # state_new = None
    # for state_id in state_m_parent.state.states:
    #     if state_id not in state_ids_old:
    #         state_new = state_m_parent.state.states[state_id]
    # call_gui_callback(state_m_parent.state.ungroup_state, state_new.state_id)

    ##########################################################
    # group states
    # TODO improve test to related data flows
    print "#"*30, "\n", '#### group states \n', "#"*30, "\n"
    state_m_parent = sm_m.get_state_model_by_path('CDMJPK/RMKGEW/KYENSZ')
    state_ids_old = [state_id for state_id in state_m_parent.state.states]
    state_m_list = [state_m_parent.states[child_state_id] for child_state_id in ['PAYECU', 'UEPNNW', 'KQDJYS']]
    call_gui_callback(gui_helper_state.group_states_and_scoped_variables, state_m_list, [])

    ##########################################################
    # ungroup new state
    print "#"*30, "\n", '#### ungroup state \n', "#"*30, "\n"
    new_state = None
    for state_id in state_m_parent.state.states:
        if state_id not in state_ids_old:
            new_state = state_m_parent.state.states[state_id]
    call_gui_callback(gui_helper_state.ungroup_state, sm_m.get_state_model_by_path(new_state.get_path()))

    #########################################################
    print "select & copy an execution state -> and paste it somewhere"
    select_and_paste_state(sm_m, sm_m.get_state_model_by_path('CDMJPK/RMKGEW/KYENSZ'), sm_m.get_state_model_by_path(
        'CDMJPK/RMKGEW'), menubar_ctrl, 'copy', main_window_controller, page)

    ###########################################################
    print "select & copy a hierarchy state -> and paste it some where"
    select_and_paste_state(sm_m, sm_m.get_state_model_by_path('CDMJPK/RMKGEW/KYENSZ/VCWTIY'),
                           sm_m.get_state_model_by_path('CDMJPK'), menubar_ctrl, 'copy', main_window_controller, page)

    ##########################################################
    print "select a library state -> and paste it some where WITH CUT !!!"
    state_m, old_child_state_count = select_and_paste_state(sm_m,
                                                            sm_m.get_state_model_by_path('CDMJPK/RMKGEW/KYENSZ/VCWTIY'),
                                                            sm_m.get_state_model_by_path('CDMJPK'), menubar_ctrl, 'cut',
                                                            main_window_controller, page)

    ##########################################################
    # create complex state with all elements
    call_gui_callback(sm_m.selection.set, [sm_m.get_state_model_by_path('CDMJPK'), ])
    lib_state = LibraryState(join("generic", "dialog"), "Dialog [3 options]", "0.1", "Dialog [3 options]")
    call_gui_callback(gui_helper_state_machine.insert_state_into_selected_state, lib_state, True)
    assert len(state_m.state.states) == old_child_state_count + 2

    state = None
    for state in state_m.state.states.values():
        if state.name == "Dialog [3 options]":
            break
    assert state is not None
    new_template_state = state
    call_gui_callback(new_template_state.add_scoped_variable, 'scoopy', float, 0.3)
    state_m_to_copy = sm_m.get_state_model_by_path('CDMJPK/' + new_template_state.state_id)

    ##########################################################
    print "copy & paste complex state into itself"

    copy_and_paste_state_into_itself(sm_m, state_m_to_copy, page, menubar_ctrl)
    print "increase complexity by doing it twice -> increase the hierarchy-level"
    copy_and_paste_state_into_itself(sm_m, state_m_to_copy, page, menubar_ctrl)

    ##########################################################
    # substitute state with template
    lib_state = rafcon.gui.singleton.library_manager.get_library_instance('generic', 'wait')
    old_keys = state_m_parent.state.states.keys()
    transitions_before, data_flows_before = state_m_parent.state.related_linkage_state('RQXPAI')
    call_gui_callback(state_m_parent.state.substitute_state, 'RQXPAI', lib_state.state_copy)
    new_state_id = None
    for state_id in state_m_parent.state.states.keys():
        if state_id not in old_keys:
            new_state_id = state_id
    transitions_after, data_flows_after = state_m_parent.state.related_linkage_state(new_state_id)
    # transition is not preserved because of unequal outcome naming
    assert len(transitions_before['external']['ingoing']) == 1
    assert len(transitions_after['external']['ingoing']) == 1
    assert len(transitions_before['external']['outgoing']) == 1
    assert len(transitions_after['external']['outgoing']) == 0
    call_gui_callback(state_m_parent.state.add_transition, new_state_id, 0, 'MCOLIQ', None)

    # modify the template with other data type and respective data flows to parent
    call_gui_callback(state_m_parent.states[new_state_id].state.input_data_ports.items()[0][1].__setattr__, "data_type", "int")
    call_gui_callback(state_m_parent.state.add_input_data_port, 'in_time', "int")
    call_gui_callback(state_m_parent.state.add_data_flow,
                      state_m_parent.state.state_id,
                      state_m_parent.state.input_data_ports.items()[0][1].data_port_id,
                      new_state_id,
                      state_m_parent.states[new_state_id].state.input_data_ports.items()[0][1].data_port_id)

    old_keys = state_m_parent.state.states.keys()
    transitions_before, data_flows_before = state_m_parent.state.related_linkage_state(new_state_id)
    lib_state = rafcon.gui.singleton.library_manager.get_library_instance('generic', 'wait')
    call_gui_callback(state_m_parent.state.substitute_state, new_state_id, lib_state)
    new_state_id = None
    for state_id in state_m_parent.state.states.keys():
        if state_id not in old_keys:
            new_state_id = state_id
    transitions_after, data_flows_after = state_m_parent.state.related_linkage_state(new_state_id)
    # test if data flow is ignored
    assert len(transitions_before['external']['ingoing']) == 1
    assert len(transitions_after['external']['ingoing']) == 1
    assert len(transitions_before['external']['outgoing']) == 1
    assert len(transitions_after['external']['outgoing']) == 1
    assert len(data_flows_before['external']['ingoing']) == 1
    assert len(data_flows_after['external']['ingoing']) == 0

    # data flow is preserved if right data type and name is used
    call_gui_callback(state_m_parent.state.input_data_ports.items()[0][1].__setattr__, "data_type", "float")
    if isinstance(state_m_parent.state.states[new_state_id], LibraryState):
        data_port_id = state_m_parent.state.states[new_state_id].input_data_ports.items()[0][0]
        state_m_parent.state.states[new_state_id].use_runtime_value_input_data_ports[data_port_id] = True
        state_m_parent.state.states[new_state_id].input_data_port_runtime_values[data_port_id] = 2.0
        print
    else:
        raise
        # state_m_parent.state.states[new_state_id].input_data_ports.items()[0][1].default_value = 2.0
    call_gui_callback(state_m_parent.state.add_data_flow,
                      state_m_parent.state.state_id,
                      state_m_parent.state.input_data_ports.items()[0][1].data_port_id,
                      new_state_id,
                      state_m_parent.states[new_state_id].state.input_data_ports.items()[0][1].data_port_id)

    old_keys = state_m_parent.state.states.keys()
    transitions_before, data_flows_before = state_m_parent.state.related_linkage_state(new_state_id)
    lib_state = rafcon.gui.singleton.library_manager.get_library_instance('generic', 'wait')
    call_gui_callback(state_m_parent.state.substitute_state, new_state_id, lib_state.state_copy)
    new_state_id = None
    for state_id in state_m_parent.state.states.keys():
        if state_id not in old_keys:
            new_state_id = state_id
    transitions_after, data_flows_after = state_m_parent.state.related_linkage_state(new_state_id)
    # test if data flow is ignored
    assert len(transitions_before['external']['ingoing']) == 1
    assert len(transitions_after['external']['ingoing']) == 1
    assert len(transitions_before['external']['outgoing']) == 1
    assert len(transitions_after['external']['outgoing']) == 1
    assert len(data_flows_before['external']['ingoing']) == 1
    assert len(data_flows_after['external']['ingoing']) == 1
    assert state_m_parent.state.states[new_state_id].input_data_ports.items()[0][1].default_value == 2.0

    if with_refresh:
        call_gui_callback(menubar_ctrl.on_refresh_libraries_activate)
        call_gui_callback(testing_utils.wait_for_gui)
        call_gui_callback(menubar_ctrl.on_refresh_all_activate, None, None, True)
        call_gui_callback(testing_utils.wait_for_gui)
        assert len(sm_manager_model.state_machines) == 1


def test_gui(caplog):
    from os.path import join

    change_in_gui_config = {'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': False}

    libraries = {"ros": join(testing_utils.EXAMPLES_PATH, "libraries", "ros_libraries"),
                 "turtle_libraries": join(testing_utils.EXAMPLES_PATH, "libraries", "turtle_libraries"),
                 "generic": join(testing_utils.LIBRARY_SM_PATH, "generic")}
    testing_utils.run_gui(gui_config=change_in_gui_config, libraries=libraries)

    try:
        trigger_gui_signals()
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=0, expected_errors=0)


if __name__ == '__main__':
    # test_gui(None)
    pytest.main(['-s', __file__])
