
from os.path import join

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

    state_machine = StateMachine(ctr_state)
    return state_machine


def focus_graphical_editor_in_page(page):
    from rafcon.gui.views.graphical_editor import GraphicalEditor as OpenGLEditor
    from rafcon.gui.mygaphas.view import ExtendedGtkView as GaphasEditor
    graphical_controller = page.children()[0]
    if not isinstance(graphical_controller, (OpenGLEditor, GaphasEditor)):
        graphical_controller = graphical_controller.children()[0]
    graphical_controller.grab_focus()


def trigger_ungroup_signals():
    import rafcon.core.singleton
    import rafcon.gui.singleton as gui_singleton
    import rafcon.gui.helpers.state as gui_helper_state
    import rafcon.gui.helpers.state_machine as gui_helper_state_machine

    sm_manager_model = rafcon.gui.singleton.state_machine_manager_model
    main_window_controller = gui_singleton.main_window_controller
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')

    state_machine = create_state_machine()
    first_sm_id = state_machine.state_machine_id
    call_gui_callback(rafcon.core.singleton.state_machine_manager.add_state_machine, state_machine)
    call_gui_callback(rafcon.core.singleton.state_machine_manager.__setattr__, "active_state_machine_id", first_sm_id)

    call_gui_callback(main_window_controller.view['main_window'].grab_focus)
    call_gui_callback(sm_manager_model.__setattr__, "selected_state_machine_id", first_sm_id)

    state_machines_ctrl = main_window_controller.get_controller('state_machines_editor_ctrl')
    page_id = state_machines_ctrl.get_page_num(first_sm_id)
    page = state_machines_ctrl.view.notebook.get_nth_page(page_id)
    call_gui_callback(focus_graphical_editor_in_page, page)
    sm_m = sm_manager_model.get_selected_state_machine_model()
    assert sm_m
    call_gui_callback(gui_helper_state.change_state_type, sm_m.get_state_model_by_path("ROOTSTATE/STATE3"),
                      gui_helper_state.BarrierConcurrencyState)

    call_gui_callback(sm_m.selection.set, sm_m.get_state_model_by_path("ROOTSTATE/STATE3"))
    call_gui_callback(gui_helper_state_machine.ungroup_selected_state)

    call_gui_callback(menubar_ctrl.on_save_as_activate, None, None, testing_utils.get_unique_temp_path())
    call_gui_callback(menubar_ctrl.on_stop_activate, None)


def test_ungroup(caplog):
    testing_utils.run_gui(
        gui_config={'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': False},
        libraries={"ros": join(testing_utils.EXAMPLES_PATH, "libraries", "ros_libraries"),
                   "turtle_libraries": join(testing_utils.EXAMPLES_PATH, "libraries", "turtle_libraries"),
                   "generic": join(testing_utils.LIBRARY_SM_PATH, "generic")}
    )

    try:
        trigger_ungroup_signals()
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog)


def trigger_issue_539_reproduction_sequence():
    import rafcon.gui.singleton
    import rafcon.gui.helpers.state_machine as gui_helper_state_machine
    sm_manager_model = rafcon.gui.singleton.state_machine_manager_model
    main_window_controller = rafcon.gui.singleton.main_window_controller
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')

    current_sm_length = len(sm_manager_model.state_machines)
    assert current_sm_length == 0
    call_gui_callback(menubar_ctrl.on_new_activate, None)
    new_state_machine_m = sm_manager_model.state_machines.values()[0]

    call_gui_callback(menubar_ctrl.on_add_state_activate, None)

    new_state_m = new_state_machine_m.root_state.states.values()[0]
    call_gui_callback(new_state_machine_m.selection.set, new_state_m)

    call_gui_callback(menubar_ctrl.on_toggle_is_start_state_active, None)

    call_gui_callback(new_state_machine_m.root_state.state.add_transition,
                      new_state_m.state.state_id, 0,
                      new_state_machine_m.root_state.state.state_id, 0)

    # make the example bit more complex
    call_gui_callback(new_state_machine_m.selection.set, new_state_machine_m.root_state)
    call_gui_callback(menubar_ctrl.on_add_state_activate, None)
    new_state_m_2 = new_state_machine_m.root_state.states.values()[0]
    if new_state_m_2 == new_state_m:
        new_state_m_2 = new_state_machine_m.root_state.states.values()[1]
    # -> add two transitions to rebuild
    call_gui_callback(new_state_machine_m.root_state.state.add_transition,
                      new_state_m.state.state_id, -1,
                      new_state_m_2.state.state_id, None)
    call_gui_callback(new_state_machine_m.root_state.state.add_transition,
                      new_state_m_2.state.state_id, 0,
                      new_state_machine_m.root_state.state.state_id, 0)

    # secure selection
    call_gui_callback(new_state_machine_m.selection.set, [new_state_m, new_state_m_2])

    call_gui_callback(gui_helper_state_machine.group_selected_states_and_scoped_variables)

    new_state_m_from_group_action = new_state_machine_m.root_state.states.values()[0]

    # TODO substitute these checks by creation of transitions and an in list check on respective hierarchy level
    # TODO (will be faster)
    # check if transitions are there by negative check and trying to create them
    print "check start transition in root state"
    with pytest.raises(ValueError):
        call_gui_callback(new_state_machine_m.root_state.state.add_transition,
                          new_state_machine_m.root_state.state.state_id, None,
                          new_state_m_from_group_action.state.state_id, None)

    print "check start inside of grouped state"
    with pytest.raises(ValueError):
        call_gui_callback(new_state_m_from_group_action.state.add_transition,
                          new_state_m_from_group_action.state.state_id, None,
                          new_state_m.state.state_id, None)

    print "check end inside of grouped state"
    with pytest.raises(ValueError):
        call_gui_callback(new_state_m_from_group_action.state.add_transition,
                          new_state_m.state.state_id, 0,
                          new_state_m_from_group_action.state.state_id, 0)

    print "check end transition in root state - relevant transition for bug issue 539"
    with pytest.raises(ValueError):
        call_gui_callback(new_state_machine_m.root_state.state.add_transition,
                          new_state_m_from_group_action.state.state_id, 0,
                          new_state_machine_m.root_state.state.state_id, 0)

    # the extra checks for more complex scenario
    print "check income transition for second state"
    with pytest.raises(ValueError):
        call_gui_callback(new_state_m_from_group_action.state.add_transition,
                          new_state_m.state.state_id, -1,
                          new_state_m_2.state.state_id, 0)
    print "check outcome transition for second state"
    with pytest.raises(ValueError):
        call_gui_callback(new_state_m_from_group_action.state.add_transition,
                          new_state_m_2.state.state_id, 0,
                          new_state_m_from_group_action.state.state_id, 0)

    print "finished run of trigger_issue_539_reproduction_sequence"


def test_bug_issue_539(caplog):
    testing_utils.run_gui(
        gui_config={'AUTO_BACKUP_ENABLED': False, 'HISTORY_ENABLED': False},
        libraries={"ros": join(testing_utils.EXAMPLES_PATH, "libraries", "ros_libraries"),
                   "turtle_libraries": join(testing_utils.EXAMPLES_PATH, "libraries", "turtle_libraries"),
                   "generic": join(testing_utils.LIBRARY_SM_PATH, "generic")}
    )

    try:
        trigger_issue_539_reproduction_sequence()
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog)

if __name__ == '__main__':
    # test_ungroup(None)
    test_bug_issue_539(None)
    # pytest.main(['-s', __file__])
