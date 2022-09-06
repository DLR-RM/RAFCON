from os.path import join

from rafcon.utils import log
# test environment elements
from tests import utils as testing_utils
from tests.utils import call_gui_callback, focus_graphical_editor_in_page

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


def trigger_ungroup_signals(gui):
    import rafcon.gui.helpers.state as gui_helper_state
    import rafcon.gui.helpers.state_machine as gui_helper_state_machine

    sm_manager_model = gui.singletons.state_machine_manager_model
    main_window_controller = gui.singletons.main_window_controller
    menubar_ctrl = main_window_controller.menu_bar_controller

    state_machine = create_state_machine()
    first_sm_id = state_machine.state_machine_id
    gui(gui.core_singletons.state_machine_manager.add_state_machine, state_machine)

    gui(main_window_controller.view['main_window'].grab_focus)
    gui(sm_manager_model.__setattr__, "selected_state_machine_id", first_sm_id)

    state_machines_ctrl = main_window_controller.state_machines_editor_ctrl
    page_id = state_machines_ctrl.get_page_num(first_sm_id)
    page = state_machines_ctrl.view.notebook.get_nth_page(page_id)
    gui(focus_graphical_editor_in_page, page)
    sm_m = sm_manager_model.get_selected_state_machine_model()
    assert sm_m
    gui(gui_helper_state.change_state_type, sm_m.get_state_model_by_path("ROOTSTATE/STATE3"),
                      gui_helper_state.BarrierConcurrencyState)

    gui(sm_m.selection.set, sm_m.get_state_model_by_path("ROOTSTATE/STATE3"))
    gui(gui_helper_state_machine.ungroup_selected_state)

    gui(menubar_ctrl.on_save_as_activate, None, None, testing_utils.get_unique_temp_path())
    gui(menubar_ctrl.on_stop_activate, None)


@pytest.mark.parametrize('gui', [{"libraries": {
    "ros": join(testing_utils.EXAMPLES_PATH, "libraries", "ros_libraries"),
    "turtle_libraries": join(testing_utils.EXAMPLES_PATH, "libraries", "turtle_libraries")
}}], indirect=True, ids=["with ros and turtle libraries"])
def test_ungroup(gui):
    trigger_ungroup_signals(gui)


@pytest.mark.parametrize('gui', [{"libraries": {
    "ros": join(testing_utils.EXAMPLES_PATH, "libraries", "ros_libraries"),
    "turtle_libraries": join(testing_utils.EXAMPLES_PATH, "libraries", "turtle_libraries")
}}], indirect=True, ids=["with ros and turtle libraries"])
def test_bug_issue_539(gui):

    import rafcon.gui.helpers.state_machine as gui_helper_state_machine
    sm_manager_model = gui.singletons.state_machine_manager_model
    main_window_controller = gui.singletons.main_window_controller
    menubar_ctrl = main_window_controller.menu_bar_controller

    current_sm_length = len(sm_manager_model.state_machines)
    assert current_sm_length == 0
    gui(menubar_ctrl.on_new_activate, None)
    new_state_machine_m = list(sm_manager_model.state_machines.values())[0]

    gui(menubar_ctrl.on_add_state_activate, None)

    new_state_m = list(new_state_machine_m.root_state.states.values())[0]
    gui(new_state_machine_m.selection.set, new_state_m)

    gui(menubar_ctrl.on_toggle_is_start_state_active, None)

    gui(new_state_machine_m.root_state.state.add_transition,
                      new_state_m.state.state_id, 0,
                      new_state_machine_m.root_state.state.state_id, 0)

    # make the example bit more complex
    gui(new_state_machine_m.selection.set, new_state_machine_m.root_state)
    gui(menubar_ctrl.on_add_state_activate, None)
    new_state_m_2 = list(new_state_machine_m.root_state.states.values())[0]
    if new_state_m_2 == new_state_m:
        new_state_m_2 = list(new_state_machine_m.root_state.states.values())[1]
    # -> add two transitions to rebuild
    gui(new_state_machine_m.root_state.state.add_transition,
                      new_state_m.state.state_id, -1,
                      new_state_m_2.state.state_id, None)
    gui(new_state_machine_m.root_state.state.add_transition,
                      new_state_m_2.state.state_id, 0,
                      new_state_machine_m.root_state.state.state_id, 0)

    # secure selection
    gui(new_state_machine_m.selection.set, [new_state_m, new_state_m_2])

    gui(gui_helper_state_machine.group_selected_states_and_scoped_variables)

    new_state_m_from_group_action = list(new_state_machine_m.root_state.states.values())[0]

    # TODO substitute these checks by creation of transitions and an in list check on respective hierarchy level
    # TODO (will be faster)
    # check if transitions are there by negative check and trying to create them
    print("check start transition in root state")
    with pytest.raises(ValueError):
        gui(new_state_machine_m.root_state.state.add_transition,
                          new_state_machine_m.root_state.state.state_id, None,
                          new_state_m_from_group_action.state.state_id, None)

    print("check start inside of grouped state")
    with pytest.raises(ValueError):
        gui(new_state_m_from_group_action.state.add_transition,
                          new_state_m_from_group_action.state.state_id, None,
                          new_state_m.state.state_id, None)

    print("check end inside of grouped state")
    with pytest.raises(ValueError):
        gui(new_state_m_from_group_action.state.add_transition,
                          new_state_m.state.state_id, 0,
                          new_state_m_from_group_action.state.state_id, 0)

    print("check end transition in root state - relevant transition for bug issue 539")
    with pytest.raises(ValueError):
        gui(new_state_machine_m.root_state.state.add_transition,
                          new_state_m_from_group_action.state.state_id, 0,
                          new_state_machine_m.root_state.state.state_id, 0)

    # the extra checks for more complex scenario
    print("check income transition for second state")
    with pytest.raises(ValueError):
        gui(new_state_m_from_group_action.state.add_transition,
                          new_state_m.state.state_id, -1,
                          new_state_m_2.state.state_id, 0)
    print("check outcome transition for second state")
    with pytest.raises(ValueError):
        gui(new_state_m_from_group_action.state.add_transition,
                          new_state_m_2.state.state_id, 0,
                          new_state_m_from_group_action.state.state_id, 0)

    print("finished run of trigger_issue_539_reproduction_sequence")


@pytest.mark.parametrize('gui', [{"gui_config": {'HISTORY_ENABLED': True}}], indirect=True, ids=["with history"])
def test_bug_issue_574(gui):
    from os.path import join
    sm_manager_model = gui.singletons.state_machine_manager_model
    main_window_controller = gui.singletons.main_window_controller
    menubar_ctrl = main_window_controller.menu_bar_controller

    gui(menubar_ctrl.on_open_activate, None, None, join(testing_utils.TEST_ASSETS_PATH,
                                                                      "unit_test_state_machines",
                                                                      "99_bottles_of_beer_no_wait"))
    sm_m = gui.singletons.state_machine_manager_model.state_machines[sm_manager_model.selected_state_machine_id]
    gui(sm_m.selection.set, list(sm_m.root_state.states.values()))
    gui(menubar_ctrl.on_group_states_activate, None, None)

    # TODO add check for warning if a data-flow is connected to scoped variables which has ingoing and outgoing
    # TODO data flows linked with selected states

    gui(sm_m.history.undo)

    gui(sm_m.selection.set, list(sm_m.root_state.states.values()) + list(sm_m.root_state.scoped_variables))
    gui(menubar_ctrl.on_group_states_activate, None, None)


@pytest.mark.timeout(60)
@pytest.mark.parametrize('gui', [{"gui_config": {'HISTORY_ENABLED': True}}], indirect=True, ids=["with history"])
def test_bug_issue_586(gui):
    from os.path import join
    sm_manager_model = gui.singletons.state_machine_manager_model
    main_window_controller = gui.singletons.main_window_controller
    menubar_ctrl = main_window_controller.menu_bar_controller
    current_sm_length = len(sm_manager_model.state_machines)
    assert current_sm_length == 0

    # backward step barrier test is chosen to work on an existing test state machine including equal child state ids
    gui(menubar_ctrl.on_open_activate, None, None, join(testing_utils.TEST_ASSETS_PATH,
                                                                      "unit_test_state_machines",
                                                                      "backward_step_barrier_test"))
    sm_m = list(sm_manager_model.state_machines.values())[0]
    assert sm_m.state_machine_id == sm_manager_model.selected_state_machine_id
    concurrent_decimate_state_m = sm_m.get_state_model_by_path("GLSUJY/OOECFM")

    # check start conditions overlapping ids -> all states have four states and those have the same ids
    import rafcon.core.constants
    state_ids = list(concurrent_decimate_state_m.states.keys())
    state_ids.remove(rafcon.core.constants.UNIQUE_DECIDER_STATE_ID)

    state_ids_set = set()
    for state_id in state_ids:
        for child_state_id in concurrent_decimate_state_m.states[state_id].states.keys():
            state_ids_set.add(child_state_id)
        assert len(state_ids_set) == 4

    gui(sm_m.selection.set, concurrent_decimate_state_m)
    gui(menubar_ctrl.on_ungroup_state_activate, None, None)
    testing_utils.wait_for_gui()

    # ungroup all three child states which all have the same state ids as there child states plus data flows
    for state_id in state_ids:
        print("ungroup state:", state_id)
        assert state_id in sm_m.root_state.states
        child_state_m = sm_m.get_state_model_by_path("GLSUJY/" + state_id)
        gui(sm_m.selection.set, child_state_m)
        gui(menubar_ctrl.on_ungroup_state_activate, None, None)

    # store and refresh selected
    gui(menubar_ctrl.on_save_as_activate, None, None, testing_utils.get_unique_temp_path())
    gui(menubar_ctrl.on_refresh_selected_activate, None, None)

    # # or an extra -> undo all
    # gui(sm_m.history.undo)
    #
    # gui(sm_m.history.undo)
    #
    # gui(sm_m.history.undo)
    #
    # gui(sm_m.history.undo)

if __name__ == '__main__':
    # test_ungroup(None)
    # test_bug_issue_539(None)
    # test_bug_issue_574(None)
    test_bug_issue_586(None)
    # pytest.main(['-s', __file__])
