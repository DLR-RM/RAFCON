"""Every test function of this module should later be improved in a separate module to secure all functionality of the 
every respective feature."""

# general tool elements
from rafcon.utils import log

# test environment elements
from tests import utils as testing_utils
import pytest

logger = log.get_logger(__name__)


def create_models(*args, **kargs):
    import rafcon.core.singleton
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.core.state_machine import StateMachine

    state1 = HierarchyState('State1', state_id="State1")
    state2 = ExecutionState('State2', state_id="State2")

    ctr_state = HierarchyState(name="Root", state_id="Root")
    ctr_state.add_state(state1)
    ctr_state.add_state(state2)
    ctr_state.start_state_id = state1.state_id
    ctr_state.add_transition(state1.state_id, from_outcome=0, to_state_id=state2.state_id, to_outcome=None)
    ctr_state.add_transition(state2.state_id, from_outcome=0, to_state_id=ctr_state.state_id, to_outcome=0)
    ctr_state.name = "Container"

    sm = StateMachine(ctr_state)

    # add new state machine
    rafcon.core.singleton.state_machine_manager.add_state_machine(sm)

# def test_state_type_change(caplog):
#     pass
#
#
# def test_substitute_state(caplog):
#     pass
#
#
# def test_group_states(caplog):
#     pass
#
#
# def test_ungroup_state(caplog):
#     pass


def set_state_name(state, new_value):
    state.name = new_value


def trigger_repetitive_group_ungroup(gui):
    import rafcon.gui.helpers.state as gui_helper_state
    import rafcon.gui.helpers.state_machine as gui_helper_state_machine
    from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState, UNIQUE_DECIDER_STATE_ID

    gui(create_models)
    gui(testing_utils.wait_for_gui)

    sm_m = gui.singletons.state_machine_manager_model.get_selected_state_machine_model()
    print("select: ", list(sm_m.root_state.states.values()))
    gui(sm_m.selection.set, list(sm_m.root_state.states.values()))

    gui(gui_helper_state_machine.group_selected_states_and_scoped_variables)
    # gui(sm_m.root_state.states.values()[0].state.name, "Stage 1")
    print("Stage 1", list(sm_m.root_state.states.values())[0].state.get_path())
    gui(set_state_name, list(sm_m.root_state.states.values())[0].state, "Stage 1")

    print("select: ", list(sm_m.root_state.states.values()))
    gui(sm_m.selection.set, list(list(sm_m.root_state.states.values())[0].states.values()))
    # time.sleep(1)
    gui(gui_helper_state_machine.group_selected_states_and_scoped_variables)
    print("Stage 2", list(list(sm_m.root_state.states.values())[0].states.values())[0].state.get_path())
    gui(set_state_name, list(list(sm_m.root_state.states.values())[0].states.values())[0].state, "Stage 2")

    print("select: ", list(sm_m.root_state.states.values()))
    gui(sm_m.selection.set, list(list(sm_m.root_state.states.values())[0].states.values())[0].states.values())
    # time.sleep(1)
    gui(gui_helper_state_machine.group_selected_states_and_scoped_variables)
    print("Stage 3", list(list(list(sm_m.root_state.states.values())[0].states.values())[0].states.values())[0].state.get_path())
    gui(
        set_state_name,
        list(list(list(sm_m.root_state.states.values())[0].states.values())[0].states.values())[0].state, "Stage 3"
    )
    # time.sleep(5)

    # raw_input("press enter")
    gui(sm_m.selection.set, list(list(sm_m.root_state.states.values())[0].states.values())[0])
    # time.sleep(1)
    s = list(list(sm_m.root_state.states.values())[0].states.values())[0]
    print("ungroup Stage 2", s.state.get_path(), s)
    gui(gui_helper_state_machine.ungroup_selected_state)
    gui(sm_m.selection.set, list(sm_m.root_state.states.values())[0])
    # time.sleep(1)
    gui(gui_helper_state_machine.ungroup_selected_state)

    gui(sm_m.selection.set, list(sm_m.root_state.states.values())[0])
    # time.sleep(1)
    gui(gui_helper_state_machine.ungroup_selected_state)

    print("make Stage 1 group all states of root state")
    gui(sm_m.selection.set, list(sm_m.root_state.states.values()))
    # time.sleep(1)
    gui(gui_helper_state_machine.group_selected_states_and_scoped_variables)
    gui(set_state_name, list(sm_m.root_state.states.values())[0].state, "Stage 1")

    print("make Stage 1 to barrier state")
    gui(sm_m.selection.set, list(sm_m.root_state.states.values())[0])
    gui(gui_helper_state.change_state_type, list(sm_m.root_state.states.values())[0], BarrierConcurrencyState)
    # time.sleep(1)

    # raw_input("enter")
    print('#'*30, '\n', '### positive example #1 \n', '#'*30)
    selected_states = [list(sm_m.root_state.states.values())[0].states[state_id] for state_id in ['State1', 'State2']]
    print("\n" * 50)
    print("select: ", [str(state_m) for state_m in selected_states])
    gui(sm_m.selection.set, selected_states)
    # time.sleep(1)
    gui(gui_helper_state_machine.group_selected_states_and_scoped_variables)

    # core test
    # gui(sm_m.root_state.states.values()[0].state.group_states, ['State1', 'State2'])
    # sm_m.root_state.states.values()[0].states.values()[0].state.name = "Stage 2"
    print("\n" * 50)
    # raw_input("enter")
    print("ungroup by undo again")
    # time.sleep(10)
    gui(sm_m.history.undo)
    print("wait2 undo")
    # import time
    # time.sleep(10)

    print('#'*30, '\n', '### negative example #1 by including a decider state-> exception test\n', '#'*30)
    selected_states = [list(sm_m.root_state.states.values())[0].states[state_id] for state_id in
                       ['State1', 'State2', UNIQUE_DECIDER_STATE_ID]]
    print("select: ", [str(state_m) for state_m in selected_states])
    gui(sm_m.selection.set, list(list(sm_m.root_state.states.values())[0].states.values()))
    gui(gui_helper_state_machine.group_selected_states_and_scoped_variables)

    print('#'*30, '\n', '### positive example #2 ungroup a barrier state \n', '#'*30)
    gui(sm_m.selection.set, list(sm_m.root_state.states.values())[0])
    gui(gui_helper_state_machine.ungroup_selected_state)

    # exception core test
    # gui(sm_m.root_state.states.values()[0].state.group_states, ['State1', 'State2', UNIQUE_DECIDER_STATE_ID])
    # print "wait3 failure"


@pytest.mark.parametrize('gui', [{"gui_config": {'HISTORY_ENABLED': True}}], indirect=True, ids=["with history"])
def test_repetitive_ungroup_state_and_group_states(gui):
    """Check if repetitive group and ungroup works"""
    trigger_repetitive_group_ungroup(gui)
    gui.expected_errors = 1


@pytest.mark.parametrize('gui', [{"gui_config": {'HISTORY_ENABLED': True}}], indirect=True, ids=["with history"])
def test_cut_of_multiple_states(gui):
    """Check if order of selection and deselection while a cut operation can create a problem with 
    the states editor generation and destruction
    
    - covers bug issue #631 cut multiple states fails
    """

    import rafcon.gui.singleton
    from rafcon.gui.controllers.menu_bar import MenuBarController
    from rafcon.gui.controllers.states_editor import StatesEditorController
    from rafcon.gui.clipboard import global_clipboard
    sm_manager_model = rafcon.gui.singleton.state_machine_manager_model
    main_window_controller = rafcon.gui.singleton.main_window_controller
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
    states_editor_ctrl = main_window_controller.get_controller('states_editor_ctrl')
    assert isinstance(menubar_ctrl, MenuBarController)
    assert isinstance(states_editor_ctrl, StatesEditorController)

    # generate new state machine -> it should be selected directly
    gui(menubar_ctrl.on_new_activate, None)
    sm_m = sm_manager_model.get_selected_state_machine_model()

    # add two states
    gui(menubar_ctrl.on_add_state_activate, None)
    gui(menubar_ctrl.on_add_state_activate, None)
    assert len(sm_m.root_state.states) == 2

    # select them step by step and check that only one state editor is created
    state_ids = list(sm_m.root_state.states.keys())
    print("states in root state are {0}".format(state_ids))
    gui(sm_m.selection.set, sm_m.root_state.states[state_ids[0]])
    gui(sm_m.selection.add, sm_m.root_state.states[state_ids[1]])
    print("selected states are {0}".format(sm_m.selection.states))
    assert len(states_editor_ctrl.tabs) == 1

    # cut and check that both states are removed
    gui(global_clipboard.cut, sm_m.selection)
    assert len(sm_m.root_state.states) == 0


# def test_paste_method(caplog):
#     """Check multiple Scenarios of paste methods"""
#     # meta data adjustments
#     # model assignments
#     pass

if __name__ == '__main__':
    testing_utils.dummy_gui(None)
    test_repetitive_ungroup_state_and_group_states(None)
    # test_cut_of_multiple_states(None)
    pytest.main([__file__, '-xs'])
