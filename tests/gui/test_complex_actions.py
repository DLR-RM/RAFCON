"""Every test function of this module should later be improved in a separate module to secure all functionality of the 
every respective feature."""
import threading

# general tool elements
from rafcon.utils import log

# test environment elements
import testing_utils
from testing_utils import call_gui_callback
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
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = sm.state_machine_id

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


@log.log_exceptions(None, gtk_quit=True)
def trigger_repetitive_group_ungroup(*args):
    import rafcon.gui.helpers.state as gui_helper_state
    import rafcon.gui.helpers.state_machine as gui_helper_state_machine
    import rafcon.gui.singleton as gui_singletons
    from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState, UNIQUE_DECIDER_STATE_ID

    call_gui_callback(create_models)
    call_gui_callback(testing_utils.wait_for_gui)

    sm_m = gui_singletons.state_machine_manager_model.get_selected_state_machine_model()
    print "select: ", sm_m.root_state.states.values()
    call_gui_callback(sm_m.selection.set, sm_m.root_state.states.values())

    call_gui_callback(gui_helper_state_machine.group_selected_states_and_scoped_variables)
    # call_gui_callback(sm_m.root_state.states.values()[0].state.name, "Stage 1")
    print "Stage 1", sm_m.root_state.states.values()[0].state.get_path()
    call_gui_callback(set_state_name, sm_m.root_state.states.values()[0].state, "Stage 1")

    print "select: ", sm_m.root_state.states.values()
    call_gui_callback(sm_m.selection.set, sm_m.root_state.states.values()[0].states.values())
    # time.sleep(1)
    call_gui_callback(gui_helper_state_machine.group_selected_states_and_scoped_variables)
    print "Stage 2", sm_m.root_state.states.values()[0].states.values()[0].state.get_path()
    call_gui_callback(set_state_name, sm_m.root_state.states.values()[0].states.values()[0].state, "Stage 2")

    print "select: ", sm_m.root_state.states.values()
    call_gui_callback(sm_m.selection.set, sm_m.root_state.states.values()[0].states.values()[0].states.values())
    # time.sleep(1)
    call_gui_callback(gui_helper_state_machine.group_selected_states_and_scoped_variables)
    print "Stage 3", sm_m.root_state.states.values()[0].states.values()[0].states.values()[0].state.get_path()
    call_gui_callback(
        set_state_name, sm_m.root_state.states.values()[0].states.values()[0].states.values()[0].state, "Stage 3"
    )
    # time.sleep(5)

    # raw_input("press enter")
    call_gui_callback(sm_m.selection.set, sm_m.root_state.states.values()[0].states.values()[0])
    # time.sleep(1)
    s = sm_m.root_state.states.values()[0].states.values()[0]
    print "ungroup Stage 2", s.state.get_path(), s
    call_gui_callback(gui_helper_state_machine.ungroup_selected_state)
    call_gui_callback(sm_m.selection.set, sm_m.root_state.states.values()[0])
    # time.sleep(1)
    call_gui_callback(gui_helper_state_machine.ungroup_selected_state)

    call_gui_callback(sm_m.selection.set, sm_m.root_state.states.values()[0])
    # time.sleep(1)
    call_gui_callback(gui_helper_state_machine.ungroup_selected_state)

    print "make Stage 1 group all states of root state"
    call_gui_callback(sm_m.selection.set, sm_m.root_state.states.values())
    # time.sleep(1)
    call_gui_callback(gui_helper_state_machine.group_selected_states_and_scoped_variables)
    call_gui_callback(set_state_name, sm_m.root_state.states.values()[0].state, "Stage 1")

    print "make Stage 1 to barrier state"
    call_gui_callback(sm_m.selection.set, sm_m.root_state.states.values()[0])
    call_gui_callback(gui_helper_state.change_state_type, sm_m.root_state.states.values()[0], BarrierConcurrencyState)
    # time.sleep(1)

    # raw_input("enter")
    print '#'*30, '\n', '### positive example #1 \n', '#'*30
    selected_states = [sm_m.root_state.states.values()[0].states[state_id] for state_id in ['State1', 'State2']]
    print "\n" * 50
    print "select: ", [str(state_m) for state_m in selected_states]
    call_gui_callback(sm_m.selection.set, selected_states)
    # time.sleep(1)
    call_gui_callback(gui_helper_state_machine.group_selected_states_and_scoped_variables)

    # core test
    # call_gui_callback(sm_m.root_state.states.values()[0].state.group_states, ['State1', 'State2'])
    # sm_m.root_state.states.values()[0].states.values()[0].state.name = "Stage 2"
    print "\n" * 50
    # raw_input("enter")
    print "ungroup by undo again"
    # time.sleep(10)
    call_gui_callback(sm_m.history.undo)
    print "wait2 undo"
    # import time
    # time.sleep(10)

    print '#'*30, '\n', '### negative example #1 by including a decider state-> exception test\n', '#'*30
    selected_states = [sm_m.root_state.states.values()[0].states[state_id] for state_id in
                       ['State1', 'State2', UNIQUE_DECIDER_STATE_ID]]
    print "select: ", [str(state_m) for state_m in selected_states]
    call_gui_callback(sm_m.selection.set, sm_m.root_state.states.values()[0].states.values())
    call_gui_callback(gui_helper_state_machine.group_selected_states_and_scoped_variables)

    print '#'*30, '\n', '### positive example #2 ungroup a barrier state \n', '#'*30
    call_gui_callback(sm_m.selection.set, sm_m.root_state.states.values()[0])
    call_gui_callback(gui_helper_state_machine.ungroup_selected_state)

    # exception core test
    # call_gui_callback(sm_m.root_state.states.values()[0].state.group_states, ['State1', 'State2', UNIQUE_DECIDER_STATE_ID])
    # print "wait3 failure"


def test_repetitive_ungroup_state_and_group_states(caplog):
    """Check if repetitive group and ungroup works"""
    libraries = {"unit_test_state_machines": testing_utils.get_test_sm_path("unit_test_state_machines")}
    testing_utils.run_gui(gui_config={'HISTORY_ENABLED': True}, libraries=libraries)
    try:
        trigger_repetitive_group_ungroup()
    except Exception:
        raise
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=0, expected_errors=1)
    pass


# def test_paste_method(caplog):
#     """Check multiple Scenarios of paste methods"""
#     # meta data adjustments
#     # model assignments
#     pass

if __name__ == '__main__':
    testing_utils.dummy_gui(None)
    # test_repetitive_ungroup_state_and_group_states(None)
    pytest.main([__file__, '-xs'])
