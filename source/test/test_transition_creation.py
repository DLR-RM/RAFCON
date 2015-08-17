import pytest
import time
from pytest import raises
from awesome_tool.statemachine.states.execution_state import ExecutionState
from awesome_tool.statemachine.states.hierarchy_state import HierarchyState
from awesome_tool.statemachine.storage.storage import StateMachineStorage
import awesome_tool.statemachine.singleton
from awesome_tool.statemachine.state_machine import StateMachine
import variables_for_pytest


def create_statemachine():
    state1 = ExecutionState("DummyState1", path="../test_scripts", filename="transition_test_state.py")
    state1.add_outcome('dummy_outcome_1', 3)
    state1.add_outcome('dummy_outcome_2', 4)

    state2 = ExecutionState("DummyState2", path="../test_scripts", filename="transition_test_state.py")
    state2.add_outcome('dummy_outcome_1', 3)
    state2.add_outcome('dummy_outcome_2', 4)

    state3 = ExecutionState("DummyState3", path="../test_scripts", filename="transition_test_state.py")
    state3.add_outcome('dummy_outcome_1', 3)
    state3.add_outcome('dummy_outcome_2', 4)

    state4 = HierarchyState("DummyHierarchyState")
    state4.add_state(state1)
    state4.set_start_state(state1.state_id)
    state4.add_state(state2)
    state4.add_state(state3)
    state4.add_outcome("final_outcome", 5)

    state4.add_transition(state1.state_id, 3, state2.state_id, None)
    state4.add_transition(state1.state_id, 4, state3.state_id, None)
    state4.add_transition(state2.state_id, 3, state4.state_id, 5)
    state4.add_transition(state3.state_id, 3, state4.state_id, 5)

    t = state4.add_transition(state2.state_id, 4, state1.state_id, None)

    state4.remove_transition(t)
    state4.add_transition(state2.state_id, 4, state1.state_id, None)

    # no target at all
    with raises(ValueError):
        state4.add_transition(state3.state_id, 4, None, None)

    # no to_state
    with raises(ValueError):
        state4.add_transition(state3.state_id, 4, None, 5)

    # start transition already existing
    with raises(ValueError):
        state4.add_transition(None, None, state3.state_id, None)

    state4.add_transition(state3.state_id, 4, state4.state_id, 5)

    state4.start_state_id = None
    state4.add_transition(None, None, state1.state_id, None)

    return StateMachine(state4)


def test_transition_creation():

    test_storage = StateMachineStorage("../test_scripts/stored_statemachine")

    sm = create_statemachine()

    test_storage.save_statemachine_as_yaml(sm, "../test_scripts/stored_statemachine")
    [sm_loaded, version, creation_time] = test_storage.load_statemachine_from_yaml()

    root_state = sm_loaded.root_state

    state_machine = StateMachine(root_state)
    assert variables_for_pytest.test_multithrading_lock.acquire(False)
    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    awesome_tool.statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    awesome_tool.statemachine.singleton.state_machine_execution_engine.start()
    time.sleep(0.2)
    root_state.join()
    time.sleep(0.2)
    awesome_tool.statemachine.singleton.state_machine_execution_engine.stop()
    variables_for_pytest.test_multithrading_lock.release()


if __name__ == '__main__':
    #pytest.main()
    test_transition_creation()
