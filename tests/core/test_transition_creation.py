# core elements
import rafcon.core.singleton
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.storage import storage
from rafcon.core.state_machine import StateMachine

# test environment elements
from pytest import raises
import testing_utils
import pytest


def create_state_machine():
    state1 = ExecutionState("DummyState1", path=testing_utils.TEST_SCRIPT_PATH, filename="transition_test_state.py")
    state1.add_outcome('dummy_outcome_1', 3)
    state1.add_outcome('dummy_outcome_2', 4)

    state2 = ExecutionState("DummyState2", path=testing_utils.TEST_SCRIPT_PATH, filename="transition_test_state.py")
    state2.add_outcome('dummy_outcome_1', 3)
    state2.add_outcome('dummy_outcome_2', 4)

    state3 = ExecutionState("DummyState3", path=testing_utils.TEST_SCRIPT_PATH, filename="transition_test_state.py")
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
    state4.add_transition(state3.state_id, 4, state4.state_id, 5)

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

    state4.start_state_id = None
    state4.add_transition(None, None, state1.state_id, None)

    return StateMachine(state4)


def test_transition_creation(caplog):

    storage_path = testing_utils.get_unique_temp_path()

    sm = create_state_machine()

    storage.save_state_machine_to_path(sm, storage_path)
    sm_loaded = storage.load_state_machine_from_path(storage_path)

    root_state = sm_loaded.root_state

    state_machine = StateMachine(root_state)
    testing_utils.test_multithreading_lock.acquire()
    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    rafcon.core.singleton.state_machine_execution_engine.start()
    rafcon.core.singleton.state_machine_execution_engine.join()
    try:
        testing_utils.assert_logger_warnings_and_errors(caplog)
    finally:
        testing_utils.test_multithreading_lock.release()

    rafcon.core.singleton.state_machine_manager.delete_all_state_machines()


if __name__ == '__main__':
    pytest.main([__file__])
