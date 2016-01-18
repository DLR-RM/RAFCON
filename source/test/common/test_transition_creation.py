import time
import os

# core elements
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.storage.storage import StateMachineStorage
from rafcon.statemachine.state_machine import StateMachine

# singleton elements
import rafcon.statemachine.singleton

# test environment elements
from pytest import raises
import testing_utils
import pytest


def create_statemachine():
    state1 = ExecutionState("DummyState1", path=rafcon.__path__[0] + "/../test_scripts", filename="transition_test_state.py")
    state1.add_outcome('dummy_outcome_1', 3)
    state1.add_outcome('dummy_outcome_2', 4)

    state2 = ExecutionState("DummyState2", path=rafcon.__path__[0] + "/../test_scripts", filename="transition_test_state.py")
    state2.add_outcome('dummy_outcome_1', 3)
    state2.add_outcome('dummy_outcome_2', 4)

    state3 = ExecutionState("DummyState3", path=rafcon.__path__[0] + "/../test_scripts", filename="transition_test_state.py")
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

    storage_path = testing_utils.get_tmp_unit_test_path() + os.path.split(__file__)[0] + os.path.split(__file__)[1]
    test_storage = StateMachineStorage(storage_path)

    sm = create_statemachine()

    test_storage.save_statemachine_to_path(sm, storage_path)
    [sm_loaded, version, creation_time] = test_storage.load_statemachine_from_path()

    root_state = sm_loaded.root_state

    state_machine = StateMachine(root_state)
    assert testing_utils.test_multithrading_lock.acquire(False)
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    rafcon.statemachine.singleton.state_machine_execution_engine.start()
    rafcon.statemachine.singleton.state_machine_execution_engine.join()
    testing_utils.test_multithrading_lock.release()
    testing_utils.assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    pytest.main([__file__])
