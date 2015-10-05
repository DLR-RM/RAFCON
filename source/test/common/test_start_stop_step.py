import time

# core elements
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.storage.storage import StateMachineStorage
from rafcon.statemachine.state_machine import StateMachine

# singleton elements
import rafcon.statemachine.singleton

# test environment elements
import test_utils
import pytest


def return_loop_state_machine():
    state1 = ExecutionState("MyFirstState", path=test_utils.TEST_SM_PATH, filename="loop_state1.py")
    state1.add_outcome("MyFirstOutcome", 3)

    state2 = ExecutionState("MySecondState", path=test_utils.TEST_SM_PATH, filename="loop_state2.py")
    state2.add_outcome("FirstOutcome", 3)

    state3 = HierarchyState("MyFirstHierarchyState")
    state3.add_state(state1)
    state3.add_state(state2)
    state3.set_start_state(state1.state_id)
    state3.add_transition(state1.state_id, 3, state2.state_id, None)
    state3.add_transition(state2.state_id, 3, state1.state_id, None)

    return StateMachine(state3)


def test_start_stop_pause_step(caplog):

    sm = return_loop_state_machine()
    rafcon.statemachine.singleton.global_variable_manager.set_variable("counter", 0)

    s = StateMachineStorage(test_utils.get_test_sm_path("stored_statemachine"))
    s.save_statemachine_as_yaml(sm, test_utils.get_test_sm_path("stored_statemachine"))
    sm_loaded, version, creation_time = s.load_statemachine_from_yaml()

    test_utils.test_multithrading_lock.acquire()
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(sm_loaded)
    rafcon.statemachine.singleton.state_machine_manager.active_state_machine_id = sm_loaded.state_machine_id
    rafcon.statemachine.singleton.state_machine_execution_engine.step_mode()

    for i in range(5):
        time.sleep(0.2)
        rafcon.statemachine.singleton.state_machine_execution_engine.step()

    # give the state machine time to execute
    time.sleep(0.2)
    rafcon.statemachine.singleton.state_machine_execution_engine.stop()
    sm_loaded.root_state.join()

    assert rafcon.statemachine.singleton.global_variable_manager.get_variable("counter") == 5
    rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(sm_loaded.state_machine_id)
    test_utils.assert_logger_warnings_and_errors(caplog)
    test_utils.test_multithrading_lock.release()

if __name__ == '__main__':
    pytest.main([__file__])