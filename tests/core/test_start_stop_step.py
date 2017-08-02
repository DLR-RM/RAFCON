import time

# core elements
import rafcon.core.singleton
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.state_machine import StateMachine

# test environment elements
import testing_utils
import pytest


def return_loop_state_machine():
    state1 = ExecutionState("MyFirstState", path=testing_utils.TEST_SCRIPT_PATH, filename="loop_state1.py")
    state1.add_outcome("MyFirstOutcome", 3)

    state2 = ExecutionState("MySecondState", path=testing_utils.TEST_SCRIPT_PATH, filename="loop_state2.py")
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
    rafcon.core.singleton.global_variable_manager.set_variable("counter", 0)

    testing_utils.test_multithreading_lock.acquire()
    rafcon.core.singleton.state_machine_manager.add_state_machine(sm)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = sm.state_machine_id
    rafcon.core.singleton.state_machine_execution_engine.step_mode()

    for i in range(5):
        time.sleep(0.2)
        rafcon.core.singleton.state_machine_execution_engine.step_into()

    # give the state machine time to execute
    time.sleep(0.2)
    rafcon.core.singleton.state_machine_execution_engine.stop()
    rafcon.core.singleton.state_machine_execution_engine.join()

    try:
        assert rafcon.core.singleton.global_variable_manager.get_variable("counter") == 5
        rafcon.core.singleton.state_machine_manager.remove_state_machine(sm.state_machine_id)
        testing_utils.assert_logger_warnings_and_errors(caplog)
    finally:
        testing_utils.test_multithreading_lock.release()

if __name__ == '__main__':
    pytest.main([__file__])