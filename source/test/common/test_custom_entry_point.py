# core elements
from rafcon.statemachine.execution.state_machine_execution_engine import StateMachineExecutionEngine
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState

# singleton elements
import rafcon.statemachine.singleton

# test environment elements
import testing_utils
import pytest


def test_custom_entry_point(caplog):
    testing_utils.remove_all_libraries()

    rafcon.statemachine.singleton.state_machine_manager.delete_all_state_machines()
    testing_utils.test_multithreading_lock.acquire()

    start_state_id = "RWUZOP/ZDWBKU/HADSLI"
    sm = rafcon.statemachine.singleton.state_machine_execution_engine.execute_state_machine_from_path(
        path=testing_utils.get_test_sm_path("unit_test_state_machines/test_custom_entry_point"),
        start_state_path=start_state_id)
    rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(sm.state_machine_id)
    assert not rafcon.statemachine.singleton.global_variable_manager.variable_exist("start_id21")

    testing_utils.test_multithreading_lock.release()
    testing_utils.assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    test_custom_entry_point(None)
    # pytest.main([__file__])
