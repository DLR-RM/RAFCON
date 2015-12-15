# core elements
from rafcon.statemachine.execution.statemachine_execution_engine import StatemachineExecutionEngine
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState

# singleton elements
from rafcon.statemachine.singleton import state_machine_manager

# test environment elements
import test_utils
import pytest


def test_error_propagation(caplog):
    test_utils.remove_all_libraries()

    state_machine_manager.delete_all_state_machines()
    test_utils.test_multithrading_lock.acquire()

    sm = StatemachineExecutionEngine.execute_state_machine_from_path(
        test_utils.get_test_sm_path("unit_test_state_machines/error_propagation_test"))
    state_machine_manager.remove_state_machine(sm.state_machine_id)
    assert sm.root_state.output_data["error_check"] == "successfull"

    test_utils.reload_config()
    test_utils.assert_logger_warnings_and_errors(caplog, 0, 2)
    test_utils.test_multithrading_lock.release()


if __name__ == '__main__':
    # test_error_propagation(None)
    pytest.main([__file__])
