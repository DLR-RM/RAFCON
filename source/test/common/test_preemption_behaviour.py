# core elements
from rafcon.statemachine.execution.statemachine_execution_engine import StatemachineExecutionEngine
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState
# import to make reload_config() working
import rafcon.mvc.singleton

# singleton elements
import rafcon.statemachine.singleton

# test environment elements
import test_utils
import pytest


def test_preemption_behaviour(caplog):
    test_utils.remove_all_libraries()

    test_utils.test_multithrading_lock.acquire()
    rafcon.statemachine.singleton.state_machine_manager.delete_all_state_machines()

    sm = StatemachineExecutionEngine.execute_state_machine_from_path(test_utils.get_test_sm_path("preemption_bahaviour_test_sm"))
    rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(sm.state_machine_id)
    from rafcon.statemachine.singleton import global_variable_manager
    assert global_variable_manager.get_variable("s2") == 1.0
    assert not global_variable_manager.variable_exist("s3")

    test_utils.reload_config()
    test_utils.test_multithrading_lock.release()
    test_utils.assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    test_preemption_behaviour(None)
    # pytest.main([__file__])