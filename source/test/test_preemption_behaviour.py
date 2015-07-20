import variables_for_pytest
import awesome_tool.statemachine.singleton
from awesome_tool.statemachine.execution.statemachine_execution_engine import StatemachineExecutionEngine
import awesome_tool.statemachine.states.execution_state
from awesome_tool.statemachine.states.hierarchy_state import HierarchyState
from awesome_tool.statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState


def test_preemption_behaviour():

    variables_for_pytest.test_multithrading_lock.acquire()

    sm = StatemachineExecutionEngine.execute_state_machine_from_path("../test_scripts/preemption_bahaviour_test_sm")
    awesome_tool.statemachine.singleton.state_machine_manager.remove_state_machine(sm.state_machine_id)
    from awesome_tool.statemachine.singleton import global_variable_manager
    assert global_variable_manager.get_variable("s2") == 1.0
    assert global_variable_manager.variable_exist("s3") == False

    variables_for_pytest.test_multithrading_lock.release()


if __name__ == '__main__':
    test_preemption_behaviour()