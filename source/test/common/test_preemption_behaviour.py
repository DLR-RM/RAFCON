# core elements
from rafcon.statemachine.execution.statemachine_execution_engine import StatemachineExecutionEngine
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState

# singleton elements
import rafcon.statemachine.singleton

# test environment elements
import utils


def test_preemption_behaviour():

    rafcon.statemachine.singleton.state_machine_manager.delete_all_state_machines()
    utils.test_multithrading_lock.acquire()

    sm = StatemachineExecutionEngine.execute_state_machine_from_path(rafcon.__path__[0] + "/../test_scripts/preemption_bahaviour_test_sm")
    rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(sm.state_machine_id)
    from rafcon.statemachine.singleton import global_variable_manager
    assert global_variable_manager.get_variable("s2") == 1.0
    assert global_variable_manager.variable_exist("s3") == False

    utils.test_multithrading_lock.release()


if __name__ == '__main__':
    test_preemption_behaviour()