import variables_for_pytest
import rafcon.statemachine.singleton
from rafcon.statemachine.execution.statemachine_execution_engine import StatemachineExecutionEngine
import rafcon.statemachine.states.execution_state
from rafcon.statemachine.states.hierarchy_state import HierarchyState


def custom_entry_point():

    variables_for_pytest.test_multithrading_lock.acquire()

    start_state_id = "RWUZOP/ZDWBKU/HADSLI"
    sm = StatemachineExecutionEngine.execute_state_machine_from_path(
        "../test_scripts/unit_test_state_machines/test_custom_entry_point", start_state_id)
    rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(sm.state_machine_id)
    assert sm.root_state.output_data["error_check"] == "successfull"

    variables_for_pytest.test_multithrading_lock.release()


if __name__ == '__main__':
    custom_entry_point()