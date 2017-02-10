import pytest

# state machine
import rafcon.core.singleton
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.state_machine import StateMachine

# test environment elements
import testing_utils


def create_state_machine():
    state1 = ExecutionState("MyFirstState", path=testing_utils.TEST_SCRIPT_PATH, filename="global_variable_state.py")
    state1.add_outcome("first_outcome", 3)
    input_state1 = state1.add_input_data_port("input_data_port1", "float")
    output_state1 = state1.add_output_data_port("output_data_port1", "float")

    state3 = HierarchyState("MyFirstHierarchyState")
    state3.add_state(state1)
    state3.set_start_state(state1.state_id)
    state3.add_outcome("Container_Outcome", 6)
    state3.add_transition(state1.state_id, 3, state3.state_id, 6)
    input_state3 = state3.add_input_data_port("input_data_port1", "float", 22.0)
    output_state3 = state3.add_output_data_port("output_data_port1", "float")
    state3.add_data_flow(state3.state_id, input_state3, state1.state_id, input_state1)
    state3.add_data_flow(state1.state_id, output_state1, state3.state_id, output_state3)

    return StateMachine(state3)


def test_concurrency_barrier_state_execution(caplog):

    testing_utils.test_multithreading_lock.acquire()
    sm = create_state_machine()
    root_state = sm.root_state
    state_machine = StateMachine(root_state)
    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    rafcon.core.singleton.state_machine_execution_engine.start()
    rafcon.core.singleton.state_machine_execution_engine.join()
    rafcon.core.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    try:
        assert root_state.output_data["output_data_port1"] == 42
        testing_utils.assert_logger_warnings_and_errors(caplog)
    finally:
        testing_utils.test_multithreading_lock.release()


if __name__ == '__main__':
    pytest.main([__file__])