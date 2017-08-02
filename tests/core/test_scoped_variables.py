# core elements
import rafcon.core.singleton
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.storage import storage
from rafcon.core.state_machine import StateMachine

# test environment elements
import testing_utils
import pytest


def create_state_machine():
    state1 = ExecutionState("scoped_data_test_state", path=testing_utils.TEST_SCRIPT_PATH,
                            filename="scoped_variable_test_state.py")
    state1.add_outcome("loop", 1)
    input1_state1 = state1.add_input_data_port("input_data_port1", "float")
    input2_state1 = state1.add_input_data_port("input_data_port2", "float")
    output_state1 = state1.add_output_data_port("output_data_port1", "float")

    state2 = HierarchyState("scoped_data_hierarchy_state")
    state2.add_state(state1)
    state2.set_start_state(state1.state_id)
    state2.add_transition(state1.state_id, 0, state2.state_id, 0)
    state2.add_transition(state1.state_id, 1, state1.state_id, None)
    input_state2 = state2.add_input_data_port("input_data_port1", "float", 10.0)
    output_state2 = state2.add_output_data_port("output_data_port1", "float")
    scoped_variable_state2 = state2.add_scoped_variable("scoped_variable1", "float", 12.0)

    state2.add_data_flow(state2.state_id,
                         state2.get_scoped_variable_from_name("scoped_variable1"),
                         state1.state_id,
                         input1_state1)

    state2.add_data_flow(state2.state_id,
                         input_state2,
                         state1.state_id,
                         input2_state1)

    state2.add_data_flow(state1.state_id,
                         output_state1,
                         state2.state_id,
                         output_state2)

    state2.add_data_flow(state1.state_id,
                         output_state1,
                         state2.state_id,
                         scoped_variable_state2)

    return StateMachine(state2)


def test_scoped_variables(caplog):

    storage_path = testing_utils.get_unique_temp_path()

    sm = create_state_machine()

    storage.save_state_machine_to_path(sm, storage_path)
    sm_loaded = storage.load_state_machine_from_path(storage_path)

    state_machine = StateMachine(sm_loaded.root_state)

    testing_utils.test_multithreading_lock.acquire()
    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    rafcon.core.singleton.state_machine_execution_engine.start()
    rafcon.core.singleton.state_machine_execution_engine.join()
    rafcon.core.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    try:
        assert state_machine.root_state.output_data["output_data_port1"] == 42
        testing_utils.assert_logger_warnings_and_errors(caplog)
    finally:
        testing_utils.test_multithreading_lock.release()


if __name__ == '__main__':
    pytest.main([__file__])
