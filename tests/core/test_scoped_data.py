# core elements
import rafcon.core.singleton
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.state_elements.data_port import InputDataPort, OutputDataPort
from rafcon.core.storage import storage
from rafcon.core.state_machine import StateMachine

# test environment elements
import testing_utils
import pytest


def create_state_machine():
    state1 = ExecutionState("first_state", path=testing_utils.TEST_SCRIPT_PATH, filename="scoped_data_test_state1.py")
    state1.add_outcome("first_outcome", 3)
    state1.add_input_data_port("data_input_port1", "float")
    state1.add_output_data_port("data_output_port1", "float")

    state2 = ExecutionState("second_state", path=testing_utils.TEST_SCRIPT_PATH, filename="scoped_data_test_state2.py")
    state2.add_outcome("first_outcome", 3)
    state2.add_input_data_port("data_input_port1", "float")
    state2.add_output_data_port("data_output_port1", "float")

    state3 = HierarchyState("hierarchy_state")
    state3.add_state(state1)
    state3.add_state(state2)
    state3.set_start_state(state1.state_id)
    state3.add_outcome("Container_Outcome", 6)
    state3.add_transition(state1.state_id, 3, state2.state_id, None)
    state3.add_transition(state2.state_id, 3, state3.state_id, 6)
    state3.add_input_data_port("data_input_port1", "float", 22.0)
    state3.add_output_data_port("data_output_port1", "float")
    state3.add_data_flow(state3.state_id,
                         state3.get_io_data_port_id_from_name_and_type("data_input_port1", InputDataPort),
                         state1.state_id,
                         state1.get_io_data_port_id_from_name_and_type("data_input_port1", InputDataPort))
    state3.add_data_flow(state1.state_id,
                         state1.get_io_data_port_id_from_name_and_type("data_output_port1", OutputDataPort),
                         state2.state_id,
                         state2.get_io_data_port_id_from_name_and_type("data_input_port1", InputDataPort))
    state3.add_data_flow(state2.state_id,
                         state2.get_io_data_port_id_from_name_and_type("data_output_port1", OutputDataPort),
                         state3.state_id,
                         state3.get_io_data_port_id_from_name_and_type("data_output_port1", OutputDataPort))
    return StateMachine(state3)


# remember: scoped data is all data in a container state (including input_data, scoped variables and outputs of child
# states)
def test_scoped_data(caplog):
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
        assert state_machine.root_state.output_data["data_output_port1"] == 42.0
        testing_utils.assert_logger_warnings_and_errors(caplog)
    finally:
        testing_utils.test_multithreading_lock.release()


if __name__ == '__main__':
    pytest.main([__file__])
