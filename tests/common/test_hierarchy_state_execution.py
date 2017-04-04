# core elements
import rafcon.core.singleton
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.storage import storage
from rafcon.core.state_machine import StateMachine
from rafcon.core.state_elements.data_port import InputDataPort, OutputDataPort

# test environment elements
import testing_utils


def create_hierarchy_state():
    state1 = ExecutionState("MyFirstState", path=testing_utils.TEST_SCRIPT_PATH, filename="first_execution_state.py")
    state1.add_outcome("MyFirstOutcome", 3)
    state1.add_input_data_port("data_input_port1", "float")
    state1.add_output_data_port("faulty_output_port", "float")
    state1.add_output_data_port("data_output_port1", "float")

    state2 = HierarchyState("MyFirstHierarchyState")
    state2.add_state(state1)
    state2.set_start_state(state1.state_id)
    state2.add_outcome("Container_Outcome", 6)
    transition_id = state2.add_transition(state1.state_id, 3, state2.state_id, 6)
    # print state2.transitions[transition_id]
    input_data_port_id = state2.add_input_data_port("input1", "float", 42.0, data_port_id=42)
    state2.add_output_data_port("output1", "float")
    state2.add_data_flow(state2.state_id,
                         state2.get_io_data_port_id_from_name_and_type("input1", InputDataPort),
                         state1.state_id,
                         state1.get_io_data_port_id_from_name_and_type("data_input_port1", InputDataPort))
    state2.add_data_flow(state1.state_id,
                         state1.get_io_data_port_id_from_name_and_type("data_output_port1", OutputDataPort),
                         state2.state_id,
                         state2.get_io_data_port_id_from_name_and_type("output1", OutputDataPort))
    return state2


def test_hierarchy_state_execution(caplog):
    hierarchy_state = create_hierarchy_state()

    state_machine = StateMachine(hierarchy_state)

    try:
        # Changing the data type has to fail, as the data port is already connected to a data flow
        state_machine.root_state.input_data_ports[42].data_type = str
    except Exception, e:
        assert isinstance(e, ValueError)

    testing_utils.test_multithreading_lock.acquire()
    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    rafcon.core.singleton.state_machine_execution_engine.start()
    rafcon.core.singleton.state_machine_execution_engine.join()
    rafcon.core.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    try:
        assert hierarchy_state.output_data["output1"] == 52.0
        # 2 type error -> one child output port data type error and root state scoped data type error
        testing_utils.assert_logger_warnings_and_errors(caplog, expected_errors=2)
    finally:
        testing_utils.test_multithreading_lock.release()


def test_hierarchy_save_load_test(caplog):
    storage_path = testing_utils.get_unique_temp_path()

    hierarchy_state = create_hierarchy_state()
    sm = StateMachine(hierarchy_state)

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
        assert state_machine.root_state.output_data["output1"] == 52.0
        # 2 type error -> one child output port data type error and root state scoped data type error
        testing_utils.assert_logger_warnings_and_errors(caplog, expected_errors=2)
    finally:
        testing_utils.test_multithreading_lock.release()

if __name__ == '__main__':
    test_hierarchy_state_execution(None)
    test_hierarchy_save_load_test(None)
    # pytest.main([__file__])
