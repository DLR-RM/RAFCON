import os

# core elements
import rafcon.statemachine.singleton
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.state import DataPortType
from rafcon.statemachine.storage import storage
from rafcon.statemachine.state_machine import StateMachine

# test environment elements
import testing_utils
import pytest


def create_hierarchy_state():
    state1 = ExecutionState("MyFirstState", path=rafcon.__path__[0] + "/../test_scripts", filename="first_execution_state.py")
    state1.add_outcome("MyFirstOutcome", 3)
    state1.add_input_data_port("data_input_port1", "float")
    state1.add_output_data_port("faulty_output_port", "float")
    state1.add_output_data_port("data_output_port1", "float")

    state2 = HierarchyState("MyFirstHierarchyState")
    state2.add_state(state1)
    state2.set_start_state(state1.state_id)
    state2.add_outcome("Container_Outcome", 6)
    transition_id = state2.add_transition(state1.state_id, 3, state2.state_id, 6)
    print state2.transitions[transition_id]
    state2.add_input_data_port("input1", "float", 42.0)
    state2.add_output_data_port("output1", "float")
    state2.add_data_flow(state2.state_id,
                         state2.get_io_data_port_id_from_name_and_type("input1", DataPortType.INPUT),
                         state1.state_id,
                         state1.get_io_data_port_id_from_name_and_type("data_input_port1", DataPortType.INPUT))
    state2.add_data_flow(state1.state_id,
                         state1.get_io_data_port_id_from_name_and_type("data_output_port1", DataPortType.OUTPUT),
                         state2.state_id,
                         state2.get_io_data_port_id_from_name_and_type("output1", DataPortType.OUTPUT))
    return state2


def test_hierarchy_state_execution(caplog):
    hierarchy_state = create_hierarchy_state()

    state_machine = StateMachine(hierarchy_state)

    testing_utils.test_multithreading_lock.acquire()
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    rafcon.statemachine.singleton.state_machine_execution_engine.start()
    rafcon.statemachine.singleton.state_machine_execution_engine.join()
    rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    testing_utils.test_multithreading_lock.release()

    assert hierarchy_state.output_data["output1"] == 52.0
    testing_utils.assert_logger_warnings_and_errors(caplog, expected_errors=1)


def test_hierarchy_save_load_test(caplog):
    storage_path = testing_utils.get_unique_temp_path()

    hierarchy_state = create_hierarchy_state()
    sm = StateMachine(hierarchy_state)

    storage.save_state_machine_to_path(sm, storage_path)
    sm_loaded = storage.load_state_machine_from_path(storage_path)

    state_machine = StateMachine(sm_loaded.root_state)

    testing_utils.test_multithreading_lock.acquire()
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    rafcon.statemachine.singleton.state_machine_execution_engine.start()
    rafcon.statemachine.singleton.state_machine_execution_engine.join()
    rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    testing_utils.test_multithreading_lock.release()

    assert state_machine.root_state.output_data["output1"] == 52.0
    testing_utils.assert_logger_warnings_and_errors(caplog, expected_errors=1)

if __name__ == '__main__':
    test_hierarchy_state_execution(None)
    test_hierarchy_save_load_test(None)
    # pytest.main([__file__])