# core elements
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.state import DataPortType
from rafcon.statemachine.storage.storage import StateMachineStorage
from rafcon.statemachine.state_machine import StateMachine

# singleton elements
import rafcon.statemachine.singleton

# test environment elements
import test_utils
import pytest


def create_hierarchy_state():
    state1 = ExecutionState("MyFirstState", path=rafcon.__path__[0] + "/../test_scripts", filename="first_execution_state.py")
    state1.add_outcome("MyFirstOutcome", 3)
    state1.add_input_data_port("data_input_port1", "float")
    state1.add_output_data_port("data_output_port1", "float")

    state2 = HierarchyState("MyFirstHierarchyState", path=rafcon.__path__[0] + "/../test_scripts", filename="hierarchy_state.py")
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

    test_utils.test_multithrading_lock.acquire()
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    rafcon.statemachine.singleton.state_machine_execution_engine.start()
    hierarchy_state.join()
    rafcon.statemachine.singleton.state_machine_execution_engine.stop()
    rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    test_utils.test_multithrading_lock.release()

    assert hierarchy_state.output_data["output1"] == 52.0
    test_utils.assert_logger_warnings_and_errors(caplog)


def test_hierarchy_save_load_test(caplog):
    s = StateMachineStorage(rafcon.__path__[0] + "/../test_scripts/stored_statemachine")

    hierarchy_state = create_hierarchy_state()
    sm = StateMachine(hierarchy_state)

    s.save_statemachine_as_yaml(sm, rafcon.__path__[0] + "/../test_scripts/stored_statemachine")
    [sm_loaded, version, creation_time] = s.load_statemachine_from_yaml()

    state_machine = StateMachine(sm_loaded.root_state)

    test_utils.test_multithrading_lock.acquire()
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    rafcon.statemachine.singleton.state_machine_execution_engine.start()
    sm_loaded.root_state.join()
    rafcon.statemachine.singleton.state_machine_execution_engine.stop()
    rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    test_utils.test_multithrading_lock.release()

    assert state_machine.root_state.output_data["output1"] == 52.0
    test_utils.assert_logger_warnings_and_errors(caplog)

if __name__ == '__main__':
    pytest.main([__file__])