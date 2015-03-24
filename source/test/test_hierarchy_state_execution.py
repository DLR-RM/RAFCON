import pytest
from pytest import raises
import awesome_tool

from awesome_tool.statemachine.states.execution_state import ExecutionState
from awesome_tool.statemachine.states.hierarchy_state import HierarchyState
import awesome_tool.statemachine.singleton
from awesome_tool.statemachine.states.state import DataPortType
from awesome_tool.statemachine.storage.storage import StateMachineStorage
from awesome_tool.statemachine.state_machine import StateMachine
import variables_for_pytest


def create_hierarchy_state():
    state1 = ExecutionState("MyFirstState", path="../test_scripts", filename="first_execution_state.py")
    state1.add_outcome("MyFirstOutcome", 3)
    state1.add_input_data_port("data_input_port1", "float")
    state1.add_output_data_port("data_output_port1", "float")

    state2 = HierarchyState("MyFirstHierarchyState", path="../test_scripts", filename="hierarchy_state.py")
    state2.add_state(state1)
    state2.set_start_state(state1.state_id)
    state2.add_outcome("Container_Outcome", 6)
    state2.add_transition(state1.state_id, 3, None, 6)
    state2.add_input_data_port("input1", "float")
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

def test_hierarchy_state_execution():
    hierarchy_state = create_hierarchy_state()

    input_data = {"input1": 42.0}
    output_data = {"output1": None}
    hierarchy_state.input_data = input_data
    hierarchy_state.output_data = output_data
    state_machine = StateMachine(hierarchy_state)

    variables_for_pytest.test_multithrading_lock.acquire()
    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    awesome_tool.statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    awesome_tool.statemachine.singleton.state_machine_execution_engine.start()
    hierarchy_state.join()
    awesome_tool.statemachine.singleton.state_machine_execution_engine.stop()
    awesome_tool.statemachine.singleton.state_machine_manager.remove_state_machine(state_machine)
    variables_for_pytest.test_multithrading_lock.release()

    assert output_data["output1"] == 52.0

def test_hierarchy_save_load_test():
    s = StateMachineStorage("../test_scripts/stored_statemachine")

    hierarchy_state = create_hierarchy_state()
    sm = StateMachine(hierarchy_state)

    s.save_statemachine_as_yaml(sm, "../test_scripts/stored_statemachine")
    [sm_loaded, version, creation_time] = s.load_statemachine_from_yaml()

    input_data = {"input1": 42.0}
    output_data = {"output1": None}
    sm_loaded.root_state.input_data = input_data
    sm_loaded.root_state.output_data = output_data
    state_machine = StateMachine(sm_loaded.root_state)

    variables_for_pytest.test_multithrading_lock.acquire()
    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    awesome_tool.statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    awesome_tool.statemachine.singleton.state_machine_execution_engine.start()
    sm_loaded.root_state.join()
    awesome_tool.statemachine.singleton.state_machine_execution_engine.stop()
    awesome_tool.statemachine.singleton.state_machine_manager.remove_state_machine(state_machine)
    variables_for_pytest.test_multithrading_lock.release()

    assert output_data["output1"] == 52.0

if __name__ == '__main__':
    #pytest.main()
    test_hierarchy_save_load_test()