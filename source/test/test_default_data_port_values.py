import pytest

from awesome_tool.statemachine.states.execution_state import ExecutionState
from awesome_tool.statemachine.states.hierarchy_state import HierarchyState
from awesome_tool.statemachine.storage.storage import StateMachineStorage
import awesome_tool.statemachine.singleton
from awesome_tool.statemachine.state_machine import StateMachine
import variables_for_pytest


def create_statemachine():
    state1 = ExecutionState("MyFirstState", path="../test_scripts", filename="default_data_port_test_state.py")
    state1.add_outcome("first_outcome", 3)
    input_state1 = state1.add_input_data_port("input_data_port1", "str", "default_value")
    output_state1 = state1.add_output_data_port("output_data_port1", "str")

    state2 = HierarchyState("MyFirstHierarchyState", path="../test_scripts", filename="hierarchy_state.py")
    state2.add_state(state1)
    state2.set_start_state(state1.state_id)
    state2.add_outcome("Container_Outcome", 6)
    state2.add_transition(state1.state_id, 3, None, 6)
    input_state2 = state2.add_input_data_port("input_data_port1", "str")
    output_state2 = state2.add_output_data_port("output_data_port1", "str")
    state2.add_data_flow(state2.state_id,
                         input_state2,
                         state1.state_id,
                         input_state1)
    state2.add_data_flow(state1.state_id,
                         output_state1,
                         state2.state_id,
                         output_state2)

    return StateMachine(state2)


def test_default_values_of_data_ports():

    test_storage = StateMachineStorage("../test_scripts/stored_statemachine")

    sm = create_statemachine()

    test_storage.save_statemachine_as_yaml(sm, "../test_scripts/stored_statemachine")
    [sm_loaded, version, creation_time] = test_storage.load_statemachine_from_yaml()

    root_state = sm_loaded.root_state
    input_data = {}
    output_data = {"output_data_port1": None}
    root_state.input_data = input_data
    root_state.output_data = output_data

    state_machine = StateMachine(root_state)
    variables_for_pytest.test_multithrading_lock.acquire()
    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    awesome_tool.statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    awesome_tool.statemachine.singleton.state_machine_execution_engine.start()
    root_state.join()
    awesome_tool.statemachine.singleton.state_machine_execution_engine.stop()
    awesome_tool.statemachine.singleton.state_machine_manager.remove_state_machine(state_machine)
    variables_for_pytest.test_multithrading_lock.release()

    #print output_data["output_data_port1"]
    assert output_data["output_data_port1"] == "default_value"


if __name__ == '__main__':
    #pytest.main()
    test_default_values_of_data_ports()
