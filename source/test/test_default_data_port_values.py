import pytest

from statemachine.states.execution_state import ExecutionState
from statemachine.states.hierarchy_state import HierarchyState
from statemachine.states.state import StateType, DataPortType
from statemachine.storage.storage import Storage
import statemachine.singleton
from statemachine.state_machine import StateMachine
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

    return state2


def test_default_values_of_data_ports():

    s = Storage("../test_scripts/stored_statemachine")

    root_state = create_statemachine()

    s.save_statemachine_as_yaml(root_state, "../test_scripts/stored_statemachine")
    [root_state, version, creation_time] = s.load_statemachine_from_yaml()

    input_data = {}
    output_data = {"output_data_port1": None}
    root_state.input_data = input_data
    root_state.output_data = output_data

    state_machine = StateMachine(root_state)
    variables_for_pytest.test_multithrading_lock.acquire()
    statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    statemachine.singleton.state_machine_execution_engine.start()
    root_state.join()
    statemachine.singleton.state_machine_execution_engine.stop()
    variables_for_pytest.test_multithrading_lock.release()

    #print output_data["output_data_port1"]
    assert output_data["output_data_port1"] == "default_value"


if __name__ == '__main__':
    #pytest.main()
    test_default_values_of_data_ports()
