import pytest
from pytest import raises

from statemachine.states.execution_state import ExecutionState
from statemachine.states.hierarchy_state import HierarchyState
import statemachine.singleton
from statemachine.state_machine import StateMachine
import variables_for_pytest


def create_state_machine():
    state1 = ExecutionState("MyFirstState", path="../test_scripts", filename="global_variable_state.py")
    state1.add_outcome("first_outcome", 3)
    input_state1 = state1.add_input_data_port("input_data_port1", "float")
    output_state1 = state1.add_output_data_port("output_data_port1", "float")

    state3 = HierarchyState("MyFirstHierarchyState", path="../test_scripts", filename="hierarchy_state.py")
    state3.add_state(state1)
    state3.set_start_state(state1.state_id)
    state3.add_outcome("Container_Outcome", 6)
    state3.add_transition(state1.state_id, 3, None, 6)
    input_state3 = state3.add_input_data_port("input_data_port1", "float")
    output_state3 = state3.add_output_data_port("output_data_port1", "float")
    state3.add_data_flow(state3.state_id, input_state3, state1.state_id, input_state1)
    state3.add_data_flow(state1.state_id, output_state1, state3.state_id, output_state3)

    return state3


def test_concurrency_barrier_state_execution():

    container_state = create_state_machine()

    input_data = {"input_data_port1": 22.0}
    output_data = {"output_data_port1": None}
    container_state.input_data = input_data
    container_state.output_data = output_data
    state_machine = StateMachine(container_state)
    variables_for_pytest.test_multithrading_lock.acquire()
    statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    statemachine.singleton.state_machine_execution_engine.start()
    container_state.join()
    statemachine.singleton.state_machine_execution_engine.stop()
    variables_for_pytest.test_multithrading_lock.release()

    assert output_data["output_data_port1"] == 42


if __name__ == '__main__':
    pytest.main()
    #test_concurrency_barrier_state_execution()