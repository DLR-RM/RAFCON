import pytest
from pytest import raises

from statemachine.states.execution_state import ExecutionState
from statemachine.states.hierarchy_state import HierarchyState
import statemachine.singleton
from statemachine.external_modules.external_module import ExternalModule
from statemachine.state_machine import StateMachine
import variables_for_pytest

def create_statemachine():
    state0 = HierarchyState("hierarchy_state")
    state1 = ExecutionState("execution_state", path="../test_scripts", filename="external_module_test_state.py")
    state1.add_outcome("success", 0)
    state1_input = state1.add_input_data_port("input_data_port1", "float")
    state1_output= state1.add_output_data_port("output_data_port1", "float")
    state0.add_state(state1)
    state0.set_start_state(state1.state_id)
    state0.add_outcome("success", 0)
    state0.add_transition(state1.state_id, 0, None, 0)
    state0_input = state0.add_input_data_port("input_data_port1", "float")
    state0_output = state0.add_output_data_port("output_data_port1", "float")
    state0.add_data_flow(state0.state_id, state0_input,
                         state1.state_id, state1_input)
    state0.add_data_flow(state1.state_id, state1_output,
                         state0.state_id, state0_output)
    return state0


def test_external_module():
    root_state = create_statemachine()

    input_data = {"input_data_port1": 32.0}
    output_data = {"output_data_port1": None}
    root_state.input_data = input_data
    root_state.output_data = output_data

    em = ExternalModule(name="em1", module_name="external_module_test", class_name="TestModule")
    statemachine.singleton.external_module_manager.add_external_module(em)
    statemachine.singleton.external_module_manager.external_modules["em1"].connect([])
    statemachine.singleton.external_module_manager.external_modules["em1"].start()

    state_machine = StateMachine(root_state)
    variables_for_pytest.test_multithrading_lock.acquire()
    statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    statemachine.singleton.state_machine_execution_engine.start()
    root_state.join()
    statemachine.singleton.state_machine_execution_engine.stop()
    variables_for_pytest.test_multithrading_lock.release()

    assert output_data["output_data_port1"] == 42.0


if __name__ == '__main__':
    #pytest.main()
    test_external_module()