import pytest
from pytest import raises

from statemachine.states.execution_state import ExecutionState
import statemachine.singleton
from statemachine.external_modules.external_module import ExternalModule


def create_statemachine():
    state1 = ExecutionState("MyFirstState", path="../test_scripts", filename="external_module_test_state.py")
    state1.add_outcome("MyFirstOutcome", 3)
    state1.add_input_data_port("input_data_port1", "float")
    state1.add_output_data_port("output_data_port1", "float")
    return state1


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

    statemachine.singleton.state_machine_manager.root_state = root_state
    statemachine.singleton.state_machine_execution_engine.start()
    root_state.join()
    statemachine.singleton.state_machine_execution_engine.stop()

    assert output_data["output_data_port1"] == 42.0


if __name__ == '__main__':
    pytest.main()
    #test_external_module()