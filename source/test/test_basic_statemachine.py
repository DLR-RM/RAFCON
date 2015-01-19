import pytest
from statemachine.states.execution_state import ExecutionState


def test_create_state():
    state1 = ExecutionState("MyFirstState", path="../../test_scripts", filename="first_state.py")
    state1.add_outcome("MyFirstOutcome", 3)

    assert len(state1.outcomes) == 3

    state1.add_input_data_port("MyFirstDataInputPort", "str")
    state1.add_output_data_port("MyFirstDataOutputPort", "float")

    assert len(state1.input_data_ports) == 1
    assert len(state1.output_data_ports) == 1

if __name__ == '__main__':
    pytest.main()