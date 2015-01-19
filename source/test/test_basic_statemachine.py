import pytest
from pytest import raises
from statemachine.states.execution_state import ExecutionState
from statemachine.states.container_state import ContainerState
from statemachine.states.state import DataPort

def test_create_state():
    state1 = ExecutionState("MyFirstState")
    out = state1.add_outcome("MyFirstOutcome", 3)

    assert len(state1.outcomes) == 3

    state1.remove_outcome(out)

    with raises(AttributeError):
        # AttributeError should be raised if not existing outcome ID is to be removed
        state1.remove_outcome(out)

    assert len(state1.outcomes) == 2

    state1.add_input_data_port("input", "str")
    state1.add_output_data_port("output", "float")

    assert len(state1.input_data_ports) == 1
    assert len(state1.output_data_ports) == 1

    state1.remove_input_data_port("input")
    state1.remove_input_data_port("outut")

    assert len(state1.input_data_ports) == 0
    assert len(state1.output_data_ports) == 0

    with raises(AttributeError):
        # AttributeError should be raised if not existing input is to be removed
        state1.remove_input_data_port("input")

    with raises(AttributeError):
        # AttributeError should be raised if not existing output is to be removed
        state1.remove_output_data_port("output")

    state2 = ExecutionState(name="State2", state_id=state1.state_id)

    # This should work, as data_type and default_value are optional parameters
    port = DataPort('input')

    with raises(AttributeError):
        # The name of the port differs in key and class member
        ExecutionState(input_keys={'diff_input': port})

    with raises(AttributeError):
        container = ContainerState()
        container.add_state(state1)
        # As the ID of two states is identical, the add method should raise an AttributeError
        container.add_state(state2)

    # UTF8 strings should be allowed at least for descriptions
    state1.description = u'My English is not v\xc3ry good'


if __name__ == '__main__':
    pytest.main()