import pytest
from pytest import raises
from statemachine.states.execution_state import ExecutionState
from statemachine.states.container_state import ContainerState
from statemachine.states.state import DataPort

def test_create_state():
    state1 = ExecutionState("MyFirstState")

    assert len(state1.outcomes) == 2

    out = state1.add_outcome("MyFirstOutcome", 3)

    assert len(state1.outcomes) == 3

    state1.remove_outcome(out)

    with raises(AttributeError):
        # AttributeError should be raised if not existing outcome ID is to be removed
        state1.remove_outcome(out)
    with raises(AttributeError):
        # AttributeError should be raised if outcome preempted is to be removed
        state1.remove_outcome(-1)
    with raises(AttributeError):
        # AttributeError should be raised if outcome aborted is to be removed
        state1.remove_outcome(-2)

    assert len(state1.outcomes) == 2

    assert len(state1.input_data_ports) == 0
    assert len(state1.output_data_ports) == 0

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


    # UTF8 strings should be allowed at least for descriptions
    state1.description = u'My English is not v\xc3ry good'


def test_create_container_state():

    container = ContainerState("Container")
    assert len(container.states) == 0

    container.add_input_data_port("input", "double")
    container.add_output_data_port("input", "double")

    state1 = ExecutionState("MyFirstState")
    container.add_state(state1)
    assert len(container.states) == 1

    with raises(AttributeError):
        # As the ID of two states is identical, the add method should raise an AttributeError
        container.add_state(ExecutionState(state_id=state1.state_id))
        assert len(container.states) == 1

    state2 = ExecutionState("2nd State")
    container.add_state(state2)
    assert len(container.states) == 2

    state1.add_input_data_port("input", "double")
    state1.add_output_data_port("output", "float")
    state2.add_input_data_port("input", "float")
    state2.add_output_data_port("output", "double")

    assert len(container.data_flows) == 0
    container.add_data_flow(state1.state_id, "output", state2.state_id, "input")
    assert len(container.data_flows) == 1

    with raises(AttributeError):
        container.add_data_flow(state1.state_id, "output", state2.state_id, "no_input")
    with raises(AttributeError):
        container.add_data_flow(state1.state_id, "no_output", state2.state_id, "input")
    with raises(AttributeError):
        container.add_data_flow(-1, "output", state2.state_id, "input")
    with raises(AttributeError):
        container.add_data_flow(state1.state_id, "output", -1, "input")

    container.add_data_flow(container.state_id, "input", state1, "input")
    container.add_data_flow(state2.state_id, "output", state2, "output")

    assert len(container.data_flows) == 3

    assert len(container.transitions) == 0

    container.add_transition(state1.state_id, -1, state2.state_id)
    assert len(container.transitions) == 1
    container.add_transition(state1.state_id, -2, None, -2)
    assert len(container.transitions) == 2
    t3 = container.add_transition(state2.state_id, -1, None, -1)
    assert len(container.transitions) == 3

    with raises(AttributeError):
        # The removal of an undefined transition should throw an AttributeError
        container.remove_transition(-1)

    container.remove_transition(t3)
    assert len(container.transitions) == 2
    with raises(AttributeError):
        # The removal of an undefined transition should throw an AttributeError
        container.remove_transition(t3)
    container.add_transition(state2.state_id, -1, None, -1)
    assert len(container.transitions) == 3

    container.remove_state(state1.state_id)
    assert len(container.states) == 1
    assert len(container.transitions) == 1
    assert len(container.data_flows) == 1



if __name__ == '__main__':
    pytest.main()