from pytest import raises

# state machine
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.container_state import ContainerState
from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.core.states.state import InputDataPort
from testing_utils import assert_logger_warnings_and_errors
from rafcon.utils import log
logger = log.get_logger(__name__)


def test_create_state(caplog):
    state1 = ExecutionState("MyFirstState")

    assert len(state1.outcomes) == 3

    out = state1.add_outcome("MyFirstOutcome", 3)

    assert len(state1.outcomes) == 4

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

    assert len(state1.outcomes) == 3

    assert len(state1.input_data_ports) == 0
    assert len(state1.output_data_ports) == 0

    input_port_id = state1.add_input_data_port("input", "str")
    output_port_id = state1.add_output_data_port("output", "float")

    assert len(state1.input_data_ports) == 1
    assert len(state1.output_data_ports) == 1

    state1.remove_input_data_port(input_port_id)
    state1.remove_output_data_port(output_port_id)

    assert len(state1.input_data_ports) == 0
    assert len(state1.output_data_ports) == 0

    with raises(AttributeError):
        # AttributeError should be raised if not existing input is to be removed
        state1.remove_input_data_port(input_port_id)

    with raises(AttributeError):
        # AttributeError should be raised if not existing output is to be removed
        state1.remove_output_data_port(output_port_id)

    state2 = ExecutionState(name="State2", state_id=state1.state_id)

    # This should work, as data_type and default_value are optional parameters
    port = InputDataPort('input', data_port_id=99)

    with raises(AttributeError):
        # The name of the port differs in key and class member
        ExecutionState("test_execution_state", input_data_ports={'diff_input': port})


    # UTF8 strings should be allowed at least for descriptions
    state1.description = u'My English is not v\xc3ry good'

    assert_logger_warnings_and_errors(caplog)

    a = ExecutionState("test", state_id=10)
    b = ExecutionState("test", state_id=10)
    
    assert a == b
    assert a is not b


def test_create_container_state(caplog):

    container = ContainerState("Container")
    assert len(container.states) == 0

    input_port_container_state = container.add_input_data_port("input", "float")
    output_port_container_state = container.add_output_data_port("output", "float")
    scoped_var_1_container_state = container.add_scoped_variable("scope_1", "float")
    scoped_var_2_container_state = container.add_scoped_variable("scope_2", "float")

    state1 = ExecutionState("MyFirstState")
    container.add_state(state1)
    assert len(container.states) == 1

    with raises(AttributeError):
        # As the ID of two states is identical, the add method should raise an AttributeError
        container.add_state(ExecutionState("test_execution_state", state_id=state1.state_id))
        assert len(container.states) == 1

    state2 = ExecutionState("2nd State", state_id=container.state_id)
    logger.debug("Old state id: {0}".format(str(state2.state_id)))
    new_state_id = container.add_state(state2)
    logger.debug("New state id: {0}".format(str(new_state_id)))

    assert len(container.states) == 2

    input_state1 = state1.add_input_data_port("input", "float")
    output_state1 = state1.add_output_data_port("output", "float")
    input_state2 = state2.add_input_data_port("input", "float")
    input2_state2 = state2.add_input_data_port("input2", "float")
    output_state2 = state2.add_output_data_port("output", "float")

    assert len(container.data_flows) == 0
    container.add_data_flow(state1.state_id, output_state1, state2.state_id, input_state2)
    assert len(container.data_flows) == 1

    with raises(ValueError):
        # Data flow to connected input port
        container.add_data_flow(state1.state_id, input_state1, state2.state_id, input_state2)
    with raises(ValueError):
        # Data flow to non-existing port
        wrong_data_port_id = 218347
        container.add_data_flow(state1.state_id, output_state1, state2.state_id, wrong_data_port_id)
    with raises(ValueError):
        # Data flow from non-existing port
        wrong_data_port_id = 239847
        container.add_data_flow(state1.state_id, wrong_data_port_id, state2.state_id, input_state2)
    with raises(ValueError):
        # Data flow from non-existing state
        container.add_data_flow(-1, output_state1, state2.state_id, input_state2)
    with raises(ValueError):
        # Data flow to non-existing state
        container.add_data_flow(state1.state_id, output_state1, -1, input_state2)
    with raises(ValueError):
        # Connect port to itself
        container.add_data_flow(state1.state_id, output_state1, state1.state_id, output_state1)
    with raises(ValueError):
        # Connect two scoped variable
        container.add_data_flow(container.state_id, scoped_var_1_container_state,
                                container.state_id, scoped_var_2_container_state)

    container.add_data_flow(container.state_id, input_port_container_state, state1.state_id, input_state1)

    # with raises(ValueError):  # cannot connect data flow to same child state
    container.add_data_flow(state2.state_id, output_state2, state2.state_id, input2_state2)

    assert len(container.data_flows) == 3

    assert len(container.transitions) == 0

    container.add_transition(state1.state_id, -1, state2.state_id, None)
    assert len(container.transitions) == 1
    container.add_transition(state1.state_id, -2, container.state_id, -2)
    assert len(container.transitions) == 2
    t3 = container.add_transition(state2.state_id, -1, container.state_id, -1)
    assert len(container.transitions) == 3

    with raises(ValueError):
        # Transition from connected outcome
        container.add_transition(state1.state_id, -1, state2.state_id, None)
    with raises(ValueError):
        # Non-existing from state id
        container.add_transition(-1, -1, state2.state_id, None)
    with raises(ValueError):
        # Non-existing from outcome
        container.add_transition(state1.state_id, -3, state2.state_id, None)
    with raises(ValueError):
        # Non-existing to state id
        container.add_transition(state1.state_id, -1, -1, None)
    with raises(ValueError):
        # Non-existing to outcome
        container.add_transition(state1.state_id, -1, container.state_id, -3)
    with raises(ValueError):
        # Transition pointing to the state itself
        container.add_transition(state1.state_id, -2, state1.state_id, None)
    with raises(ValueError):
        # to_state_id and to_outcome not None
        container.add_transition(state1.state_id, -2, state1.state_id, -1)
    with raises(ValueError):
        # Transition from connected outcome
        container.add_transition(state2.state_id, -1, state2.state_id, -2)
    with raises(ValueError):
        # Transition going from one outcome to another outcome of the same state
        container.add_transition(state2.state_id, -1, state2.state_id, -2)

    with raises(AttributeError):
        # The removal of an undefined transition should throw an AttributeError
        container.remove_transition(-1)

    container.remove_transition(t3)
    assert len(container.transitions) == 2
    with raises(AttributeError):
        # The removal of an undefined transition should throw an AttributeError
        container.remove_transition(t3)
    container.add_transition(state2.state_id, -1, container.state_id, -1)
    assert len(container.transitions) == 3

    container.remove_state(state1.state_id)
    assert len(container.states) == 1
    assert len(container.transitions) == 1
    assert len(container.data_flows) == 1

    # barrier state remove test and bug test elements for issue #346
    barrier_state_id = container.add_state(BarrierConcurrencyState())
    container.remove(container.states[barrier_state_id])

    barrier_state_id = container.add_state(BarrierConcurrencyState())
    with raises(AttributeError):
        container.states[barrier_state_id].remove(container.states[barrier_state_id].states.values()[0])
    container.remove_state(barrier_state_id)
    ###########################################

    assert_logger_warnings_and_errors(caplog)


def test_port_and_outcome_removal(caplog):
    container = ContainerState("Container")
    input_container_state = container.add_input_data_port("input", "float")
    output_container_state = container.add_output_data_port("output", "float")
    scoped_variable_container_state = container.add_scoped_variable("scope", "float")

    assert len(container.transitions) == 0
    assert len(container.data_flows) == 0
    assert len(container.outcomes) == 3
    assert len(container.input_data_ports) == 1
    assert len(container.output_data_ports) == 1
    assert len(container.scoped_variables) == 1

    state1 = ExecutionState("test_execution_state")
    input_state1 = state1.add_input_data_port("input", "float")
    output_state1 = state1.add_output_data_port("output", "float")

    container.add_state(state1)
    container.add_transition(state1.state_id, 0, container.state_id, -2)
    container.add_data_flow(container.state_id, input_container_state, state1.state_id, input_state1)
    container.add_data_flow(state1.state_id, output_state1, container.state_id, output_container_state)
    container.add_data_flow(container.state_id, input_container_state,
                            container.state_id, scoped_variable_container_state)

    assert len(container.transitions) == 1
    assert len(container.data_flows) == 3

    state1.remove_outcome(0)
    assert len(container.transitions) == 0

    state1.remove_output_data_port(output_state1)
    assert len(container.data_flows) == 2

    state1.remove_input_data_port(input_state1)
    assert len(container.data_flows) == 1

    container.remove_scoped_variable(scoped_variable_container_state)
    assert len(container.data_flows) == 0

    assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    test_create_state(None)
    test_port_and_outcome_removal(None)
    test_create_container_state(None)
    # pytest.main([__file__])
