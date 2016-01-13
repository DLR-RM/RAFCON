import pytest
# test for expected exceptions
from pytest import raises

# core elements
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.storage.storage import StateMachineStorage
from rafcon.statemachine.state_machine import StateMachine

# singleton elements
import rafcon.statemachine.singleton

# test environment elements
import test_utils


def create_statemachine():
    state1 = ExecutionState("MyFirstState", path=rafcon.__path__[0] + "/../test_scripts",
                            filename="default_data_port_test_state.py")
    state1.add_outcome("first_outcome", 3)
    input_state1 = state1.add_input_data_port("input_data_port1", "str", "default_value")
    output_state1 = state1.add_output_data_port("output_data_port1", "str")

    state2 = HierarchyState("MyFirstHierarchyState")
    state2.add_state(state1)
    state2.set_start_state(state1.state_id)
    state2.add_outcome("Container_Outcome", 6)
    state2.add_transition(state1.state_id, 3, state2.state_id, 6)
    input_state2 = state2.add_input_data_port("input_data_port1", "str")
    output_state2 = state2.add_output_data_port("output_data_port1", "str")
    # state2.add_data_flow(state2.state_id,
    #                      input_state2,
    #                      state1.state_id,
    #                      input_state1)
    state2.add_data_flow(state1.state_id,
                         output_state1,
                         state2.state_id,
                         output_state2)

    return StateMachine(state2)


def test_default_values_of_data_ports(caplog):

    test_storage = StateMachineStorage(rafcon.__path__[0] + "/../test_scripts/stored_statemachine")

    sm = create_statemachine()

    test_storage.save_statemachine_to_path(sm, rafcon.__path__[0] + "/../test_scripts/stored_statemachine")
    [sm_loaded, version, creation_time] = test_storage.load_statemachine_from_path()

    root_state = sm_loaded.root_state

    state_machine = StateMachine(root_state)
    test_utils.test_multithrading_lock.acquire()
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    rafcon.statemachine.singleton.state_machine_execution_engine.start()
    root_state.join()
    rafcon.statemachine.singleton.state_machine_execution_engine.stop()
    rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    test_utils.test_multithrading_lock.release()
    test_utils.assert_logger_warnings_and_errors(caplog)

    print root_state.output_data
    assert root_state.output_data["output_data_port1"] == "default_value"


def test_unique_port_names(caplog):

    state = ExecutionState('execution state')

    state.add_input_data_port("in", "int", 0)
    state.add_output_data_port("out", "int", 0)

    # Data port name must be unique within data port set (input or output)
    with raises(ValueError):
        state.add_input_data_port("in", "int", 0)
    with raises(ValueError):
        state.add_input_data_port("in", "double", 0)
    with raises(ValueError):
        state.add_output_data_port("out", "int", 0)
    with raises(ValueError):
        state.add_output_data_port("out", "double", 0)

    # Names are allowed for other data port set
    state.add_output_data_port("in", "int", 0)
    state.add_input_data_port("out", "int", 0)

    assert len(state.input_data_ports) == 2
    assert len(state.output_data_ports) == 2

    state = HierarchyState('hierarchy state')

    in_id = state.add_input_data_port("in", "int", 0)
    out_id = state.add_output_data_port("out", "int", 0)
    scope_id = state.add_scoped_variable("scope", "int", 0)

    # Data port name must be unique within data port set (input, output or scope)
    with raises(ValueError):
        state.add_input_data_port("in", "int", 0)
    with raises(ValueError):
        state.add_input_data_port("in", "double", 0)
    with raises(ValueError):
        state.add_output_data_port("out", "int", 0)
    with raises(ValueError):
        state.add_output_data_port("out", "double", 0)
    with raises(ValueError):
        state.add_scoped_variable("scope", "int", 0)
    with raises(ValueError):
        state.add_scoped_variable("scope", "double", 0)

    # Names are allowed for other data port set
    state.add_output_data_port("in", "int", 0)
    state.add_scoped_variable("in", "int", 0)
    state.add_input_data_port("out", "int", 0)
    state.add_scoped_variable("out", "int", 0)
    state.add_input_data_port("scope", "int", 0)
    state.add_output_data_port("scope", "int", 0)

    # Also renaming should raise exceptions
    with raises(ValueError):
        state.input_data_ports[in_id].name = "out"
    with raises(ValueError):
        state.output_data_ports[out_id].name = "in"
    with raises(ValueError):
        state.scoped_variables[scope_id].name = "out"

    assert len(state.input_data_ports) == 3
    assert len(state.output_data_ports) == 3
    assert len(state.scoped_variables) == 3

    test_utils.assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    # test_default_values_of_data_ports(None)
    # test_unique_port_names(None)
    pytest.main([__file__])
