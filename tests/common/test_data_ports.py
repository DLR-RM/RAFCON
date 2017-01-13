import pytest
import os
# test for expected exceptions
from pytest import raises

# core elements
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.storage import storage
from rafcon.core.state_machine import StateMachine

# singleton elements
import rafcon.core.singleton

# test environment elements
import testing_utils


def create_state_machine():
    state1 = ExecutionState("MyFirstState", path=testing_utils.TEST_SCRIPT_PATH,
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

    storage_path = testing_utils.get_unique_temp_path()
    print storage_path

    sm = create_state_machine()

    storage.save_state_machine_to_path(sm, storage_path)
    sm_loaded = storage.load_state_machine_from_path(storage_path)

    root_state = sm_loaded.root_state

    state_machine = StateMachine(root_state)
    testing_utils.test_multithreading_lock.acquire()

    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    rafcon.core.singleton.state_machine_execution_engine.start()
    rafcon.core.singleton.state_machine_execution_engine.join()
    rafcon.core.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    testing_utils.test_multithreading_lock.release()
    testing_utils.assert_logger_warnings_and_errors(caplog)

    print root_state.output_data
    assert root_state.output_data["output_data_port1"] == "default_value"


def test_last_wins_value_collection_for_data_ports(caplog):

    storage_path = testing_utils.get_unique_temp_path()

    sm_loaded = storage.load_state_machine_from_path(
        testing_utils.get_test_sm_path("unit_test_state_machines/last_data_wins_test"))

    root_state = sm_loaded.root_state

    state_machine = StateMachine(root_state)
    testing_utils.test_multithreading_lock.acquire()

    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    rafcon.core.singleton.state_machine_execution_engine.start()
    rafcon.core.singleton.state_machine_execution_engine.join()
    rafcon.core.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    testing_utils.test_multithreading_lock.release()
    testing_utils.assert_logger_warnings_and_errors(caplog)


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

    testing_utils.assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    test_default_values_of_data_ports(None)
    test_last_wins_value_collection_for_data_ports(None)
    test_unique_port_names(None)
    # pytest.main([__file__])
