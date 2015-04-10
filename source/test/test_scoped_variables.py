import pytest

from awesome_tool.statemachine.states.execution_state import ExecutionState
from awesome_tool.statemachine.states.hierarchy_state import HierarchyState
from awesome_tool.statemachine.states.state import StateType, DataPortType
from awesome_tool.statemachine.storage.storage import StateMachineStorage
import awesome_tool.statemachine.singleton
from awesome_tool.statemachine.state_machine import StateMachine
import variables_for_pytest


def create_statemachine():
    state1 = ExecutionState("scoped_data_test_state", path="../test_scripts", filename="scoped_variable_test_state.py")
    state1.add_outcome("loop", 1)
    input1_state1 = state1.add_input_data_port("input_data_port1", "float")
    input2_state1 = state1.add_input_data_port("input_data_port2", "float")
    output_state1 = state1.add_output_data_port("output_data_port1", "float")

    state2 = HierarchyState("scoped_data_hierarchy_state", path="../test_scripts", filename="scoped_variable_hierarchy_state.py")
    state2.add_state(state1)
    state2.set_start_state(state1.state_id)
    state2.add_transition(state1.state_id, 0, None, 0)
    state2.add_transition(state1.state_id, 1, state1.state_id, None)
    input_state2 = state2.add_input_data_port("input_data_port1", "float")
    output_state2 = state2.add_output_data_port("output_data_port1", "float")
    scoped_variable_state2 = state2.add_scoped_variable("scoped_variable1", "float", 5.0)

    state2.add_data_flow(state2.state_id,
                         state2.get_scoped_variable_from_name("scoped_variable1"),
                         state1.state_id,
                         input1_state1)

    state2.add_data_flow(state2.state_id,
                         input_state2,
                         state1.state_id,
                         input2_state1)

    state2.add_data_flow(state1.state_id,
                         output_state1,
                         state2.state_id,
                         output_state2)

    state2.add_data_flow(state1.state_id,
                         output_state1,
                         state2.state_id,
                         scoped_variable_state2)


    return StateMachine(state2)


def test_scoped_variables():

    s = StateMachineStorage("../test_scripts/stored_statemachine")

    sm = create_statemachine()

    s.save_statemachine_as_yaml(sm, "../test_scripts/stored_statemachine")
    [sm_loaded, version, creation_time] = s.load_statemachine_from_yaml()

    input_data = {"input_data_port1": 10.0}
    output_data = {"output_data_port1": None}

    sm_loaded.root_state.input_data = input_data
    sm_loaded.root_state.output_data = output_data
    state_machine = StateMachine(sm_loaded.root_state)

    variables_for_pytest.test_multithrading_lock.acquire()
    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    awesome_tool.statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    awesome_tool.statemachine.singleton.state_machine_execution_engine.start()
    sm_loaded.root_state.join()
    awesome_tool.statemachine.singleton.state_machine_execution_engine.stop()
    awesome_tool.statemachine.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    variables_for_pytest.test_multithrading_lock.release()

    #print output_data["output_data_port1"]
    assert output_data["output_data_port1"] == 42


if __name__ == '__main__':
    #pytest.main()
    test_scoped_variables()
