import pytest
from pytest import raises

from awesome_tool.statemachine.states.execution_state import ExecutionState
from awesome_tool.statemachine.states.barrier_concurrency_state import BarrierConcurrencyState
import awesome_tool.statemachine.singleton
from awesome_tool.statemachine.storage.storage import StateMachineStorage
from awesome_tool.statemachine.state_machine import StateMachine

import variables_for_pytest


def create_concurrency_barrier_state():
    state1 = ExecutionState("FirstState", path="../test_scripts", filename="concurrency_barrier1.py")
    state1.add_outcome("FirstOutcome", 3)
    input_state1 = state1.add_input_data_port("input_data_port1", "float")
    output_state1 = state1.add_output_data_port("output_data_port1", "float")

    state2 = ExecutionState("SecondState", path="../test_scripts", filename="concurrency_barrier2.py")
    state2.add_outcome("FirstOutcome", 3)
    input_state2 = state2.add_input_data_port("input_data_port1", "float")
    output_state2 = state2.add_output_data_port("output_data_port1", "float")

    state3 = BarrierConcurrencyState("FirstConcurrencyState", path="../test_scripts",
                                     filename="concurrency_container.py")
    state3.add_state(state1)
    state3.add_state(state2)
    input_state3 = state3.add_input_data_port("input_data_port1", "float", 0.1)
    input2_state3 = state3.add_input_data_port("input_data_port2", "float", 0.1)
    state3.add_data_flow(state3.state_id, input_state3, state1.state_id, input_state1)
    state3.add_data_flow(state3.state_id, input2_state3, state2.state_id, input_state2)
    state3.add_output_data_port("output_data_port1", "str", "default_output_value")

    return state3


def test_concurrency_barrier_state_execution():

    concurrency_barrier_state = create_concurrency_barrier_state()

    # input_data = {"input_data_port1": 0.1, "input_data_port2": 0.1}
    # output_data = {"output_data_port1": None}
    # concurrency_barrier_state.input_data = input_data
    # concurrency_barrier_state.output_data = output_data

    state_machine = StateMachine(concurrency_barrier_state)
    variables_for_pytest.test_multithrading_lock.acquire()
    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    awesome_tool.statemachine.singleton.state_machine_manager.active_state_machine = state_machine.state_machine_id
    awesome_tool.statemachine.singleton.state_machine_execution_engine.start()
    concurrency_barrier_state.join()
    awesome_tool.statemachine.singleton.state_machine_execution_engine.stop()

    assert awesome_tool.statemachine.singleton.global_variable_manager.get_variable("var_x") == 10
    assert awesome_tool.statemachine.singleton.global_variable_manager.get_variable("var_y") == 20
    awesome_tool.statemachine.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    variables_for_pytest.test_multithrading_lock.release()


def test_concurrency_barrier_save_load():
    test_storage = StateMachineStorage("../test_scripts/stored_statemachine")

    concurrency_barrier_state = create_concurrency_barrier_state()
    sm = StateMachine(concurrency_barrier_state)

    test_storage.save_statemachine_as_yaml(sm, "../test_scripts/stored_statemachine")
    sm_loaded, version, creation_time = test_storage.load_statemachine_from_yaml()

    root_state = sm_loaded.root_state
    input_data = {"input_data_port1": 0.1, "input_data_port2": 0.1}
    output_data = {"output_data_port1": None}
    root_state.input_data = input_data
    root_state.output_data = output_data

    state_machine = StateMachine(root_state)
    variables_for_pytest.test_multithrading_lock.acquire()
    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    awesome_tool.statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    awesome_tool.statemachine.singleton.state_machine_execution_engine.start()
    root_state.join()
    awesome_tool.statemachine.singleton.state_machine_execution_engine.stop()

    assert awesome_tool.statemachine.singleton.global_variable_manager.get_variable("var_x") == 10
    assert awesome_tool.statemachine.singleton.global_variable_manager.get_variable("var_y") == 20

    awesome_tool.statemachine.singleton.state_machine_manager.remove_state_machine(state_machine)
    variables_for_pytest.test_multithrading_lock.release()

if __name__ == '__main__':
    #pytest.main()
    test_concurrency_barrier_state_execution()
    #test_concurrency_barrier_save_load()