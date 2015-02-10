import pytest
from pytest import raises

from statemachine.states.execution_state import ExecutionState
from statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState
import statemachine.singleton
from statemachine.storage.storage import Storage


def create_preemption_statemachine():
    state1 = ExecutionState("FirstState", path="../test_scripts", filename="concurrence_preemption1.py")
    state1.add_outcome("FirstOutcome", 3)
    input_state1 = state1.add_input_data_port("input_data_port1", "float")

    state2 = ExecutionState("SecondState", path="../test_scripts", filename="concurrence_preemption2.py")
    state2.add_outcome("FirstOutcome", 3)
    input_state2 = state2.add_input_data_port("input_data_port1", "float")

    state3 = PreemptiveConcurrencyState("FirstConcurrencyState", path="../test_scripts",
                                        filename="concurrency_container.py")
    state3.add_state(state1)
    state3.add_state(state2)
    state3.add_outcome("State1 preempted", 3)
    input_state3 = state3.add_input_data_port("input_data_port1", "float", 3.0)
    input2_state3 = state3.add_input_data_port("input_data_port2", "float", 3.0)
    state3.add_data_flow(state3.state_id, input_state3, state1.state_id, input_state1)
    state3.add_data_flow(state3.state_id, input2_state3, state2.state_id, input_state2)
    state3.add_transition(state1.state_id, 3, None, 3)

    return state3


def test_concurrency_preemption_state_execution():

    preemption_state = create_preemption_statemachine()

    input_data = {"input_data_port1": 0.1, "input_data_port2": 0.1}
    preemption_state.input_data = input_data
    preemption_state.output_data = {}
    statemachine.singleton.state_machine_manager.root_state = preemption_state
    statemachine.singleton.state_machine_execution_engine.start()
    preemption_state.join()
    statemachine.singleton.state_machine_execution_engine.stop()

    assert statemachine.singleton.global_variable_manager.get_variable("preempted_state2_code") == "DF3LFXD34G"


def test_concurrency_preemption_save_load():
    s = Storage("../test_scripts/stored_statemachine")

    preemption_state = create_preemption_statemachine()

    s.save_statemachine_as_yaml(preemption_state)
    [root_state, version, creation_time] = s.load_statemachine_from_yaml()

    input_data = {"input_data_port1": 0.1, "input_data_port2": 0.1}
    output_data = {"output_data_port1": None}
    preemption_state.input_data = input_data
    preemption_state.output_data = output_data
    statemachine.singleton.state_machine_manager.root_state = preemption_state
    statemachine.singleton.state_machine_execution_engine.start()
    preemption_state.join()
    statemachine.singleton.state_machine_execution_engine.stop()

    assert statemachine.singleton.global_variable_manager.get_variable("preempted_state2_code") == "DF3LFXD34G"

if __name__ == '__main__':
    pytest.main()
    #test_concurrency_preemption_state_execution()