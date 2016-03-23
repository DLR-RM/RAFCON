import os

# core elements
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from rafcon.statemachine.storage.storage import StateMachineStorage
from rafcon.statemachine.state_machine import StateMachine

# singleton elements
import rafcon.statemachine.singleton

# test environment elements
import testing_utils
import pytest


def create_preemption_statemachine():
    state1 = ExecutionState("FirstState", path=rafcon.__path__[0] + "/../test_scripts", filename="concurrence_preemption1.py")
    state1.add_outcome("FirstOutcome", 3)
    input_state1 = state1.add_input_data_port("input_data_port1", "float")

    state2 = ExecutionState("SecondState", path=rafcon.__path__[0] + "/../test_scripts", filename="concurrence_preemption2.py")
    state2.add_outcome("FirstOutcome", 3)
    input_state2 = state2.add_input_data_port("input_data_port1", "float")

    state3 = PreemptiveConcurrencyState("FirstConcurrencyState")
    state3.add_state(state1)
    state3.add_state(state2)
    state3.add_outcome("State1 preempted", 3)
    input_state3 = state3.add_input_data_port("input_data_port1", "float", 0.1)
    input2_state3 = state3.add_input_data_port("input_data_port2", "float", 0.1)
    state3.add_data_flow(state3.state_id, input_state3, state1.state_id, input_state1)
    state3.add_data_flow(state3.state_id, input2_state3, state2.state_id, input_state2)
    state3.add_transition(state1.state_id, 3, state3.state_id, 3)

    return StateMachine(state3)


def test_concurrency_preemption_state_execution(caplog):

    preemption_state_sm = create_preemption_statemachine()

    testing_utils.test_multithrading_lock.acquire()
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(preemption_state_sm)
    rafcon.statemachine.singleton.state_machine_manager.active_state_machine_id = preemption_state_sm.state_machine_id
    rafcon.statemachine.singleton.state_machine_execution_engine.start()
    rafcon.statemachine.singleton.state_machine_execution_engine.join()

    assert rafcon.statemachine.singleton.global_variable_manager.get_variable("preempted_state2_code") == "DF3LFXD34G"
    assert preemption_state_sm.root_state.final_outcome.outcome_id == 3
    rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(preemption_state_sm.state_machine_id)
    testing_utils.test_multithrading_lock.release()
    testing_utils.assert_logger_warnings_and_errors(caplog)


def test_concurrency_preemption_save_load(caplog):
    testing_utils.test_multithrading_lock.acquire()

    storage_path = testing_utils.get_unique_temp_path()
    s = StateMachineStorage(storage_path)

    preemption_state_sm = create_preemption_statemachine()

    s.save_statemachine_to_path(preemption_state_sm, storage_path)
    [root_state, version, creation_time] = s.load_statemachine_from_path(storage_path)

    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(preemption_state_sm)
    rafcon.statemachine.singleton.state_machine_manager.active_state_machine_id = preemption_state_sm.state_machine_id
    rafcon.statemachine.singleton.state_machine_execution_engine.start()
    rafcon.statemachine.singleton.state_machine_execution_engine.join()

    assert rafcon.statemachine.singleton.global_variable_manager.get_variable("preempted_state2_code") == "DF3LFXD34G"
    rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(preemption_state_sm.state_machine_id)
    testing_utils.test_multithrading_lock.release()
    testing_utils.assert_logger_warnings_and_errors(caplog)

if __name__ == '__main__':
    pytest.main([__file__])