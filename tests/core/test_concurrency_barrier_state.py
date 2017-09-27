# core elements
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.core.storage import storage
from rafcon.core.state_machine import StateMachine
from rafcon.core.script import Script

# singleton elements
import rafcon.core.singleton

# test environment elements
import pytest
import testing_utils
from rafcon.core.constants import UNIQUE_DECIDER_STATE_ID


def create_concurrency_barrier_state():
    state1 = ExecutionState("FirstState", "id_of_state_1", path=testing_utils.TEST_SCRIPT_PATH,
                            filename="concurrency_barrier1.py")
    state1.add_outcome("FirstOutcomeState1", 3)
    state1.add_outcome("SecondOutcomeState1", 4)
    input_state1 = state1.add_input_data_port("input_data_port1", "float")
    output_state1 = state1.add_output_data_port("output_data_port1", "float")

    state2 = ExecutionState("SecondState", "id_of_state_2", path=testing_utils.TEST_SCRIPT_PATH,
                            filename="concurrency_barrier2.py")
    state2.add_outcome("FirstOutcomeState2", 3)
    state2.add_outcome("SecondOutcomeState2", 4)
    input_state2 = state2.add_input_data_port("input_data_port1", "float")
    output_state2 = state2.add_output_data_port("output_data_port1", "float")

    barrier_state = BarrierConcurrencyState("FirstConcurrencyState", "barrier_state_id")
    barrier_state.add_state(state1)
    barrier_state.add_state(state2)
    input1_state3 = barrier_state.add_input_data_port("input_data_port1", "float", 0.1)
    input2_state3 = barrier_state.add_input_data_port("input_data_port2", "float", 0.1)
    barrier_state.add_data_flow(barrier_state.state_id, input1_state3, state1.state_id, input_state1)
    barrier_state.add_data_flow(barrier_state.state_id, input2_state3, state2.state_id, input_state2)
    barrier_state.add_output_data_port("output_data_port1", "str", "default_output_value")
    barrier_state.add_outcome("success_outcome", 3)
    barrier_state.add_outcome("error_outcome", 4)

    barrier_state.states[UNIQUE_DECIDER_STATE_ID].name = "decider_state"
    barrier_state.states[UNIQUE_DECIDER_STATE_ID]._script = Script(path=testing_utils.TEST_SCRIPT_PATH,
                                                                   filename="decider_state.py",
                                                                   check_path=True,
                                                                   parent=barrier_state.states[UNIQUE_DECIDER_STATE_ID])
    barrier_state.states[UNIQUE_DECIDER_STATE_ID].add_outcome("FirstOutcomeDecider", 3)
    barrier_state.states[UNIQUE_DECIDER_STATE_ID].add_outcome("SecondOutcomeDecider", 4)

    barrier_state.add_transition(barrier_state.states[UNIQUE_DECIDER_STATE_ID].state_id, 3, barrier_state.state_id, 3)
    barrier_state.add_transition(barrier_state.states[UNIQUE_DECIDER_STATE_ID].state_id, 4, barrier_state.state_id, 4)

    return barrier_state


def test_concurrency_barrier_save_load(caplog):
    concurrency_barrier_state = create_concurrency_barrier_state()

    state_machine = StateMachine(concurrency_barrier_state)
    test_path = testing_utils.get_unique_temp_path()
    storage.save_state_machine_to_path(state_machine, test_path)
    sm_loaded = storage.load_state_machine_from_path(test_path)

    root_state = sm_loaded.root_state
    input_data = {"input_data_port1": 0.1, "input_data_port2": 0.1}
    output_data = {"output_data_port1": None}
    root_state.input_data = input_data
    root_state.output_data = output_data

    state_machine = StateMachine(root_state)
    testing_utils.test_multithreading_lock.acquire()
    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    rafcon.core.singleton.state_machine_execution_engine.start()
    rafcon.core.singleton.state_machine_execution_engine.join()

    try:
        assert rafcon.core.singleton.global_variable_manager.get_variable("var_x") == 10
        assert rafcon.core.singleton.global_variable_manager.get_variable("var_y") == 20
        assert root_state.final_outcome.outcome_id == 4

        rafcon.core.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog, expected_warnings=0, expected_errors=1)

if __name__ == '__main__':
    pytest.main([__file__])