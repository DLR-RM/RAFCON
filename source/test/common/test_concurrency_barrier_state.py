# core elements
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.statemachine.storage.storage import StateMachineStorage
from rafcon.statemachine.state_machine import StateMachine
from rafcon.statemachine.script import Script, ScriptType

# singleton elements
import rafcon.statemachine.singleton

# test environment elements
import pytest
import test_utils
from rafcon.statemachine.enums import UNIQUE_DECIDER_STATE_ID


def create_concurrency_barrier_state():
    state1 = ExecutionState("FirstState", "id_of_state_1", path=rafcon.__path__[0] + "/../test_scripts",
                            filename="concurrency_barrier1.py")
    state1.add_outcome("FirstOutcomeState1", 3)
    state1.add_outcome("SecondOutcomeState1", 4)
    input_state1 = state1.add_input_data_port("input_data_port1", "float")
    output_state1 = state1.add_output_data_port("output_data_port1", "float")

    state2 = ExecutionState("SecondState", "id_of_state_2", path=rafcon.__path__[0] + "/../test_scripts",
                            filename="concurrency_barrier2.py")
    state2.add_outcome("FirstOutcomeState2", 3)
    state2.add_outcome("SecondOutcomeState2", 4)
    input_state2 = state2.add_input_data_port("input_data_port1", "float")
    output_state2 = state2.add_output_data_port("output_data_port1", "float")

    barrier_state = BarrierConcurrencyState("FirstConcurrencyState", "barrier_state_id",
                                            path=rafcon.__path__[0] + "/../test_scripts",
                                            filename="concurrency_container.py")
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
    barrier_state.states[UNIQUE_DECIDER_STATE_ID].script = Script(path=rafcon.__path__[0] + "/../test_scripts",
                                                                  filename="decider_state.py",
                                                                  script_type=ScriptType.EXECUTION, check_path=True,
                                                                  state=barrier_state.states[UNIQUE_DECIDER_STATE_ID])
    barrier_state.states[UNIQUE_DECIDER_STATE_ID].add_outcome("FirstOutcomeDecider", 3)
    barrier_state.states[UNIQUE_DECIDER_STATE_ID].add_outcome("SecondOutcomeDecider", 4)

    barrier_state.add_transition(barrier_state.states[UNIQUE_DECIDER_STATE_ID].state_id, 3, barrier_state.state_id, 3)
    barrier_state.add_transition(barrier_state.states[UNIQUE_DECIDER_STATE_ID].state_id, 4, barrier_state.state_id, 4)

    return barrier_state


def test_concurrency_barrier_save_load(caplog):
    concurrency_barrier_state = create_concurrency_barrier_state()

    state_machine = StateMachine(concurrency_barrier_state)
    test_storage = StateMachineStorage(rafcon.__path__[0] + "/../test_scripts/decider_test_statemachine")
    # test_storage.save_statemachine_as_yaml(state_machine, "../test_scripts/decider_test_statemachine")
    test_storage.save_statemachine_as_yaml(state_machine, test_utils.TMP_TEST_PATH + "/decider_test_statemachine")
    sm_loaded, version, creation_time = test_storage.load_statemachine_from_yaml(test_utils.TMP_TEST_PATH +
                                                                                 "/decider_test_statemachine")

    root_state = sm_loaded.root_state
    input_data = {"input_data_port1": 0.1, "input_data_port2": 0.1}
    output_data = {"output_data_port1": None}
    root_state.input_data = input_data
    root_state.output_data = output_data

    state_machine = StateMachine(root_state)
    test_utils.test_multithrading_lock.acquire()
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    rafcon.statemachine.singleton.state_machine_execution_engine.start()
    root_state.join()
    rafcon.statemachine.singleton.state_machine_execution_engine.stop()

    assert rafcon.statemachine.singleton.global_variable_manager.get_variable("var_x") == 10
    assert rafcon.statemachine.singleton.global_variable_manager.get_variable("var_y") == 20
    assert root_state.final_outcome.outcome_id == 4

    rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    test_utils.assert_logger_warnings_and_errors(caplog, 0, 1)
    test_utils.test_multithrading_lock.release()

if __name__ == '__main__':
    pytest.main([__file__])