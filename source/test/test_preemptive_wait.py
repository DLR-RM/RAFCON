import pytest
from pytest import raises

from awesome_tool.statemachine.states.execution_state import ExecutionState
from awesome_tool.statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState
import awesome_tool.statemachine.singleton
from awesome_tool.statemachine.singleton import global_variable_manager as gvm
from awesome_tool.statemachine.state_machine import StateMachine
import variables_for_pytest


def create_preemptive_wait_statemachine():
    state1 = ExecutionState("state_1", path="../test_scripts", filename="preemptive_wait_test.py")
    state1.add_outcome("FirstOutcome", 3)

    state2 = ExecutionState("state_2", path="../test_scripts", filename="preemptive_wait_test.py")
    state2.add_outcome("FirstOutcome", 3)

    ctr_state = PreemptiveConcurrencyState("FirstConcurrencyState", path="../test_scripts",
                                        filename="concurrency_container.py")
    ctr_state.add_state(state1)
    ctr_state.add_state(state2)
    ctr_state.add_outcome("end", 3)
    ctr_state.add_transition(state1.state_id, 3, None, 3)
    ctr_state.add_transition(state2.state_id, 3, None, 3)

    return StateMachine(ctr_state)


def test_preemptive_wait_daemon():
    variables_for_pytest.test_multithrading_lock.acquire()

    gvm.set_variable('state_1_wait', 0.5)
    gvm.set_variable('state_2_wait', None)

    run_statemachine()

    assert 0.5 < gvm.get_variable('state_1_wait_time') < 0.55
    assert 0.5 < gvm.get_variable('state_2_wait_time') < 0.55
    assert not gvm.get_variable('state_1_preempted')
    assert gvm.get_variable('state_2_preempted')

    variables_for_pytest.test_multithrading_lock.release()


def run_statemachine():

    preemption_state_sm = create_preemptive_wait_statemachine()
    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(preemption_state_sm)
    awesome_tool.statemachine.singleton.state_machine_manager.active_state_machine_id = preemption_state_sm.state_machine_id
    awesome_tool.statemachine.singleton.state_machine_execution_engine.start()
    preemption_state_sm.root_state.join()
    awesome_tool.statemachine.singleton.state_machine_execution_engine.stop()
    awesome_tool.statemachine.singleton.state_machine_manager.remove_state_machine(preemption_state_sm.state_machine_id)


def test_preemptive_wait_timeout():
    variables_for_pytest.test_multithrading_lock.acquire()

    gvm.set_variable('state_1_wait', 0.5)
    gvm.set_variable('state_2_wait', 1.)

    run_statemachine()

    assert 0.5 < gvm.get_variable('state_1_wait_time') < 0.55
    assert 0.5 < gvm.get_variable('state_2_wait_time') < 0.55
    assert not gvm.get_variable('state_1_preempted')
    assert gvm.get_variable('state_2_preempted')

    variables_for_pytest.test_multithrading_lock.release()


def test_preemptive_wait2_timeout():
    variables_for_pytest.test_multithrading_lock.acquire()

    gvm.set_variable('state_2_wait', 0.5)
    gvm.set_variable('state_1_wait', 1.)

    run_statemachine()

    assert 0.5 < gvm.get_variable('state_1_wait_time') < 0.55
    assert 0.5 < gvm.get_variable('state_2_wait_time') < 0.55
    assert gvm.get_variable('state_1_preempted')
    assert not gvm.get_variable('state_2_preempted')

    variables_for_pytest.test_multithrading_lock.release()

if __name__ == '__main__':
    test_preemptive_wait_timeout()
    test_preemptive_wait2_timeout()
    test_preemptive_wait_daemon()