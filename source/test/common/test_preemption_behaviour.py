import threading
import time

# core elements
from rafcon.statemachine.execution.state_machine_execution_engine import StateMachineExecutionEngine
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState
# import to make reload_config() working
import rafcon.mvc.singleton
from rafcon.statemachine.singleton import global_variable_manager

# singleton elements
import rafcon.statemachine.singleton
from rafcon.statemachine.storage import storage

# test environment elements
import testing_utils
import pytest


def test_preemption_behaviour_in_preemption_state(caplog):
    testing_utils.remove_all_libraries()

    testing_utils.test_multithrading_lock.acquire()
    rafcon.statemachine.singleton.state_machine_manager.delete_all_state_machines()

    sm = StateMachineExecutionEngine.execute_state_machine_from_path(testing_utils.get_test_sm_path("unit_test_state_machines/preemption_behaviour_test_sm"))
    rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(sm.state_machine_id)
    from rafcon.statemachine.singleton import global_variable_manager
    assert global_variable_manager.get_variable("s2") == 1.0
    assert not global_variable_manager.variable_exist("s3")

    testing_utils.reload_config()
    testing_utils.test_multithrading_lock.release()
    testing_utils.assert_logger_warnings_and_errors(caplog)


def trigger_stop(sm, execution_engine):
    while not global_variable_manager.variable_exists("s1"):
        time.sleep(0.1)
    execution_engine.stop()


def test_preemption_behaviour_during_stop(caplog):
    testing_utils.remove_all_libraries()

    testing_utils.test_multithrading_lock.acquire()
    rafcon.statemachine.singleton.state_machine_manager.delete_all_state_machines()

    state_machine = storage.load_state_machine_from_path(testing_utils.get_test_sm_path(
        "unit_test_state_machines/preemption_behaviour_during_stop"))
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)

    thread = threading.Thread(target=trigger_stop, args=[state_machine,
                                                         rafcon.statemachine.singleton.state_machine_execution_engine])
    thread.start()

    rafcon.statemachine.singleton.state_machine_execution_engine.start()
    rafcon.statemachine.singleton.state_machine_execution_engine.join()

    rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    assert global_variable_manager.get_variable("s1") == 1
    assert global_variable_manager.get_variable("s2") == 1
    assert not global_variable_manager.variable_exist("s3")

    testing_utils.reload_config()
    testing_utils.test_multithrading_lock.release()
    testing_utils.assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    test_preemption_behaviour_in_preemption_state(None)
    test_preemption_behaviour_during_stop(None)
    # pytest.main([__file__])