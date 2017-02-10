import threading
import time

# mvc
import rafcon.gui.singleton

# core elements
import rafcon.core.singleton
from rafcon.core.storage import storage
from rafcon.core.singleton import global_variable_manager
from rafcon.core.singleton import state_machine_execution_engine
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState

# test environment elements
import testing_utils
import pytest


def test_preemption_behaviour_in_preemption_state(caplog):
    testing_utils.initialize_environment()

    sm = state_machine_execution_engine.execute_state_machine_from_path(
        path=testing_utils.get_test_sm_path("unit_test_state_machines/preemption_behaviour_test_sm"))
    rafcon.core.singleton.state_machine_manager.remove_state_machine(sm.state_machine_id)
    from rafcon.core.singleton import global_variable_manager
    try:
        assert global_variable_manager.get_variable("s2") == 1.0
        assert not global_variable_manager.variable_exist("s3")
    finally:
        testing_utils.shutdown_environment(caplog=caplog)


def trigger_stop(sm, execution_engine):
    while not global_variable_manager.variable_exists("s1"):
        time.sleep(0.1)
    execution_engine.stop()


def test_preemption_behaviour_during_stop(caplog):
    testing_utils.initialize_environment()

    state_machine = storage.load_state_machine_from_path(testing_utils.get_test_sm_path(
        "unit_test_state_machines/preemption_behaviour_during_stop"))
    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)

    thread = threading.Thread(target=trigger_stop, args=[state_machine,
                                                         rafcon.core.singleton.state_machine_execution_engine])
    thread.start()

    rafcon.core.singleton.state_machine_execution_engine.start()
    rafcon.core.singleton.state_machine_execution_engine.join()

    rafcon.core.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    try:
        assert global_variable_manager.get_variable("s1") == 1
        assert global_variable_manager.get_variable("s2") == 1
        assert not global_variable_manager.variable_exist("s3")
    finally:
        testing_utils.shutdown_environment(caplog=caplog)


if __name__ == '__main__':
    test_preemption_behaviour_in_preemption_state(None)
    test_preemption_behaviour_during_stop(None)
    # pytest.main([__file__])