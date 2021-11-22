import os
import time

# core elements
from rafcon.core.storage import storage
import rafcon.core.singleton
from rafcon.core.singleton import state_machine_execution_engine
from rafcon.utils import log

# test environment elements
from tests import utils as testing_utils
from tests.utils import wait_for_execution_engine_sync_counter

logger = log.get_logger(__name__)


def execute_command_synchronized_on_state(state_machine, state_id, command, counter=1, join=True):
    old_run_id = state_machine.get_state_by_path(state_id).run_id
    old_execution_counter = state_machine.get_state_by_path(state_id)._execution_counter
    getattr(rafcon.core.singleton.state_machine_execution_engine, command)()
    # let the state start properly
    while old_run_id == state_machine.get_state_by_path(state_id).run_id:
        time.sleep(0.005)
    while old_execution_counter == state_machine.get_state_by_path(state_id)._execution_counter:
        time.sleep(0.005)
    if join:
        try:
            state_machine.get_state_by_path(state_id).join()
        except RuntimeError:
            # if the state is already executed then join() returns with an RuntimeError
            pass
    # let the hierarchy properly chose the next state
    wait_for_execution_engine_sync_counter(counter, logger)


def test_run_this_state(caplog):
    # run_selected
    # Initialize testing environment
    testing_utils.initialize_environment_core()
    # Load State Machine
    state_machine = storage.load_state_machine_from_path(testing_utils.get_test_sm_path(os.path.join(
        "unit_test_state_machines", "test_run_this_state")))
    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)

    # Run selected state machine (NDIVLD)
    execute_command_synchronized_on_state(state_machine, "BTWFZQ/EPQSTG", "run_selected", 1)
    # Stop execution engine
    rafcon.core.singleton.state_machine_execution_engine.stop()
    # assert variable state
    try:
        assert rafcon.core.singleton.global_variable_manager.get_variable("test_value") == 2
    # Shutdown testing environment
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)


if __name__ == '__main__':
    test_run_this_state(None)
