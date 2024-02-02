import os
import threading
# core elements
from rafcon.core.states.state import StateExecutionStatus

from rafcon.core.storage import storage
import rafcon.core.singleton
from rafcon.core.singleton import state_machine_execution_engine
from tests.utils import wait_for_execution_engine_sync_counter
from rafcon.utils import log

# test environment elements
from tests import utils as testing_utils

logger = log.get_logger(__name__)


def test_run_this_state(caplog):
    # run_selected
    # Initialize testing environment
    testing_utils.initialize_environment_core()
    # Load State Machine
    sm = storage.load_state_machine_from_path(testing_utils.get_test_sm_path(os.path.join(
        "unit_test_state_machines", "test_run_this_state")))
    rafcon.core.singleton.state_machine_manager.add_state_machine(sm)
    # Run selected state machine
    rafcon.core.singleton.global_variable_manager.set_variable("test_value", 1)
    state_machine_execution_engine.run_selected_state("BTWFZQ/EPQSTG", sm.state_machine_id)
    wait_for_execution_engine_sync_counter(1, logger)
    state_machine_execution_engine.stop()
    rafcon.core.singleton.state_machine_execution_engine.join()
    # assert variable state
    try:
        assert rafcon.core.singleton.global_variable_manager.get_variable("test_value") == 2
    # Shutdown testing environment
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)


if __name__ == '__main__':
    test_run_this_state(None)
