import os
# import time

# core elements
from rafcon.core.states.state import StateExecutionStatus

from rafcon.core.storage import storage
import rafcon.core.singleton
from rafcon.core.singleton import state_machine_execution_engine
from rafcon.utils import log

# test environment elements
from tests import utils as testing_utils

logger = log.get_logger(__name__)


# FIXME: Test failing non deterministically
def _test_only_run_this_state(caplog):
    # run_selected
    # Initialize testing environment
    testing_utils.initialize_environment_core()
    # Load State Machine
    sm = storage.load_state_machine_from_path(testing_utils.get_test_sm_path(os.path.join(
        "unit_test_state_machines", "test_run_only_this_state")))
    rafcon.core.singleton.state_machine_manager.add_state_machine(sm)
    # Run only selected state machine
    rafcon.core.singleton.global_variable_manager.set_variable("test_value", 0)
    state_machine_execution_engine.run_only_selected_state("BSSKWR/YEKRMH/TZILCN", sm.state_machine_id)
    sm.join()

    assert rafcon.core.singleton.global_variable_manager.get_variable("test_value") == 1

    rafcon.core.singleton.global_variable_manager.set_variable("test_value_concurrency", 2)
    state_machine_execution_engine.run_only_selected_state("BSSKWR/IQURCQ/LLRMSU/VYGYRO", sm.state_machine_id)
    sm.join()

    # assert variable state
    try:
        assert rafcon.core.singleton.global_variable_manager.get_variable("test_value_concurrency") == 1
    # Shutdown testing environment
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)
