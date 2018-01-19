import os
import time

# state machine
import rafcon.core.singleton
from rafcon.core.storage import storage
from rafcon.core.states.state import StateExecutionStatus

# utils
from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE
from rafcon.utils import log

# test environment elements
import testing_utils

logger = log.get_logger(__name__)


def test_run_to_selected_state(caplog):
    testing_utils.initialize_environment_core()

    sm_path = testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "run_to_selected_state_test"))
    sm = storage.load_state_machine_from_path(sm_path)
    # select state machine for this purpose
    rafcon.core.singleton.state_machine_manager.add_state_machine(sm)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = sm.state_machine_id
    rafcon.core.singleton.state_machine_execution_engine.run_to_selected_state("VVBPOY/AOZXRY",
                                                                                       sm.state_machine_id)
    # run the statemachine to the state before AOYXRY, this is an asynchronous task
    timeout = time.time()
    while not sm.get_state_by_path("VVBPOY/ABNQFK").state_execution_status is StateExecutionStatus.WAIT_FOR_NEXT_STATE:
        time.sleep(.05)
        if time.time()-timeout > 2:
            raise RuntimeError("execution_state ABNQFK not reached --> timeout")
    # wait until the statemachine is executed until ABNQFK the state before AOZXRY, so it doesnt check for the file
    # before its even written

    with open(os.path.join(RAFCON_TEMP_PATH_BASE, 'test_file'), 'r') as test_file:
        lines = test_file.readlines()

    # the state machines waits at ABNQFK with state WAIT_FOR_NEXT_STATE, so it needs to be stopped manually
    rafcon.core.singleton.state_machine_execution_engine.stop()
    rafcon.core.singleton.state_machine_execution_engine.join()

    try:
        assert len(lines) < 3
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)


if __name__ == '__main__':
    test_run_to_selected_state(None)
    # pytest.main([__file__])
