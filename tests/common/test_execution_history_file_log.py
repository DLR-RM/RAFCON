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

import os
from rafcon.core.storage import storage as global_storage


def test_execution_log(caplog):
    try:
        testing_utils.initialize_environment(
            core_config={'EXECUTION_LOG_ENABLE': True,
                         'EXECUTION_LOG_PATH': testing_utils.get_unique_temp_path()+'/test_execution_log'})

        state_machine = global_storage.load_state_machine_from_path(
            testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines",
                                                        "execution_file_log_test")))

        rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
        rafcon.core.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
        rafcon.core.singleton.state_machine_execution_engine.start()
        rafcon.core.singleton.state_machine_execution_engine.join()

        import shelve
        import json
        ss = shelve.open(state_machine.get_last_execution_log_filename())

        assert len(ss) == 53

        rafcon.core.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    finally:
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=0, expected_errors=4)

if __name__ == '__main__':
    pytest.main([__file__])
