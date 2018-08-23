import os
import pytest

# state machine
import rafcon.core.start
import rafcon.core.singleton
from rafcon.core.storage import storage

import testing_utils


def test_execute_script_returns_none(caplog):
    testing_utils.initialize_environment_core()

    state_machine_path = testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "return_none_test_sm"))

    state_machine = storage.load_state_machine_from_path(state_machine_path)
    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id

    state_machine = rafcon.core.singleton.state_machine_manager.get_active_state_machine()
    assert state_machine.file_system_path == state_machine_path

    rafcon.core.singleton.state_machine_execution_engine.start()
    rafcon.core.singleton.state_machine_execution_engine.join()

    try:
        assert state_machine.root_state.final_outcome.outcome_id == 0
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog, expected_warnings=0, expected_errors=1)


if __name__ == '__main__':
    pytest.main([__file__])
