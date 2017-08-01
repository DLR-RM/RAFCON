import os
import pytest

# mvc
import rafcon.gui.singleton

# statemachine
import rafcon.core.start
import rafcon.core.singleton
from rafcon.core.storage import storage

import testing_utils


def test_execute_script_returns_none(caplog):
    testing_utils.initialize_environment()

    state_machine = storage.load_state_machine_from_path(
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "return_none_test_sm")))

    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)

    # load the meta data for the state machine
    rafcon.gui.singleton.state_machine_manager_model.get_selected_state_machine_model().root_state.load_meta_data()

    rafcon.core.singleton.state_machine_execution_engine.start()
    rafcon.core.singleton.state_machine_execution_engine.join()

    try:
        assert state_machine.root_state.final_outcome.outcome_id == 0
    finally:
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=0, expected_errors=1)


if __name__ == '__main__':
    pytest.main([__file__])
