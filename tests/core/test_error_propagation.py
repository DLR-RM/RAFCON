import os
import pytest


# singleton elements
from rafcon.core.singleton import state_machine_manager, state_machine_execution_engine

from tests import utils as testing_utils


@pytest.mark.parametrize("recompile", [True, False])
def test_error_propagation(caplog, recompile):
    testing_utils.initialize_environment_core({"SCRIPT_RECOMPILATION_ON_STATE_EXECUTION": recompile})

    sm = state_machine_execution_engine.execute_state_machine_from_path(
        path=testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "error_propagation_test")))
    state_machine_manager.remove_state_machine(sm.state_machine_id)
    try:
        assert sm.root_state.output_data["error_check"] == "successfull"
    finally:
        if recompile:
            testing_utils.shutdown_environment_only_core(caplog=caplog, expected_warnings=0, expected_errors=2)
        else:
            testing_utils.shutdown_environment_only_core(caplog=caplog, expected_warnings=0, expected_errors=3)


if __name__ == '__main__':
    test_error_propagation(None)
    # pytest.main([__file__])
