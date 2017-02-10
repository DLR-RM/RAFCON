import pytest

# mvc
import rafcon.gui.singleton

# core elements
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState

# singleton elements
from rafcon.core.singleton import state_machine_manager

import testing_utils

def test_error_propagation(caplog):
    testing_utils.initialize_environment()

    sm = rafcon.core.singleton.state_machine_execution_engine.execute_state_machine_from_path(
        path=testing_utils.get_test_sm_path("unit_test_state_machines/error_propagation_test"))
    state_machine_manager.remove_state_machine(sm.state_machine_id)
    try:
        assert sm.root_state.output_data["error_check"] == "successfull"
    finally:
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=0, expected_errors=2)


if __name__ == '__main__':
    # test_error_propagation(None)
    pytest.main([__file__])
