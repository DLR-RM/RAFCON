# test environment elements
import test_utils
from test_utils import TEST_SM_PATH
import pytest
import rafcon

# core elements
from rafcon.statemachine.execution.statemachine_execution_engine import StatemachineExecutionEngine
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.library_state import LibraryState

# singleton elements
from rafcon.statemachine.singleton import state_machine_manager
from rafcon.utils import log
logger = log.get_logger(__name__)


def setup_module(module=None):
    # set the test_libraries path temporarily to the correct value
    test_utils.remove_all_libraries()
    library_paths = rafcon.statemachine.config.global_config.get_config_value("LIBRARY_PATHS")
    library_paths["unit_test_state_machines"] = test_utils.get_test_sm_path("unit_test_state_machines")
    logger.debug(library_paths["unit_test_state_machines"])


def test_runtime_values(caplog):
    state_machine_manager.delete_all_state_machines()
    test_utils.test_multithrading_lock.acquire()

    sm = StatemachineExecutionEngine.execute_state_machine_from_path(test_utils.get_test_sm_path("unit_test_state_machines/library_runtime_value_test"))
    state_machine_manager.remove_state_machine(sm.state_machine_id)
    assert sm.root_state.output_data["data_output_port1"] == 114

    test_utils.test_multithrading_lock.release()
    test_utils.assert_logger_warnings_and_errors(caplog, 0, 0)


def teardown_module(module=None):
    test_utils.reload_config(gui_config=False)


if __name__ == '__main__':
    # setup_module()
    # test_runtime_values(None)
    pytest.main([__file__])
