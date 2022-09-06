import os
import time

# singleton elements
import rafcon.core.singleton
from rafcon.core.storage import storage as global_storage
import rafcon.utils.execution_log as log_helper

# testing imports
from tests import utils as testing_utils


def test_execution_log(caplog):

    testing_utils.initialize_environment_core(
        core_config={
            'IN_MEMORY_EXECUTION_HISTORY_ENABLE': False,
            'FILE_SYSTEM_EXECUTION_HISTORY_ENABLE': False,
            'EXECUTION_LOG_PATH': testing_utils.get_unique_temp_path()+'/test_execution_log'}
    )

    # this state machine features:
    # * hierarchies
    # * barrier concurrency
    # * execution states
    # * data flows
    # * logic flows
    state_machine = global_storage.load_state_machine_from_path(
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines",
                                                    "execution_file_log_test")))

    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.core.singleton.state_machine_execution_engine.start(state_machine.state_machine_id)
    while not state_machine.root_state.final_outcome:
        time.sleep(0.1)
    rafcon.core.singleton.state_machine_execution_engine.join()

    rafcon.core.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)

    # the test assertions are that there are no errors/warnings
    testing_utils.shutdown_environment_only_core(caplog=caplog, expected_warnings=0, expected_errors=0)


if __name__ == '__main__':
    test_execution_log(None)
    # pytest.main([__file__])
