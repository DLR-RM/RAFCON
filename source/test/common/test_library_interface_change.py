import os
from os.path import dirname, join, abspath

# core elements
import rafcon.core.singleton
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.states.library_state import LibraryState
from rafcon.core.states.state import DataPortType
from rafcon.core.storage import storage
from rafcon.core.state_machine import StateMachine
from rafcon.core.config import global_config

# test environment elements
import testing_utils
import pytest

from rafcon.utils import log
logger = log.get_logger("start-no-gui")


def setup_module():
    # set the test_libraries path temporarily to the correct value
    testing_utils.remove_all_libraries()
    library_paths = global_config.get_config_value("LIBRARY_PATHS")
    library_paths["unit_test_state_machines"] = os.path.join(testing_utils.TEST_SM_PATH, "unit_test_state_machines")
    global_config.set_config_value("LIBRARY_RECOVERY_MODE", True)


def get_test_state_machine(name):
    return storage.load_state_machine_from_path(
        join(
            join(
                join(
                    testing_utils.TEST_SM_PATH,
                    "unit_test_state_machines"),
                "faulty_libraries"),
        name))


def test_load_data_ports_not_existing(caplog):
    testing_utils.test_multithreading_lock.acquire()

    rafcon.core.singleton.state_machine_manager.delete_all_state_machines()
    rafcon.core.singleton.library_manager.initialize()
    state_machine = get_test_state_machine("data_ports_not_existing")
    # state_machine = get_test_state_machine("correct_library_inclusion")

    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)

    rafcon.core.singleton.state_machine_execution_engine.start()
    rafcon.core.singleton.state_machine_execution_engine.join()
    rafcon.core.singleton.state_machine_execution_engine.stop()

    assert rafcon.core.singleton.global_variable_manager.get_variable("x") == 1

    testing_utils.assert_logger_warnings_and_errors(caplog, 0, 2)
    testing_utils.reload_config(config=True, gui_config=False)
    testing_utils.test_multithreading_lock.release()

    logger.info("State machine execution finished!")


def test_load_wrong_data_types(caplog):
    testing_utils.test_multithreading_lock.acquire()

    rafcon.core.singleton.state_machine_manager.delete_all_state_machines()
    rafcon.core.singleton.library_manager.initialize()
    state_machine = get_test_state_machine("wrong_data_types")

    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)

    rafcon.core.singleton.state_machine_execution_engine.start()
    rafcon.core.singleton.state_machine_execution_engine.join()
    rafcon.core.singleton.state_machine_execution_engine.stop()

    assert rafcon.core.singleton.global_variable_manager.get_variable("x") == 1

    testing_utils.assert_logger_warnings_and_errors(caplog, 0, 2)
    testing_utils.reload_config(config=True, gui_config=False)
    testing_utils.test_multithreading_lock.release()

    logger.info("State machine execution finished!")


def test_load_not_existing_outcome(caplog):
    testing_utils.test_multithreading_lock.acquire()

    rafcon.core.singleton.state_machine_manager.delete_all_state_machines()
    rafcon.core.singleton.library_manager.initialize()
    state_machine = get_test_state_machine("outcome_not_existing")
    # state_machine = get_test_state_machine("correct_library_inclusion")

    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)

    testing_utils.assert_logger_warnings_and_errors(caplog, 0, 1)
    testing_utils.reload_config(config=True, gui_config=False)
    testing_utils.test_multithreading_lock.release()

    logger.info("State machine execution finished!")


if __name__ == '__main__':
    setup_module()
    test_load_not_existing_outcome(None)
    # test_load_wrong_data_types(None)
    # test_load_data_ports_not_existing(None)
    # pytest.main(['-s', __file__])
