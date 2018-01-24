import os
from os.path import join

# core elements
import rafcon.core.singleton
from rafcon.core.storage import storage
from rafcon.core.config import global_config

# test environment elements
import testing_utils
import pytest

from rafcon.utils import log
logger = log.get_logger("start-no-gui")


def setup_function(_):
    # set the test_libraries path temporarily to the correct value
    global_config.set_config_value("LIBRARY_RECOVERY_MODE", True)
    testing_utils.rewind_and_set_libraries({"unit_test_state_machines":
                                            join(testing_utils.TEST_ASSETS_PATH, "unit_test_state_machines")})


def get_test_state_machine(name):
    return storage.load_state_machine_from_path(testing_utils.get_test_sm_path(join("unit_test_state_machines",
                                                                                    "faulty_libraries", name)))


def test_load_data_ports_not_existing(caplog):
    testing_utils.test_multithreading_lock.acquire()

    state_machine = get_test_state_machine("data_ports_not_existing")
    # state_machine = get_test_state_machine("correct_library_inclusion")

    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id

    rafcon.core.singleton.state_machine_execution_engine.start()
    rafcon.core.singleton.state_machine_execution_engine.join()
    rafcon.core.singleton.state_machine_execution_engine.stop()

    try:
        assert rafcon.core.singleton.global_variable_manager.get_variable("x") == 1
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog, expected_warnings=0, expected_errors=2)

    logger.info("State machine execution finished!")


def test_load_wrong_data_types(caplog):
    testing_utils.test_multithreading_lock.acquire()

    rafcon.core.singleton.state_machine_manager.delete_all_state_machines()
    rafcon.core.singleton.library_manager.initialize()
    state_machine = get_test_state_machine("wrong_data_types")

    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id

    rafcon.core.singleton.state_machine_execution_engine.start()
    rafcon.core.singleton.state_machine_execution_engine.join()
    rafcon.core.singleton.state_machine_execution_engine.stop()

    try:
        assert rafcon.core.singleton.global_variable_manager.get_variable("x") == 1

        # 4 data type errors -> 2 data flow port to port data type inequality and while runtime 1 input- and 1 output data type error
        # 2 data type warnings -> 1 input- and  1 output-data port  data type warnings while loading of state machine
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog, expected_warnings=2, expected_errors=4)

    logger.info("State machine execution finished!")


def test_load_not_existing_outcome(caplog):
    testing_utils.test_multithreading_lock.acquire()

    rafcon.core.singleton.state_machine_manager.delete_all_state_machines()
    rafcon.core.singleton.library_manager.initialize()
    state_machine = get_test_state_machine("outcome_not_existing")
    # state_machine = get_test_state_machine("correct_library_inclusion")

    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id

    testing_utils.shutdown_environment_only_core(caplog=caplog, expected_warnings=0, expected_errors=1)

    logger.info("State machine execution finished!")


if __name__ == '__main__':
    test_load_not_existing_outcome(None)
    # test_load_wrong_data_types(None)
    # test_load_data_ports_not_existing(None)
    # pytest.main(['-s', __file__])
