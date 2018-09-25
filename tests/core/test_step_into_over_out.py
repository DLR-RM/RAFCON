import os
import time

# core elements
from rafcon.core.storage import storage
import rafcon.core.singleton
from rafcon.core.singleton import state_machine_execution_engine
from rafcon.utils import log

# test environment elements
import testing_utils
from testing_utils import wait_for_execution_engine_sync_counter

logger = log.get_logger(__name__)


def wait_and_join(state_machine, state_id):
    time.sleep(0.05)  # let the state start properly
    state_machine.get_state_by_path(state_id).join()
    time.sleep(0.05)  # let the hierarchy properly chose the next state


def test_custom_entry_point(caplog):

    testing_utils.initialize_environment_core()

    state_machine = storage.load_state_machine_from_path(
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "stepping_test")))

    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id

    rafcon.core.singleton.state_machine_execution_engine.step_mode()
    time.sleep(0.2)  # let the state machine start properly

    # sm structure

    # GLSUJY
    # GLSUJY/PXTKIH
    # GLSUJY/NDIVLD
    # GLSUJY/SFZGMH

    # GLSUJY/SMCOIB
    # GLSUJY/SMCOIB/YSBJGK
    # GLSUJY/SMCOIB/OUWQUJ
    # GLSUJY/SMCOIB/UGGFFI

    rafcon.core.singleton.state_machine_execution_engine.step_over()
    wait_and_join(state_machine, "GLSUJY/PXTKIH")
    rafcon.core.singleton.state_machine_execution_engine.step_over()
    wait_and_join(state_machine, "GLSUJY/NDIVLD")
    rafcon.core.singleton.state_machine_execution_engine.step_over()
    wait_and_join(state_machine, "GLSUJY/SFZGMH")
    rafcon.core.singleton.state_machine_execution_engine.step_over()
    wait_and_join(state_machine, "GLSUJY/SMCOIB")

    rafcon.core.singleton.state_machine_execution_engine.step_into()
    wait_and_join(state_machine, "GLSUJY/PXTKIH")
    rafcon.core.singleton.state_machine_execution_engine.step_into()
    wait_and_join(state_machine, "GLSUJY/NDIVLD")
    rafcon.core.singleton.state_machine_execution_engine.step_into()
    wait_and_join(state_machine, "GLSUJY/SFZGMH")
    rafcon.core.singleton.state_machine_execution_engine.step_into() # step into hierarchy state GLSUJY/SMCOIB
    rafcon.core.singleton.state_machine_execution_engine.step_into()
    wait_and_join(state_machine, "GLSUJY/SMCOIB/YSBJGK")

    rafcon.core.singleton.state_machine_execution_engine.step_out()
    wait_and_join(state_machine, "GLSUJY/SMCOIB")

    rafcon.core.singleton.state_machine_execution_engine.stop()
    rafcon.core.singleton.state_machine_execution_engine.join()

    try:
        assert rafcon.core.singleton.global_variable_manager.get_variable("bottles") == 95
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)


def test_step_through_library(caplog):

    testing_utils.initialize_environment_core()

    testing_utils.rewind_and_set_libraries({"unit_test_state_machines":
                                            os.path.join(testing_utils.TEST_ASSETS_PATH, "unit_test_state_machines")})

    state_machine = storage.load_state_machine_from_path(
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "stepping_test_with_library")))

    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id

    state_machine_execution_engine.synchronization_lock.acquire()
    state_machine_execution_engine.synchronization_counter = 0
    state_machine_execution_engine.synchronization_lock.release()

    rafcon.core.singleton.state_machine_execution_engine.step_mode()
    wait_for_execution_engine_sync_counter(1, logger)

    # step till library
    rafcon.core.singleton.state_machine_execution_engine.step_over()
    wait_for_execution_engine_sync_counter(1, logger)

    rafcon.core.singleton.state_machine_execution_engine.step_over()
    wait_for_execution_engine_sync_counter(1, logger)

    rafcon.core.singleton.state_machine_execution_engine.step_over()
    wait_for_execution_engine_sync_counter(1, logger)

    # step into library and step out
    rafcon.core.singleton.state_machine_execution_engine.step_into()
    wait_for_execution_engine_sync_counter(1, logger)

    rafcon.core.singleton.state_machine_execution_engine.step_out()
    wait_for_execution_engine_sync_counter(2, logger)

    # step till library
    rafcon.core.singleton.state_machine_execution_engine.step_over()
    wait_for_execution_engine_sync_counter(1, logger)

    rafcon.core.singleton.state_machine_execution_engine.step_over()
    wait_for_execution_engine_sync_counter(1, logger)

    rafcon.core.singleton.state_machine_execution_engine.step_over()
    wait_for_execution_engine_sync_counter(1, logger)

    # step into library and step over library children
    rafcon.core.singleton.state_machine_execution_engine.step_into()
    wait_for_execution_engine_sync_counter(1, logger)

    rafcon.core.singleton.state_machine_execution_engine.step_over()
    wait_for_execution_engine_sync_counter(1, logger)

    # step over last library item
    rafcon.core.singleton.state_machine_execution_engine.step_over()
    wait_for_execution_engine_sync_counter(1, logger)

    rafcon.core.singleton.state_machine_execution_engine.stop()
    rafcon.core.singleton.state_machine_execution_engine.join()

    try:
        from rafcon.core.state_elements.scope import ScopedVariable
        for s in list(state_machine.root_state.scoped_data.values()):
            if s.name == 'bottles' and s.data_port_type == ScopedVariable:
                assert s.value == 4
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)


if __name__ == '__main__':
    test_custom_entry_point(None)
    test_step_through_library(None)
    # pytest.main([__file__])
