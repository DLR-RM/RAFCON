import os
import time

# core elements
from rafcon.core.storage import storage
import rafcon.core.singleton
from rafcon.core.singleton import state_machine_execution_engine
from rafcon.utils import log

# test environment elements
from tests import utils as testing_utils
from tests.utils import wait_for_execution_engine_sync_counter

logger = log.get_logger(__name__)


def execute_command_synchronized_on_state(state_machine, state_id, command, counter=1, join=True):
    old_run_id = state_machine.get_state_by_path(state_id).run_id
    old_execution_counter = state_machine.get_state_by_path(state_id)._execution_counter
    getattr(rafcon.core.singleton.state_machine_execution_engine, command)()
    # let the state start properly
    while old_run_id == state_machine.get_state_by_path(state_id).run_id:
        time.sleep(0.005)
    while old_execution_counter == state_machine.get_state_by_path(state_id)._execution_counter:
        time.sleep(0.005)
    if join:
        try:
            state_machine.get_state_by_path(state_id).join()
        except RuntimeError:
            # if the state is already executed then join() returns with an RuntimeError
            pass
    # let the hierarchy properly chose the next state
    wait_for_execution_engine_sync_counter(counter, logger)


def test_step_into_over_out_no_library(caplog):

    testing_utils.initialize_environment_core()

    state_machine = storage.load_state_machine_from_path(
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "stepping_test")))

    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)

    with state_machine_execution_engine._status.execution_condition_variable:
        state_machine_execution_engine.synchronization_counter = 0

    rafcon.core.singleton.state_machine_execution_engine.step_mode(state_machine.state_machine_id)
    wait_for_execution_engine_sync_counter(1, logger)

    # sm structure

    # GLSUJY
    # GLSUJY/PXTKIH
    # GLSUJY/NDIVLD
    # GLSUJY/SFZGMH

    # GLSUJY/SMCOIB
    # GLSUJY/SMCOIB/YSBJGK
    # GLSUJY/SMCOIB/OUWQUJ
    # GLSUJY/SMCOIB/UGGFFI

    execute_command_synchronized_on_state(state_machine, "GLSUJY/PXTKIH", "step_over", 1)
    execute_command_synchronized_on_state(state_machine, "GLSUJY/NDIVLD", "step_over", 1)
    execute_command_synchronized_on_state(state_machine, "GLSUJY/SFZGMH", "step_over", 1)
    execute_command_synchronized_on_state(state_machine, "GLSUJY/SMCOIB", "step_over", 1)

    execute_command_synchronized_on_state(state_machine, "GLSUJY/PXTKIH", "step_into")
    execute_command_synchronized_on_state(state_machine, "GLSUJY/NDIVLD", "step_into")
    execute_command_synchronized_on_state(state_machine, "GLSUJY/SFZGMH", "step_into")
    execute_command_synchronized_on_state(state_machine, "GLSUJY/SMCOIB", "step_into", join=False)
    execute_command_synchronized_on_state(state_machine, "GLSUJY/SMCOIB/YSBJGK", "step_into")

    rafcon.core.singleton.state_machine_execution_engine.step_out()
    wait_for_execution_engine_sync_counter(1, logger)

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

    with state_machine_execution_engine._status.execution_condition_variable:
        state_machine_execution_engine.synchronization_counter = 0

    rafcon.core.singleton.state_machine_execution_engine.step_mode(state_machine.state_machine_id)
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
    wait_for_execution_engine_sync_counter(1, logger)

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
        for s in state_machine.root_state.scoped_data.values():
            if s.name == 'bottles' and s.data_port_type == ScopedVariable:
                assert s.value == 4
    finally:
        testing_utils.shutdown_environment_only_core(caplog=caplog)


if __name__ == '__main__':
    test_step_into_over_out_no_library(None)
    test_step_through_library(None)
    # pytest.main([__file__])
