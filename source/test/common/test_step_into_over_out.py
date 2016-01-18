import time

# core elements
from rafcon.statemachine.execution.statemachine_execution_engine import StatemachineExecutionEngine
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState

# singleton elements
import rafcon.statemachine.singleton

# test environment elements
import utils
import pytest


def wait_and_join(state_machine, state_id):
    time.sleep(0.05)  # let the state start properly
    state_machine.get_state_by_path(state_id).join()
    time.sleep(0.05)  # let the hierarchy properly chose the next state


def test_custom_entry_point(caplog):

    utils.test_multithrading_lock.acquire()

    utils.remove_all_libraries()
    rafcon.statemachine.singleton.state_machine_manager.delete_all_state_machines()
    rafcon.statemachine.singleton.library_manager.initialize()

    [state_machine, version, creation_time] = rafcon.statemachine.singleton.global_storage.load_statemachine_from_path(
        utils.get_test_sm_path("unit_test_state_machines/stepping_test"))

    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)

    rafcon.statemachine.singleton.state_machine_execution_engine.step_mode()
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

    rafcon.statemachine.singleton.state_machine_execution_engine.step_over()
    wait_and_join(state_machine, "GLSUJY/PXTKIH")
    rafcon.statemachine.singleton.state_machine_execution_engine.step_over()
    wait_and_join(state_machine, "GLSUJY/NDIVLD")
    rafcon.statemachine.singleton.state_machine_execution_engine.step_over()
    wait_and_join(state_machine, "GLSUJY/SFZGMH")
    rafcon.statemachine.singleton.state_machine_execution_engine.step_over()
    wait_and_join(state_machine, "GLSUJY/SMCOIB")

    rafcon.statemachine.singleton.state_machine_execution_engine.step_into()
    wait_and_join(state_machine, "GLSUJY/PXTKIH")
    rafcon.statemachine.singleton.state_machine_execution_engine.step_into()
    wait_and_join(state_machine, "GLSUJY/NDIVLD")
    rafcon.statemachine.singleton.state_machine_execution_engine.step_into()
    wait_and_join(state_machine, "GLSUJY/SFZGMH")
    rafcon.statemachine.singleton.state_machine_execution_engine.step_into() # step into hierarchy state GLSUJY/SMCOIB
    rafcon.statemachine.singleton.state_machine_execution_engine.step_into()
    wait_and_join(state_machine, "GLSUJY/SMCOIB/YSBJGK")

    rafcon.statemachine.singleton.state_machine_execution_engine.step_out()
    wait_and_join(state_machine, "GLSUJY/SMCOIB")

    rafcon.statemachine.singleton.state_machine_execution_engine.stop()
    rafcon.statemachine.singleton.state_machine_execution_engine.join()

    assert rafcon.statemachine.singleton.global_variable_manager.get_variable("bottles") == 95

    utils.test_multithrading_lock.release()
    utils.assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    # test_custom_entry_point(None)
    pytest.main([__file__])
