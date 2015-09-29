import pytest
import signal

from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.execution_state import ExecutionState
import rafcon.mvc.singleton
import test_utils


def test_execute_script_returns_none(caplog):
    test_utils.remove_all_libraries()

    test_utils.test_multithrading_lock.acquire()
    rafcon.statemachine.singleton.state_machine_manager.delete_all_state_machines()
    signal.signal(signal.SIGINT, rafcon.statemachine.singleton.signal_handler)

    rafcon.statemachine.singleton.library_manager.initialize()

    [state_machine, version, creation_time] = rafcon.statemachine.singleton.\
        global_storage.load_statemachine_from_yaml(test_utils.get_test_sm_path("return_none_test_sm"))

    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    if test_utils.sm_manager_model is None:
        test_utils.sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model

    # load the meta data for the state machine
    test_utils.sm_manager_model.get_selected_state_machine_model().root_state.load_meta_data_for_state()

    rafcon.statemachine.singleton.state_machine_execution_engine.start()
    state_machine.root_state.join()
    rafcon.statemachine.singleton.state_machine_execution_engine.stop()

    assert state_machine.root_state.final_outcome.outcome_id == 0

    test_utils.assert_logger_warnings_and_errors(caplog, 0, 1)
    test_utils.test_multithrading_lock.release()


if __name__ == '__main__':
    pytest.main([__file__])