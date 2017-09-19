import os
from distutils.version import StrictVersion

# core elements
import rafcon.core.config
import rafcon.core.singleton as singletons

# general tool elements
from rafcon.utils import log
import testing_utils
from testing_utils import call_gui_callback, run_gui, close_gui

logger = log.get_logger(__name__)


def run_backward_compatibility_state_machines(state_machines_path):
    for state_machine_folder in os.listdir(state_machines_path):
        state_machine_path = os.path.join(state_machines_path, state_machine_folder)
        if not os.path.isdir(state_machine_path):
            continue

        run_state_machine(state_machine_path)


def run_state_machine(state_machine_path):
    import rafcon.gui.helpers.state_machine as gui_helper_statemachine
    gvm = rafcon.core.singleton.global_variable_manager
    execution_engine = singletons.state_machine_execution_engine
    state_machine_manager = singletons.state_machine_manager

    if not execution_engine.finished_or_stopped():
        raise RuntimeError("The execution engine is not stopped")

    print "Loading state machine from path: {}".format(state_machine_path)
    call_gui_callback(gui_helper_statemachine.open_state_machine, state_machine_path)

    testing_utils.remove_all_gvm_variables()

    execution_engine.start()
    if not execution_engine.join(3):
        raise RuntimeError("State machine did not finish within the given time")

    assert gvm.get_variable("b1") == 1
    assert gvm.get_variable("b2") == 1
    assert gvm.get_variable("h1") == 1
    assert gvm.get_variable("e1") == 1
    assert gvm.get_variable("l1") == 1

    state_machine_manager.remove_state_machine(state_machine_manager.active_state_machine_id)


def test_backward_compatibility_storage(caplog):
    path = testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "backward_compatibility"))

    run_gui(gui_config={'HISTORY_ENABLED': False,
                        'AUTO_BACKUP_ENABLED': False},
            libraries={'unit_test_state_machines': testing_utils.get_test_sm_path("unit_test_state_machines")})

    try:
        run_backward_compatibility_state_machines(path)
    finally:
        # two warning per minor version lower than the current RAFCON version
        state_machines = len([filename for filename in os.listdir(path) if os.path.isdir(os.path.join(path, filename))])
        close_gui()
        testing_utils.shutdown_environment(caplog=caplog, expected_warnings=(state_machines - 1)*2)


if __name__ == '__main__':
    test_backward_compatibility_storage(None)
    # pytest.main(['-s', __file__])
