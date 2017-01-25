import os

# gui elements
import rafcon.gui.config as gui_config
import rafcon.gui.singleton

# core elements
import rafcon.core.config
import rafcon.core.singleton as singletons

# general tool elements
from rafcon.utils import log
import testing_utils
from testing_utils import call_gui_callback, run_gui, wait_for_gui_quit

logger = log.get_logger(__name__)


def run_state_machines(state_machines_path):
    from rafcon.gui.singleton import main_window_controller
    gvm = rafcon.core.singleton.global_variable_manager
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
    execution_engine = singletons.state_machine_execution_engine
    state_machine_manager = singletons.state_machine_manager

    for state_machine_folder in os.listdir(state_machines_path):
        state_machine_path = os.path.join(state_machines_path, state_machine_folder)
        if not os.path.isdir(state_machine_path):
            continue

        if not execution_engine.finished_or_stopped():
            raise RuntimeError("The execution engine is not stopped")

        print "Loading state machine from path: {}".format(state_machine_path)
        call_gui_callback(menubar_ctrl.on_open_activate, None, None, state_machine_path)

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
        run_state_machines(path)
    finally:
        from rafcon.gui.singleton import main_window_controller
        menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
        call_gui_callback(menubar_ctrl.on_quit_activate, None, None, True)

    wait_for_gui_quit()
    logger.debug("after gtk main")

    testing_utils.remove_all_libraries()
    testing_utils.test_multithreading_lock.release()
    testing_utils.assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    test_backward_compatibility_storage(None)
    # pytest.main(['-s', __file__])