import os
import time

# test environment elements
import pytest
from tests import utils as testing_utils
# noinspection PyUnresolvedReferences
from tests.gui.conftest import gui

# general tool elements
from rafcon.utils import log
logger = log.get_logger(__name__)


def initialize_global_variables():
    import rafcon.gui.singleton as gui_singleton
    gui_singleton.global_variable_manager_model.global_variable_manager.set_variable("global_variable_1", "value1")
    gui_singleton.global_variable_manager_model.global_variable_manager.set_variable("global_variable_2", "value2")


def execute_all_generic_libraries_with_keyboard_only(gui):
    menubar_ctrl = gui.singletons.main_window_controller.menu_bar_controller
    gui(
        menubar_ctrl.on_open_activate, None, None,
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "all_generic_libraries"))
    )
    testing_utils.wait_for_gui()

    gui(menubar_ctrl.on_start_activate, None, None)
    import time
    from pykeyboard import PyKeyboard
    time.sleep(0.5)
    k = PyKeyboard()
    k.tap_key('Return', 7, 0.5)
    k.tap_key('Tab', 2, 0.5)
    k.tap_key('Return', 5, 0.5)
    gui(menubar_ctrl.on_stop_activate, None, None)


@pytest.mark.user_input
@pytest.mark.timeout(60)
def test_all_generic_libraries_in_a_row(gui):
    gui(initialize_global_variables)
    with pytest.warns(log.RAFCONDeprecationWarning) as deprecations_warnings:
        execute_all_generic_libraries_with_keyboard_only(gui)
    assert len([record for record in deprecations_warnings if record.category is log.RAFCONDeprecationWarning]) == 4


def test_preemption_of_all_state_machines_at_once(gui):
    gui(initialize_global_variables)
    with pytest.warns(log.RAFCONDeprecationWarning) as deprecations_warnings:
        from rafcon.core.singleton import state_machine_execution_engine, state_machine_manager
        import rafcon.gui.singleton as gui_singleton

        menubar_ctrl = gui_singleton.main_window_controller.get_controller('menu_bar_controller')
        gui(
            menubar_ctrl.on_open_activate, None, None,
            testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "all_dialogs_parallel_preempted"))
        )

        gui(menubar_ctrl.on_start_activate, None, None)
        duration_waited = 0.
        period = 0.1
        while not state_machine_execution_engine.finished_or_stopped():
            time.sleep(period)
            duration_waited += period
            if duration_waited > 3.:
                gui(menubar_ctrl.on_stop_activate, None, None)
                raise RuntimeError("The state machine should finish in less then {0}".format(duration_waited))
        print("Run duration of execute_preemption_of_all_state_machines_at_once was: {0}".format(duration_waited))
        assert len([record for record in deprecations_warnings if record.category is log.RAFCONDeprecationWarning]) == 4
        # The warning "The value of output port is 'None'. It has replaced with the default value." will be thrown twice
        gui.expected_warnings = 2

if __name__ == '__main__':
    # test_preemption_of_all_state_machines_at_once(None)
    test_all_generic_libraries_in_a_row(None)
    # import pytest
    # pytest.main(['-s', __file__])
