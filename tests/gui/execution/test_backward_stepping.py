import pytest

import os
import time

# test environment elements
from tests import utils as testing_utils
from tests.utils import call_gui_callback, wait_for_execution_engine_sync_counter
from tests.gui.execution import state_machines_editor_tab_status_check

# general tool elements
from rafcon.utils import log
logger = log.get_logger(__name__)


def initialize_global_variables():
    import rafcon.gui.singleton as gui_singleton
    gui_singleton.global_variable_manager_model.global_variable_manager.set_variable("global_variable_1", "value1")
    gui_singleton.global_variable_manager_model.global_variable_manager.set_variable("global_variable_2", "value2")


@pytest.mark.parametrize('gui', [{"libraries": {
    'unit_test': os.path.join(testing_utils.TESTS_PATH, 'assets', 'unit_test_state_machines',
                              'backward_step_library_execution_test', 'test_library')}}], indirect=True, ids=["with backward step libraries"])
def test_backward_stepping_library_state(gui):
    gui(initialize_global_variables)

    from rafcon.core.singleton import state_machine_execution_engine, state_machine_manager
    import rafcon.gui.singleton as gui_singleton

    menubar_ctrl = gui_singleton.main_window_controller.menu_bar_controller
    sm = gui(
        menubar_ctrl.on_open_activate, None, None,
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "backward_step_library_execution_test"))
    )
    testing_utils.wait_for_gui()
    # reset the synchronization counter; although the tests run in different processes they share their memory
    # as the import statements are at the top of the file and not inside the parallel called functions
    with state_machine_execution_engine._status.execution_condition_variable:
        state_machine_execution_engine.synchronization_counter = 0

    gui(menubar_ctrl.on_step_mode_activate, None, None)
    current_state_machine_id = gui_singleton.state_machine_manager.active_state_machine_id
    state_machines_editor_tab_status_check(current_state_machine_id, active=True)  # execution start is synchronous
    wait_for_execution_engine_sync_counter(1, logger)

    for i in range(5):
        gui(menubar_ctrl.on_step_into_activate, None, None)
        wait_for_execution_engine_sync_counter(1, logger)

    for i in range(4):
        gui(menubar_ctrl.on_backward_step_activate, None, None)
        wait_for_execution_engine_sync_counter(1, logger)

    gui(menubar_ctrl.on_backward_step_activate, None, None)

    while not state_machine_execution_engine.finished_or_stopped():
        time.sleep(0.1)
    for key, sd in sm.root_state.scoped_data.items():
        if sd.name == "beer_count":
            assert sd.value == 100
    # stop or finished are asynchronous but the call_gui_callback makes the check synchronous
    gui(state_machines_editor_tab_status_check, current_state_machine_id, False)


def verify_execute_preemptive_state_forwards_backwards():
    import rafcon.gui.singleton as gui_singleton
    gvm = gui_singleton.global_variable_manager_model
    beers = gvm.global_variable_manager.get_variable('beers')
    whiskey = gvm.global_variable_manager.get_variable('whiskey')
    assert beers == 0
    assert whiskey == 0


def test_backward_stepping_preemptive_state(gui):
    gui(initialize_global_variables)

    from rafcon.core.singleton import state_machine_execution_engine

    menubar_ctrl = gui.singletons.main_window_controller.menu_bar_controller

    state_machine = gui(
        menubar_ctrl.on_open_activate, None, None,
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "backward_step_preemtive_test"))
    )
    testing_utils.wait_for_gui()

    # reset the synchronization counter; although the tests run in different processes they share their memory
    # as the import statements are at the top of the file and not inside the parallel called functions
    with state_machine_execution_engine._status.execution_condition_variable:
        state_machine_execution_engine.synchronization_counter = 0

    gui(menubar_ctrl.on_step_mode_activate, None, None)
    current_state_machine_id = gui.singletons.state_machine_manager.active_state_machine_id
    state_machines_editor_tab_status_check(current_state_machine_id, active=True)  # execution start is synchronous

    wait_for_execution_engine_sync_counter(1, logger)

    for i in range(3):
        gui(menubar_ctrl.on_step_into_activate, None, None)
        wait_for_execution_engine_sync_counter(2, logger)

    gui(menubar_ctrl.on_step_into_activate, None, None)
    wait_for_execution_engine_sync_counter(1, logger)

    # preemptive concurrency state must be finished before the next step
    while not state_machine.get_state_by_path("AOURYA/LXEMOO").final_outcome:
        time.sleep(0.010)

    gui(menubar_ctrl.on_step_into_activate, None, None)
    wait_for_execution_engine_sync_counter(1, logger)

    # "take turn" state reached

    # backward
    for i in range(1):
        gui(menubar_ctrl.on_backward_step_activate, None, None)
        wait_for_execution_engine_sync_counter(1, logger)

    for i in range(3):
        gui(menubar_ctrl.on_backward_step_activate, None, None)
        wait_for_execution_engine_sync_counter(2, logger)

    state_machines_editor_tab_status_check(current_state_machine_id, active=True)
    gui(menubar_ctrl.on_backward_step_activate, None, None)

    while not state_machine_execution_engine.finished_or_stopped():
        time.sleep(0.1)
    # stop or finished are asynchronous but the call_gui_callback makes the check synchronous
    gui(state_machines_editor_tab_status_check, current_state_machine_id, False)

    gui(verify_execute_preemptive_state_forwards_backwards)


def test_backward_stepping_barrier_state(gui):
    gui(initialize_global_variables)

    from rafcon.core.singleton import state_machine_execution_engine, state_machine_manager
    import rafcon.gui.singleton as gui_singleton

    menubar_ctrl = gui_singleton.main_window_controller.get_controller('menu_bar_controller')

    sm = gui(
        menubar_ctrl.on_open_activate, None, None,
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "backward_step_barrier_test"))
    )
    testing_utils.wait_for_gui()

    # reset the synchronization counter; although the tests run in different processes they share their memory
    # as the import statements are at the top of the file and not inside the parallel called functions
    with state_machine_execution_engine._status.execution_condition_variable:
        state_machine_execution_engine.synchronization_counter = 0

    gui(menubar_ctrl.on_step_mode_activate, sm.state_machine_id, None)
    wait_for_execution_engine_sync_counter(1, logger)

    for i in range(2):
        gui(menubar_ctrl.on_step_into_activate, None, None)
        wait_for_execution_engine_sync_counter(3, logger)

    gui(menubar_ctrl.on_step_over_activate, None, None)
    wait_for_execution_engine_sync_counter(3, logger)

    gui(menubar_ctrl.on_step_out_activate, None, None)
    wait_for_execution_engine_sync_counter(1, logger)

    for i in range(3):
        gui(menubar_ctrl.on_step_into_activate, None, None)
        wait_for_execution_engine_sync_counter(1, logger)

    # backward
    for i in range(3):
        gui(menubar_ctrl.on_backward_step_activate, None, None)
        wait_for_execution_engine_sync_counter(1, logger)

    print("cp1")

    for i in range(4):
        gui(menubar_ctrl.on_backward_step_activate, None, None)
        wait_for_execution_engine_sync_counter(3, logger)

    print("cp2")

    gui(menubar_ctrl.on_backward_step_activate, None, None)

    print("cp3")

    while not state_machine_execution_engine.finished_or_stopped():
        time.sleep(0.1)

    print("cp4")

    testing_utils.wait_for_gui()

    print("cp5")

    for key, sd in sm.root_state.scoped_data.items():
        if sd.name == "beer_number":
            assert sd.value == 100
        elif sd.name == "wine_number":
            assert sd.value == 40
        elif sd.name == "whiskey_number":
            assert sd.value == 20

    gui.expected_warnings = 3
    gui(menubar_ctrl.on_stop_activate, None)




if __name__ == '__main__':
    # test_backward_stepping_library_state(None)
    test_backward_stepping_barrier_state(None)
    # test_backward_stepping_preemptive_state(None)
    # import pytest
    # pytest.main(['-s', __file__])
