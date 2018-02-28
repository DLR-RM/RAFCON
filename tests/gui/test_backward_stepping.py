import os
import time
import datetime
import pytest

# test environment elements
import testing_utils
from testing_utils import call_gui_callback

# general tool elements
from rafcon.utils import log
logger = log.get_logger(__name__)


def initialize_global_variables():
    import rafcon.gui.singleton as gui_singleton
    gui_singleton.global_variable_manager_model.global_variable_manager.set_variable("global_variable_1", "value1")
    gui_singleton.global_variable_manager_model.global_variable_manager.set_variable("global_variable_2", "value2")


def wait_for_execution_engine_sync_counter(target_value, logger, timeout=5):
    from rafcon.core.singleton import state_machine_execution_engine
    logger.debug("++++++++++ waiting for execution engine sync for " + str(target_value) + " steps ++++++++++")
    current_time = datetime.datetime.now()
    while True:
        state_machine_execution_engine.synchronization_lock.acquire()
        if state_machine_execution_engine.synchronization_counter == target_value:
            state_machine_execution_engine.synchronization_counter = 0
            state_machine_execution_engine.synchronization_lock.release()
            break
        state_machine_execution_engine.synchronization_lock.release()
        if (datetime.datetime.now() - current_time).seconds > timeout:
            raise RuntimeError("Something went wrong while waiting for states to finish!")
        time.sleep(0.1)


def execute_library_state_forwards_backwards():
    from rafcon.core.singleton import state_machine_execution_engine, state_machine_manager
    import rafcon.gui.singleton as gui_singleton

    menubar_ctrl = gui_singleton.main_window_controller.get_controller('menu_bar_controller')
    call_gui_callback(
        menubar_ctrl.on_open_activate, None, None,
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "backward_step_library_execution_test"))
    )
    testing_utils.wait_for_gui()
    # reset the synchronization counter; although the tests run in different processes they share their memory
    # as the import statements are at the top of the file and not inside the parallel called functions
    state_machine_execution_engine.synchronization_lock.acquire()
    state_machine_execution_engine.synchronization_counter = 0
    state_machine_execution_engine.synchronization_lock.release()

    call_gui_callback(menubar_ctrl.on_step_mode_activate, None, None)
    wait_for_execution_engine_sync_counter(1, logger)

    # forward
    for i in range(5):
        call_gui_callback(menubar_ctrl.on_step_into_activate, None, None)
        wait_for_execution_engine_sync_counter(1, logger)

    # backward
    for i in range(4):
        call_gui_callback(menubar_ctrl.on_backward_step_activate, None, None)
        wait_for_execution_engine_sync_counter(1, logger)

    call_gui_callback(menubar_ctrl.on_backward_step_activate, None, None)

    sm = state_machine_manager.get_active_state_machine()
    while not state_machine_execution_engine.finished_or_stopped():
        time.sleep(0.1)
    for key, sd in sm.root_state.scoped_data.iteritems():
        if sd.name == "beer_count":
            assert sd.value == 100

    call_gui_callback(menubar_ctrl.on_stop_activate, None)


def test_backward_stepping_library_state(caplog):
    testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False},
                          libraries={'unit_test': os.path.join(testing_utils.TESTS_PATH, 'assets',
                                                               'unit_test_state_machines',
                                                               'backward_step_library_execution_test', 'test_library')
                                     }
                          )
    call_gui_callback(initialize_global_variables)
    try:
        execute_library_state_forwards_backwards()
    except Exception:
        raise
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog)


def verify_execute_preemptive_state_forwards_backwards():
    import rafcon.gui.singleton as gui_singleton
    gvm = gui_singleton.global_variable_manager_model
    beers = gvm.global_variable_manager.get_variable('beers')
    whiskey = gvm.global_variable_manager.get_variable('whiskey')
    assert beers == 0
    assert whiskey == 0


def execute_preemptive_state_forwards_backwards():
    from rafcon.core.singleton import state_machine_execution_engine
    import rafcon.gui.singleton as gui_singleton

    menubar_ctrl = gui_singleton.main_window_controller.get_controller('menu_bar_controller')

    call_gui_callback(
        menubar_ctrl.on_open_activate, None, None,
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "backward_step_preemtive_test"))
    )
    testing_utils.wait_for_gui()

    # reset the synchronization counter; although the tests run in different processes they share their memory
    # as the import statements are at the top of the file and not inside the parallel called functions
    state_machine_execution_engine.synchronization_lock.acquire()
    state_machine_execution_engine.synchronization_counter = 0
    state_machine_execution_engine.synchronization_lock.release()

    call_gui_callback(menubar_ctrl.on_step_mode_activate, None, None)

    wait_for_execution_engine_sync_counter(1, logger)

    # forward
    for i in range(3):
        call_gui_callback(menubar_ctrl.on_step_into_activate, None, None)
        wait_for_execution_engine_sync_counter(2, logger)

    for i in range(2):
        call_gui_callback(menubar_ctrl.on_step_into_activate, None, None)
        wait_for_execution_engine_sync_counter(1, logger)

    # "take turn" state reached

    # backward
    for i in range(1):
        call_gui_callback(menubar_ctrl.on_backward_step_activate, None, None)
        wait_for_execution_engine_sync_counter(1, logger)

    for i in range(3):
        call_gui_callback(menubar_ctrl.on_backward_step_activate, None, None)
        wait_for_execution_engine_sync_counter(2, logger)

    call_gui_callback(menubar_ctrl.on_backward_step_activate, None, None)

    while not state_machine_execution_engine.finished_or_stopped():
        time.sleep(0.1)

    call_gui_callback(verify_execute_preemptive_state_forwards_backwards)
    call_gui_callback(menubar_ctrl.on_stop_activate, None)


def test_backward_stepping_preemptive_state(caplog):
    testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
    call_gui_callback(initialize_global_variables)
    try:
        execute_preemptive_state_forwards_backwards()
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog)
        # testing_utils.wait_for_gui()


def execute_barrier_state_forwards_backwards():
    from rafcon.core.singleton import state_machine_execution_engine, state_machine_manager
    import rafcon.gui.singleton as gui_singleton

    menubar_ctrl = gui_singleton.main_window_controller.get_controller('menu_bar_controller')

    call_gui_callback(
        menubar_ctrl.on_open_activate, None, None,
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "backward_step_barrier_test"))
    )
    testing_utils.wait_for_gui()

    # reset the synchronization counter; although the tests run in different processes they share their memory
    # as the import statements are at the top of the file and not inside the parallel called functions
    state_machine_execution_engine.synchronization_lock.acquire()
    state_machine_execution_engine.synchronization_counter = 0
    state_machine_execution_engine.synchronization_lock.release()

    call_gui_callback(menubar_ctrl.on_step_mode_activate, None, None)
    wait_for_execution_engine_sync_counter(1, logger)

    # forward
    for i in range(4):
        call_gui_callback(menubar_ctrl.on_step_into_activate, None, None)
        wait_for_execution_engine_sync_counter(3, logger)

    for i in range(4):
        call_gui_callback(menubar_ctrl.on_step_into_activate, None, None)
        wait_for_execution_engine_sync_counter(1, logger)

    # backward
    for i in range(3):
        call_gui_callback(menubar_ctrl.on_backward_step_activate, None, None)
        wait_for_execution_engine_sync_counter(1, logger)

    print "cp1"

    for i in range(4):
        call_gui_callback(menubar_ctrl.on_backward_step_activate, None, None)
        wait_for_execution_engine_sync_counter(3, logger)

    print "cp2"

    call_gui_callback(menubar_ctrl.on_backward_step_activate, None, None)

    print "cp3"

    sm = state_machine_manager.get_active_state_machine()
    while not state_machine_execution_engine.finished_or_stopped():
        time.sleep(0.1)

    print "cp4"

    testing_utils.wait_for_gui()

    print "cp5"

    for key, sd in sm.root_state.scoped_data.iteritems():
        if sd.name == "beer_number":
            assert sd.value == 100
        elif sd.name == "wine_number":
            assert sd.value == 40
        elif sd.name == "whiskey_number":
            assert sd.value == 20

    call_gui_callback(menubar_ctrl.on_stop_activate, None)


def test_backward_stepping_barrier_state(caplog):
    testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
    call_gui_callback(initialize_global_variables)
    try:
        execute_barrier_state_forwards_backwards()
    except Exception,e:
        raise
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog)


if __name__ == '__main__':
    # test_backward_stepping_library_state(None)
    test_backward_stepping_barrier_state(None)
    # test_backward_stepping_preemptive_state(None)
    # pytest.main(['-s', __file__])
