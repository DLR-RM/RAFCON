import os
import time

from tests import utils as testing_utils

SM1_PATH = testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "99_bottles_of_beer_no_wait"))
SM2_PATH = testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "validity_check_test"))


def test_lock_execution_history_view(gui):
    import rafcon.gui.singleton as gui_singleton
    from rafcon.core.singleton import state_machine_execution_engine

    menubar_ctrl = gui_singleton.main_window_controller.get_controller('menu_bar_controller')
    smm_model = gui_singleton.state_machine_manager_model
    execution_history_ctrl = gui_singleton.main_window_controller.get_controller('execution_history_ctrl')
    lock_checkbox = execution_history_ctrl.view['lock_checkbox']

    # open and run SM1 so execution history is populated
    gui(menubar_ctrl.on_open_activate, None, None, SM1_PATH)
    sm1_id = smm_model.selected_state_machine_id
    gui(menubar_ctrl.on_start_activate, None)

    # run for max 15 seconds or until finished (whichever comes first)
    start_time = time.time()
    timeout = 15.0
    while not state_machine_execution_engine.finished_or_stopped():
        if time.time() - start_time > timeout:
            gui(menubar_ctrl.on_stop_activate, None)
            break
        time.sleep(0.05)
    testing_utils.wait_for_gui()

    # reload so the tree reflects the completed run, then capture row count
    gui(execution_history_ctrl.reload_history, None)
    history_rows_after_run = len(execution_history_ctrl.history_tree_store)
    assert history_rows_after_run > 0

    # lock the history view while SM1 is the active tab
    gui(lock_checkbox.set_active, True)
    assert execution_history_ctrl.lock_view_flag is True

    # open SM2 (never run) — switches the active tab to SM2
    gui(menubar_ctrl.on_open_activate, None, None, SM2_PATH)
    sm2_id = smm_model.selected_state_machine_id
    assert sm2_id != sm1_id

    # history must still show SM1's data — the lock held
    assert len(execution_history_ctrl.history_tree_store) == history_rows_after_run
    assert execution_history_ctrl.lock_state_machine_model.state_machine.state_machine_id == sm1_id

    # pressing "Reload History" unlocks and refreshes to the current SM (SM2, which never ran)
    gui(execution_history_ctrl.reload_history, None)
    assert execution_history_ctrl.lock_view_flag is False
    assert len(execution_history_ctrl.history_tree_store) == 0
