from __future__ import print_function
import time

# test environment elements
import testing_utils
from testing_utils import call_gui_callback, wait_for_execution_engine_sync_counter

# general tool elements
from rafcon.utils import log
logger = log.get_logger(__name__)


def initialize_global_variables():
    import rafcon.gui.singleton as gui_singleton
    gui_singleton.global_variable_manager_model.global_variable_manager.set_variable("global_variable_1", "value1")
    gui_singleton.global_variable_manager_model.global_variable_manager.set_variable("global_variable_2", "value2")


def create_state_machine():
    from rafcon.core.state_machine import StateMachine
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.states.execution_state import ExecutionState
    root = HierarchyState(name='root')
    ex1 = ExecutionState(name='1')
    root.add_state(ex1)
    ex2 = ExecutionState(name='2')
    root.add_state(ex2)
    ex3 = ExecutionState(name='3')
    root.add_state(ex3)
    root.start_state_id = ex1.state_id
    root.add_transition(ex1.state_id, 0, ex2.state_id, None)
    root.add_transition(ex2.state_id, 0, ex3.state_id, None)
    t_id = root.add_transition(ex3.state_id, 0, root.state_id, 0)
    return StateMachine(root_state=root), t_id


def iter_execution_modes():
    """ Test execution modes
    
      - The execution history widget should be updated always if a mode is changed and not STARTED
      - the execution mode change from step mode to pause should not cause a step (change of current active state)
      - or the opposite from pause to step mode should not cause a step (change of current active state)
    """
    from rafcon.core.singleton import state_machine_execution_engine, state_machine_manager
    import rafcon.gui.singleton as gui_singleton
    from rafcon.gui.controllers.menu_bar import MenuBarController
    from rafcon.gui.controllers.execution_history import ExecutionHistoryTreeController

    menubar_ctrl = gui_singleton.main_window_controller.get_controller('menu_bar_controller')
    assert isinstance(menubar_ctrl, MenuBarController)
    execution_history_ctrl = gui_singleton.main_window_controller.get_controller('execution_history_ctrl')
    assert isinstance(execution_history_ctrl, ExecutionHistoryTreeController)
    testing_utils.call_gui_callback(testing_utils.wait_for_gui)

    sm, remove_t_id = testing_utils.call_gui_callback(create_state_machine)
    sm_id = testing_utils.call_gui_callback(state_machine_manager.add_state_machine, sm)
    testing_utils.call_gui_callback(testing_utils.wait_for_gui)
    sm_m = gui_singleton.state_machine_manager_model.state_machines[sm_id]

    def wait_for_sync_counter_change(current_sync_counter=None):
        if current_sync_counter is None:
            current_sync_counter = state_machine_execution_engine.synchronization_counter
        print("##### before\n{0}\n#####".format(current_sync_counter))
        while current_sync_counter == state_machine_execution_engine.synchronization_counter:
            time.sleep(0.1)
            print("##### in wait\n{0}\n#####".format(state_machine_execution_engine.synchronization_counter))
        print("##### after\n{0}\n#####".format(state_machine_execution_engine.synchronization_counter))
        testing_utils.call_gui_callback(testing_utils.wait_for_gui)

    from gtkmvc3.observer import Observer

    class ActiveStateObserver(Observer):
        def __init__(self, model):
            Observer.__init__(self)
            self.model = model
            self.observe_model(model)
            self.last_execution_change_at_state = None

        @Observer.observe("state_machine", after=True)
        def execution_change(self, model, prop_name, info):
            from rafcon.gui.utils.notification_overview import NotificationOverview
            overview = NotificationOverview(info)
            if overview['method_name'][-1] == 'state_execution_status':
                print("CURRENT STATE: {0}".format(overview['model'][-1].state.get_path()))
                self.last_execution_change_at_state = overview['model'][-1].state.get_path()

    execution_observer = call_gui_callback(ActiveStateObserver, sm_m)

    ############################################################
    print("{0}# EXAMPLE #1: toggle from STEP MODE to PAUSE and back and RUN till finished{0}".format('\n' + 40 * '#' + '\n'))
    ############################################################
    # check that new execution history is shown in widget
    number_of_executions_before = len(execution_history_ctrl.history_tree_store)
    print("history length before: {0}\n".format(number_of_executions_before))
    current_sync_counter = state_machine_execution_engine.synchronization_counter
    testing_utils.call_gui_callback(menubar_ctrl.on_step_mode_activate, None)
    wait_for_sync_counter_change(current_sync_counter)
    number_of_executions_after = len(execution_history_ctrl.history_tree_store)
    print("history length after1: {0}\n".format(number_of_executions_after))
    # in the gui this works but here it often fails without sleep
    i1 = 0.
    while not number_of_executions_before + 1 == number_of_executions_after and not i1 > 2.:
        time.sleep(0.1)
        i1 += 0.1
    print("history length after2: {0}\n".format(number_of_executions_after))
    # assert number_of_executions_before + 1 == number_of_executions_after  # TODO uncomment and include into test

    # active state should not change from step mode to pause -> this can cause bad situations with a robot in the loop
    last_active_state = execution_observer.last_execution_change_at_state
    print("last active state {0}".format(last_active_state))
    testing_utils.call_gui_callback(menubar_ctrl.on_pause_activate, None)
    testing_utils.call_gui_callback(testing_utils.wait_for_gui)
    current_active_state = execution_observer.last_execution_change_at_state
    print("last active state {0}".format(last_active_state))
    print("current active state {0} {1}".format(current_active_state, last_active_state == current_active_state))
    # TODO This fails currently if you do it with the gui by hand -> the pause cause one more step
    # assert last_active_state == current_active_state

    # active state should not change from pause to step mode -> this can cause bad situations with a robot in the loop
    last_active_state = execution_observer.last_execution_change_at_state
    print("last active state {0}".format(last_active_state))
    testing_utils.call_gui_callback(menubar_ctrl.on_step_mode_activate, None)
    testing_utils.call_gui_callback(testing_utils.wait_for_gui)
    current_active_state = execution_observer.last_execution_change_at_state
    print("last active state {0}".format(last_active_state))
    print("current active state {0} {1}".format(current_active_state, last_active_state == current_active_state))
    # TODO This fails currently if you do it with the gui by hand -> the pause cause one more step
    # assert last_active_state == current_active_state

    # finish the state machine
    testing_utils.call_gui_callback(menubar_ctrl.on_start_activate, None)
    while not state_machine_execution_engine.finished_or_stopped():
        time.sleep(0.1)
    testing_utils.call_gui_callback(testing_utils.wait_for_gui)  # propagate securely the stop
    time.sleep(0.5)

    ############################################################
    print("{0}# EXAMPLE #2: RUN root state till finished{0}".format('\n' + 40 * '#' + '\n'))
    ############################################################
    # check that new execution history is shown in widget -> if not shown you may look into the wrong history
    number_of_executions_before = len(execution_history_ctrl.history_tree_store)
    time.sleep(0.2)
    print("history length before: {0}\n".format(number_of_executions_before))
    testing_utils.call_gui_callback(menubar_ctrl.on_start_activate, None)
    while not state_machine_execution_engine.finished_or_stopped():
        time.sleep(0.1)
    testing_utils.call_gui_callback(testing_utils.wait_for_gui)
    number_of_executions_after = len(execution_history_ctrl.history_tree_store)
    print("history length after: {0}\n".format(number_of_executions_after))
    assert number_of_executions_before + 1 == number_of_executions_after

    # REMOVE ONE TRANSITION
    testing_utils.call_gui_callback(sm_m.root_state.state.remove_transition, remove_t_id)

    # ############################################################   # TODO uncomment and include into test
    # print("{0}# EXAMPLE #3: RUN and PAUSE{0}".format('\n' + 40 * '#' + '\n'))
    # ############################################################
    # # check that new execution history is shown in widget -> if not shown you may look into the wrong history
    # number_of_executions_before = len(execution_history_ctrl.history_tree_store)
    # print("history length before: {0}\n".format(number_of_executions_before))
    # current_sync_counter = state_machine_execution_engine.synchronization_counter
    # testing_utils.call_gui_callback(menubar_ctrl.on_start_activate, None)
    # wait_for_sync_counter_change(current_sync_counter)
    #
    # time.sleep(0.5)  # TODO why I need this sleeps
    # testing_utils.call_gui_callback(menubar_ctrl.on_pause_activate, None)
    # testing_utils.call_gui_callback(testing_utils.wait_for_gui)
    # time.sleep(0.5)  # TODO why I need this sleeps
    #
    # number_of_executions_after = len(execution_history_ctrl.history_tree_store)
    # print("history length after: {0}\n".format(number_of_executions_after))
    # # this works currently with the gui but here it often fails
    # assert number_of_executions_before + 1 == number_of_executions_after
    # testing_utils.call_gui_callback(menubar_ctrl.on_stop_activate, None)
    # print("\nSTOPPED FINISHED {0}\n".format(state_machine_execution_engine.finished_or_stopped()))
    # while not state_machine_execution_engine.finished_or_stopped():
    #     time.sleep(0.1)
    #     print("\nSTOPPED FINISHED {0}\n".format(state_machine_execution_engine.finished_or_stopped()))
    # testing_utils.call_gui_callback(testing_utils.wait_for_gui)  # propagate securely the stop
    # print("\nSTOPPED FINISHED {0} {1}\n".format(state_machine_execution_engine.finished_or_stopped(), state_machine_execution_engine))
    # time.sleep(0.5)  # TODO why I need this sleeps -- and why it is so crazy hard to confirm stop but it is running while next start in next test
    #
    # ############################################################
    # print("{0}# EXAMPLE #4: RUN and STOP{0}".format('\n' + 40 * '#' + '\n'))
    # ############################################################
    # # check that new execution history is shown in widget -> if not shown you may look into the wrong history
    # number_of_executions_before = len(execution_history_ctrl.history_tree_store)
    # print("history length before: {0}\n".format(number_of_executions_before))
    # current_sync_counter = state_machine_execution_engine.synchronization_counter
    # testing_utils.call_gui_callback(menubar_ctrl.on_start_activate, None)
    # wait_for_sync_counter_change(current_sync_counter)
    # number_of_executions_after = len(execution_history_ctrl.history_tree_store)
    # print("history length after: {0}\n".format(number_of_executions_after))
    # testing_utils.call_gui_callback(menubar_ctrl.on_stop_activate, None)
    # while not state_machine_execution_engine.finished_or_stopped():
    #     time.sleep(0.1)
    # testing_utils.call_gui_callback(testing_utils.wait_for_gui)
    # testing_utils.wait_for_gui()
    # number_of_executions_after = len(execution_history_ctrl.history_tree_store)
    # # this works currently with the gui
    # assert number_of_executions_before + 1 == number_of_executions_after
    # testing_utils.call_gui_callback(testing_utils.wait_for_gui)

    execution_observer.relieve_model(sm_m)


def test_execution_modes(caplog):
    testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
    call_gui_callback(initialize_global_variables)
    try:
        iter_execution_modes()
    except Exception as e:
        raise
    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=caplog)


if __name__ == '__main__':
    testing_utils.dummy_gui(None)
    test_execution_modes(None)
    # import pytest
    # pytest.main(['-s', __file__])
