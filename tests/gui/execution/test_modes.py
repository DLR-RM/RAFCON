import time

# test environment elements
import tests.utils as testing_utils
from tests.utils import call_gui_callback, TEST_SCRIPT_PATH

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
    ex1 = ExecutionState(name='1', filename="script_small_wait.py", path=TEST_SCRIPT_PATH)
    root.add_state(ex1)
    ex2 = ExecutionState(name='2', filename="script_small_wait.py", path=TEST_SCRIPT_PATH)
    root.add_state(ex2)
    ex3 = ExecutionState(name='3', filename="script_small_wait.py", path=TEST_SCRIPT_PATH)
    root.add_state(ex3)

    # hierarchy state at the beginning
    h4 = HierarchyState('H4')
    ex41 = ExecutionState(name='41', filename="script_small_wait.py", path=TEST_SCRIPT_PATH)
    h4.add_state(ex41)
    ex42 = ExecutionState(name='42', filename="script_small_wait.py", path=TEST_SCRIPT_PATH)
    h4.add_state(ex42)
    ex43 = ExecutionState(name='43', filename="script_small_wait.py", path=TEST_SCRIPT_PATH)
    h4.add_state(ex43)
    h4.start_state_id = ex41.state_id
    h4.add_transition(ex41.state_id, 0, ex42.state_id, None)
    h4.add_transition(ex42.state_id, 0, ex43.state_id, None)
    h4.add_transition(ex43.state_id, 0, h4.state_id, 0)

    root.add_state(h4)
    root.start_state_id = ex1.state_id
    root.add_transition(h4.state_id, 0, ex1.state_id, None)
    root.add_transition(ex1.state_id, 0, ex2.state_id, None)
    root.add_transition(ex2.state_id, 0, ex3.state_id, None)
    t_id = root.add_transition(ex3.state_id, 0, root.state_id, 0)
    return StateMachine(root_state=root), t_id, h4.state_id


def test_execution_modes(gui):
    """ Test execution modes

      - The execution history widget should be updated always if a mode is changed and not STARTED
      - the execution mode change from step mode to pause should not cause a step (change of current active state)
      - or the opposite from pause to step mode should not cause a step (change of current active state)
    """
    call_gui_callback(initialize_global_variables)
    from rafcon.core.singleton import state_machine_execution_engine, state_machine_manager
    import rafcon.gui.singleton as gui_singleton
    from rafcon.gui.controllers.menu_bar import MenuBarController
    from rafcon.gui.controllers.execution_history import ExecutionHistoryTreeController

    menubar_ctrl = gui_singleton.main_window_controller.get_controller('menu_bar_controller')
    assert isinstance(menubar_ctrl, MenuBarController)
    execution_history_ctrl = gui_singleton.main_window_controller.get_controller('execution_history_ctrl')
    assert isinstance(execution_history_ctrl, ExecutionHistoryTreeController)
    testing_utils.call_gui_callback(testing_utils.wait_for_gui)

    sm, remove_t_id, add_start_state_id = testing_utils.call_gui_callback(create_state_machine)
    sm_id = testing_utils.call_gui_callback(state_machine_manager.add_state_machine, sm)
    testing_utils.call_gui_callback(testing_utils.wait_for_gui)
    sm_m = gui_singleton.state_machine_manager_model.state_machines[sm_id]

    def wait_for_sync_counter_change_and_wait_for_gui(current_sync_counter):
        assert current_sync_counter is not None
        print("##### before {0} #####".format(current_sync_counter))
        while current_sync_counter == state_machine_execution_engine.synchronization_counter:
            time.sleep(0.1)
            print("##### in wait {0} #####".format(state_machine_execution_engine.synchronization_counter))
        print("##### after {0} #####".format(state_machine_execution_engine.synchronization_counter))
        testing_utils.call_gui_callback(testing_utils.wait_for_gui)

    from rafcon.design_patterns.observer.observer import Observer

    class ActiveStateObserver(Observer):
        def __init__(self, model):
            Observer.__init__(self)
            self.model = model
            self.observe_model(model)
            self.last_execution_change_at_state = None

        def reset(self):
            self.last_execution_change_at_state = None

        @Observer.observe("state_machine", after=True)
        def execution_change(self, model, prop_name, info):
            from rafcon.gui.utils.notification_overview import NotificationOverview
            overview = NotificationOverview(info)
            if overview.get_cause() == 'state_execution_status':
                self.last_execution_change_at_state = overview.get_affected_model().state.get_path()

    execution_observer = call_gui_callback(ActiveStateObserver, sm_m)

    ############################################################
    print("{0}# EXAMPLE #1: toggle from STEP MODE to PAUSE and back and RUN till finished{0}".format('\n' + 40 * '#' + '\n'))
    ############################################################
    # check that new execution history is shown in widget
    number_of_executions_before = len(execution_history_ctrl.history_tree_store)
    print("history length before: {0}\n".format(number_of_executions_before))
    current_sync_counter = state_machine_execution_engine.synchronization_counter
    testing_utils.call_gui_callback(menubar_ctrl.on_step_mode_activate, None)
    wait_for_sync_counter_change_and_wait_for_gui(current_sync_counter)
    number_of_executions_after = len(execution_history_ctrl.history_tree_store)
    print("history length after1: {0}\n".format(number_of_executions_after))
    # Rico's old code:
    # in the gui this works but here it often fails without sleep; TODO: this cannot be!
    # i1 = 0.
    # this while will never work! number_of_executions_before/after hold number copies and no references to numbers!
    # while not number_of_executions_before + 1 == number_of_executions_after and not i1 > 2.:
    #     time.sleep(0.1)
    #     i1 += 0.1
    # print("history length after2: {0}\n".format(number_of_executions_after))
    # this must not work! the execution history is not observing the state_machine itself but the execution_engine
    # and we should not change that for performance reasons!
    # assert number_of_executions_before + 1 == number_of_executions_after

    # active state should not change from step mode to pause -> this can cause bad situations with a robot in the loop
    last_active_state = execution_observer.last_execution_change_at_state
    print("last active state {0}".format(last_active_state))
    testing_utils.call_gui_callback(menubar_ctrl.on_pause_activate, None)
    testing_utils.call_gui_callback(testing_utils.wait_for_gui)
    current_active_state = execution_observer.last_execution_change_at_state
    print("last active state {0}".format(last_active_state))
    print("current active state {0} {1}".format(current_active_state, last_active_state == current_active_state))
    assert last_active_state == current_active_state

    # active state should not change from pause to step mode -> this can cause bad situations with a robot in the loop
    last_active_state = execution_observer.last_execution_change_at_state
    print("last active state {0}".format(last_active_state))
    testing_utils.call_gui_callback(menubar_ctrl.on_step_mode_activate, None)
    testing_utils.call_gui_callback(testing_utils.wait_for_gui)
    current_active_state = execution_observer.last_execution_change_at_state
    print("last active state {0}".format(last_active_state))
    print("current active state {0} {1}".format(current_active_state, last_active_state == current_active_state))
    assert last_active_state == current_active_state

    # finish the state machine
    testing_utils.call_gui_callback(menubar_ctrl.on_start_activate, None)
    while not state_machine_execution_engine.finished_or_stopped():
        time.sleep(0.1)
    testing_utils.call_gui_callback(testing_utils.wait_for_gui)  # propagate securely the stop


    ############################################################
    print("{0}# EXAMPLE #2: RUN root state till finished{0}".format('\n' + 40 * '#' + '\n'))
    ############################################################
    # check that new execution history is shown in widget -> if not shown you may look into the wrong history
    # Look out! if you want to work with the execution history either trigger a execution command or use reload
    testing_utils.call_gui_callback(execution_history_ctrl.reload_history, None)
    number_of_executions_before = len(execution_history_ctrl.history_tree_store)
    print("history length before: {0}\n".format(number_of_executions_before))
    testing_utils.call_gui_callback(menubar_ctrl.on_start_activate, None)
    while not state_machine_execution_engine.finished_or_stopped():
        time.sleep(0.1)
    testing_utils.call_gui_callback(testing_utils.wait_for_gui)
    testing_utils.call_gui_callback(execution_history_ctrl.reload_history, None)
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
    # wait_for_sync_counter_change_and_wait_for_gui(current_sync_counter)
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
    # wait_for_sync_counter_change_and_wait_for_gui(current_sync_counter)
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

    ############################################################
    print("{0}# EXAMPLE #5: step mode (step over) with first state hierarchy state {0}".format('\n' + 40 * '#' + '\n'))
    ############################################################
    # testing_utils.call_gui_callback(sm_m.root_state.state.__setattr__, 'start_state_id', add_start_state_id)
    testing_utils.call_gui_callback(testing_utils.wait_for_gui)
    testing_utils.call_gui_callback(execution_observer.reset)
    # check that execution does a step over even if the first state is a hierarchy state
    last_active_state = execution_observer.last_execution_change_at_state
    print("#0 {0}".format(last_active_state))

    current_sync_counter = state_machine_execution_engine.synchronization_counter
    testing_utils.call_gui_callback(menubar_ctrl.on_step_mode_activate, None)
    wait_for_sync_counter_change_and_wait_for_gui(current_sync_counter)
    current_active_state = execution_observer.last_execution_change_at_state
    print("#1 {0}".format(current_active_state))

    current_sync_counter = state_machine_execution_engine.synchronization_counter
    testing_utils.call_gui_callback(menubar_ctrl.on_step_over_activate, None)
    wait_for_sync_counter_change_and_wait_for_gui(current_sync_counter)
    after_so1_current_active_state = execution_observer.last_execution_change_at_state
    print("#2 {0}".format(after_so1_current_active_state))

    current_sync_counter = state_machine_execution_engine.synchronization_counter
    testing_utils.call_gui_callback(menubar_ctrl.on_step_over_activate, None)
    wait_for_sync_counter_change_and_wait_for_gui(current_sync_counter)
    after_so2_current_active_state = execution_observer.last_execution_change_at_state
    print("#3 {0}".format(after_so2_current_active_state))

    current_sync_counter = state_machine_execution_engine.synchronization_counter
    testing_utils.call_gui_callback(menubar_ctrl.on_step_over_activate, None)
    wait_for_sync_counter_change_and_wait_for_gui(current_sync_counter)
    after_so3_current_active_state = execution_observer.last_execution_change_at_state
    print("#4 {0}".format(after_so3_current_active_state))

    assert not last_active_state == current_active_state
    assert len(after_so2_current_active_state.split('/')) == len(after_so1_current_active_state.split('/'))
    assert len(after_so3_current_active_state.split('/')) == len(after_so2_current_active_state.split('/'))

    execution_observer.relieve_model(sm_m)


if __name__ == '__main__':
    test_execution_modes(None)
    # import pytest
    # pytest.main(['-s', __file__])
