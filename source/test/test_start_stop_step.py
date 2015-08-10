from awesome_tool.statemachine.states.execution_state import ExecutionState
from awesome_tool.statemachine.states.hierarchy_state import HierarchyState
import awesome_tool.statemachine.singleton
from awesome_tool.statemachine.storage.storage import StateMachineStorage

from awesome_tool.statemachine.state_machine_manager import StateMachineManager
from awesome_tool.statemachine.state_machine import StateMachine
import variables_for_pytest

import pytest
import time


def return_loop_state_machine():
    state1 = ExecutionState("MyFirstState", path="../test_scripts", filename="loop_state1.py")
    state1.add_outcome("MyFirstOutcome", 3)

    state2 = ExecutionState("MySecondState", path="../test_scripts", filename="loop_state2.py")
    state2.add_outcome("FirstOutcome", 3)

    state3 = HierarchyState("MyFirstHierarchyState", path="../test_scripts", filename="hierarchy_state.py")
    state3.add_state(state1)
    state3.add_state(state2)
    state3.set_start_state(state1.state_id)
    state3.add_transition(state1.state_id, 3, state2.state_id, None)
    state3.add_transition(state2.state_id, 3, state1.state_id, None)

    return StateMachine(state3)


def test_start_stop_pause_step():
    # from mvc.models import ContainerStateModel
    # from mvc.views.single_widget_window import TestButtonsView
    # import gtk

    sm = return_loop_state_machine()
    awesome_tool.statemachine.singleton.global_variable_manager.set_variable("counter", 0)

    s = StateMachineStorage("../test_scripts/stored_statemachine")
    s.save_statemachine_as_yaml(sm, "../test_scripts/stored_statemachine")
    sm_loaded, version, creation_time = s.load_statemachine_from_yaml()

    # ctr_model = ContainerStateModel(root_state)
    # test_buttons_view = TestButtonsView(ctr_model)

    #root_state.daemon = True

    # care for old statemachines that were started in previous tests
    my_state_machine_manager = StateMachineManager()

    variables_for_pytest.test_multithrading_lock.acquire()
    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(sm_loaded)
    awesome_tool.statemachine.singleton.state_machine_manager.active_state_machine_id = sm_loaded.state_machine_id
    awesome_tool.statemachine.singleton.state_machine_execution_engine.step_mode()

    for i in range(5):
        time.sleep(0.2)
        awesome_tool.statemachine.singleton.state_machine_execution_engine.step()

    # give the state machine time to execute
    time.sleep(0.2)
    awesome_tool.statemachine.singleton.state_machine_execution_engine.stop()
    sm_loaded.root_state.join()

    assert awesome_tool.statemachine.singleton.global_variable_manager.get_variable("counter") == 5
    awesome_tool.statemachine.singleton.state_machine_manager.remove_state_machine(sm_loaded.state_machine_id)
    variables_for_pytest.test_multithrading_lock.release()
    # gtk.main()

if __name__ == '__main__':
    #pytest.main()
    test_start_stop_pause_step()