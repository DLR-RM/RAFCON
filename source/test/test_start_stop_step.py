from statemachine.states.execution_state import ExecutionState
from statemachine.states.hierarchy_state import HierarchyState
import statemachine.singleton
from statemachine.storage.storage import Storage

from statemachine.execution.statemachine_execution_engine import StatemachineExecutionEngine
from statemachine.state_machine_manager import StateMachineManager
from statemachine.state_machine import StateMachine
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
    return state3


def test_start_stop_pause_step():
    # from mvc.models import ContainerStateModel
    # from mvc.views.single_widget_window import TestButtonsView
    # import gtk

    state3 = return_loop_state_machine()
    statemachine.singleton.global_variable_manager.set_variable("counter", 0)

    s = Storage("../test_scripts/stored_statemachine")
    s.save_statemachine_as_yaml(state3)
    root_state, version, creation_time = s.load_statemachine_from_yaml()

    # ctr_model = ContainerStateModel(root_state)
    # test_buttons_view = TestButtonsView(ctr_model)

    #root_state.daemon = True

    # care for old statemachines that were started in previous tests
    my_state_machine_manager = StateMachineManager()
    state_machine = StateMachine(root_state)

    variables_for_pytest.test_multithrading_lock.acquire()
    statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    statemachine.singleton.state_machine_execution_engine.step_mode()

    for i in range(5):
        time.sleep(0.1)
        statemachine.singleton.state_machine_execution_engine.step()

    statemachine.singleton.state_machine_execution_engine.stop()
    root_state.join()

    assert statemachine.singleton.global_variable_manager.get_variable("counter") == 5
    variables_for_pytest.test_multithrading_lock.release()
    # gtk.main()

if __name__ == '__main__':
    #pytest.main()
    test_start_stop_pause_step()