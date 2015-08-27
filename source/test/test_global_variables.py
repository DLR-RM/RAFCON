# core elements
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.state_machine import StateMachine

# singleton elements
import rafcon.statemachine.singleton

# test environment elements
import variables_for_pytest


def create_state_machine():
    state1 = ExecutionState("MyFirstState", path="../test_scripts", filename="global_variable_state.py")
    state1.add_outcome("first_outcome", 3)
    input_state1 = state1.add_input_data_port("input_data_port1", "float")
    output_state1 = state1.add_output_data_port("output_data_port1", "float")

    state3 = HierarchyState("MyFirstHierarchyState", path="../test_scripts", filename="hierarchy_state.py")
    state3.add_state(state1)
    state3.set_start_state(state1.state_id)
    state3.add_outcome("Container_Outcome", 6)
    state3.add_transition(state1.state_id, 3, state3.state_id, 6)
    input_state3 = state3.add_input_data_port("input_data_port1", "float", 22.0)
    output_state3 = state3.add_output_data_port("output_data_port1", "float")
    state3.add_data_flow(state3.state_id, input_state3, state1.state_id, input_state1)
    state3.add_data_flow(state1.state_id, output_state1, state3.state_id, output_state3)

    return StateMachine(state3)


def test_concurrency_barrier_state_execution():

    variables_for_pytest.test_multithrading_lock.acquire()
    sm = create_state_machine()
    root_state = sm.root_state
    state_machine = StateMachine(root_state)
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    rafcon.statemachine.singleton.state_machine_execution_engine.start()
    root_state.join()
    rafcon.statemachine.singleton.state_machine_execution_engine.stop()
    rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    variables_for_pytest.test_multithrading_lock.release()

    assert root_state.output_data["output_data_port1"] == 42
    print root_state.output_data


if __name__ == '__main__':
    #pytest.main()
    test_concurrency_barrier_state_execution()