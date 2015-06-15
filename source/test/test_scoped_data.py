import pytest

from awesome_tool.statemachine.states.execution_state import ExecutionState
from awesome_tool.statemachine.states.hierarchy_state import HierarchyState
from awesome_tool.statemachine.states.state import DataPortType
from awesome_tool.statemachine.storage.storage import StateMachineStorage
import awesome_tool.statemachine.singleton
from awesome_tool.statemachine.state_machine import StateMachine
import variables_for_pytest


def create_statemachine():
    state1 = ExecutionState("first_state", path="../test_scripts", filename="scoped_data_test_state1.py")
    state1.add_outcome("first_outcome", 3)
    state1.add_input_data_port("data_input_port1", "float")
    state1.add_output_data_port("data_output_port1", "float")

    state2 = ExecutionState("second_state", path="../test_scripts", filename="scoped_data_test_state2.py")
    state2.add_outcome("first_outcome", 3)
    state2.add_input_data_port("data_input_port1", "float")
    state2.add_output_data_port("data_output_port1", "float")

    state3 = HierarchyState("hierarchy_state", path="../test_scripts", filename="hierarchy_state.py")
    state3.add_state(state1)
    state3.add_state(state2)
    state3.set_start_state(state1.state_id)
    state3.add_outcome("Container_Outcome", 6)
    state3.add_transition(state1.state_id, 3, state2.state_id, None)
    state3.add_transition(state2.state_id, 3, state3.state_id, 6)
    state3.add_input_data_port("data_input_port1", "float", 22.0)
    state3.add_output_data_port("data_output_port1", "float")
    state3.add_data_flow(state3.state_id,
                         state3.get_io_data_port_id_from_name_and_type("data_input_port1", DataPortType.INPUT),
                         state1.state_id,
                         state1.get_io_data_port_id_from_name_and_type("data_input_port1", DataPortType.INPUT))
    state3.add_data_flow(state1.state_id,
                         state1.get_io_data_port_id_from_name_and_type("data_output_port1", DataPortType.OUTPUT),
                         state2.state_id,
                         state2.get_io_data_port_id_from_name_and_type("data_input_port1", DataPortType.INPUT))
    state3.add_data_flow(state2.state_id,
                         state2.get_io_data_port_id_from_name_and_type("data_output_port1", DataPortType.OUTPUT),
                         state3.state_id,
                         state3.get_io_data_port_id_from_name_and_type("data_output_port1", DataPortType.OUTPUT))
    return StateMachine(state3)


# remember: scoped data is all data in a container state (including input_data, scoped variables and outputs of child
# states)
def test_scoped_data():
    s = StateMachineStorage("../test_scripts/stored_statemachine")

    sm = create_statemachine()

    s.save_statemachine_as_yaml(sm, "../test_scripts/stored_statemachine")
    [sm_loaded, version, creation_time] = s.load_statemachine_from_yaml()

    state_machine = StateMachine(sm_loaded.root_state)

    variables_for_pytest.test_multithrading_lock.acquire()
    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    awesome_tool.statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    awesome_tool.statemachine.singleton.state_machine_execution_engine.start()
    sm_loaded.root_state.join()
    awesome_tool.statemachine.singleton.state_machine_execution_engine.stop()
    awesome_tool.statemachine.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    variables_for_pytest.test_multithrading_lock.release()

    assert state_machine.root_state.output_data["data_output_port1"] == 42.0

if __name__ == '__main__':
    #pytest.main()
    test_scoped_data()
