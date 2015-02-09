import pytest

from statemachine.states.execution_state import ExecutionState
from statemachine.states.hierarchy_state import HierarchyState
from statemachine.states.state import StateType, DataPortType
from statemachine.storage.storage import Storage
import statemachine.singleton


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
    state3.add_transition(state2.state_id, 3, None, 6)
    state3.add_input_data_port("data_input_port1", "float")
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
    return state3


# remember: scoped data is all data in a container state (including input_data, scoped variables and outputs of child
# states)
def test_scoped_data():
    s = Storage("../../test_scripts/stored_statemachine")

    state3 = create_statemachine()

    s.save_statemachine_as_yaml(state3)
    [root_state, version, creation_time] = s.load_statemachine_from_yaml()

    input_data = {"data_input_port1": 22.0}
    output_data = {"data_output_port1": None}
    root_state.input_data = input_data
    root_state.output_data = output_data

    statemachine.singleton.state_machine_manager.root_state = root_state
    statemachine.singleton.state_machine_execution_engine.start()
    root_state.join()
    statemachine.singleton.state_machine_execution_engine.stop()

    #print output_data["data_output_port1"]
    assert output_data["data_output_port1"] == 42.0

if __name__ == '__main__':
    pytest.main()
    #test_scoped_data()
