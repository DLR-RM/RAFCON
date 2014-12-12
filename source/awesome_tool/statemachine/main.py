from statemachine.states.hierarchy_state import HierarchyState
from states.execution_state import ExecutionState


if __name__ == '__main__':
    state1 = ExecutionState("MyFirstState", path="../../test_scripts", filename="test_module.py")
    state1.add_outcome("MyFirstOutcome", 5)
    state1.add_input_key("MyFirstDataInputPort", "str")
    state1.add_output_key("MyFirstDataOutputPort", "float")
    #state2 = ExecutionState()
    #state2.start()

    state3 = HierarchyState()
    state3.add_state(state1)
    state3.add_outcome("Container_Outcome", 6)
    state3.add_transition(state1.state_id, 5, None, 6)
    state3.set_start_state(state1.state_id)
    state3.add_data_flow(state3, "in1", state1, "MyFirstDataInputPort")
    input_data = {"in1": "input_string", "in2": 2}
    output_data = {"out1": None, "out2": None}
    state3.run(inputs=input_data, outputs=output_data)
