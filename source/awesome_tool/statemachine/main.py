from statemachine.states.hierarchy_state import HierarchyState
from states.execution_state import ExecutionState


if __name__ == '__main__':
    state1 = ExecutionState("MyFirstState")
    state1.add_outcome("MyFirstOutcome", 5)
    state1.add_output_key("MyFirstDataOutpuPort", "float")
    #state2 = ExecutionState()
    #state2.start()

    state3 = HierarchyState()
    state3.add_state(state1)
    state3.add_outcome("Container_Outcome", 6)
    state3.add_transition(state1.state_id, 5, None, 6)
    state3.set_start_state(state1.state_id)
    input_data = {"test": 1, "test2": 2}
    output_data = {"test": 10, "test2": 11}
    state3.run(inputs=input_data, outputs=output_data)
