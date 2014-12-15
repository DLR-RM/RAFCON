import time

from statemachine.states.hierarchy_state import HierarchyState
from states.execution_state import ExecutionState
from statemachine.states.barrier_concurrency_state import BarrierConcurrencyState
from statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState


def concurrency_barrier_test():
    state1 = ExecutionState("FirstState", path="../../test_scripts", filename="concurrence_barrier1.py")
    state1.add_outcome("FirstOutcome", 3)
    state1.add_input_key("FirstDataInputPort", "str")
    state1.add_output_key("FirstDataOutputPort", "float")

    state2 = ExecutionState("SecondState", path="../../test_scripts", filename="concurrence_barrier2.py")
    state2.add_outcome("FirstOutcome", 3)
    state2.add_input_key("FirstDataInputPort", "str")
    state2.add_output_key("FirstDataOutputPort", "float")

    state3 = BarrierConcurrencyState("FirstConcurrencyState", path="../../test_scripts",
                                     filename="concurrency_container.py")
    state3.add_state(state1)
    state3.add_data_flow(state3, "in1", state1, "FirstDataInputPort")
    state3.add_state(state2)
    state3.add_data_flow(state3, "in1", state2, "FirstDataInputPort")

    input_data = {"in1": "input_string", "in2": 2}
    output_data = {"out1": None, "out2": None}
    state3.input_data = input_data
    state3.output_data = output_data
    state3.start()
    state3.join()

def concurrency_preemption_test():
    state1 = ExecutionState("FirstState", path="../../test_scripts", filename="concurrence_preemption1.py")
    state1.add_outcome("FirstOutcome", 3)
    state1.add_input_key("FirstDataInputPort", "str")
    state1.add_output_key("FirstDataOutputPort", "float")

    state2 = ExecutionState("SecondState", path="../../test_scripts", filename="concurrence_preemption2.py")
    state2.add_outcome("FirstOutcome", 3)
    state2.add_input_key("FirstDataInputPort", "str")
    state2.add_output_key("FirstDataOutputPort", "float")

    state3 = PreemptiveConcurrencyState("FirstConcurrencyState", path="../../test_scripts",
                                        filename="concurrency_container.py")
    state3.add_state(state1)
    state3.add_data_flow(state3, "in1", state1, "FirstDataInputPort")
    state3.add_state(state2)
    state3.add_data_flow(state3, "in1", state2, "FirstDataInputPort")

    input_data = {"in1": "input_string", "in2": 2}
    output_data = {"out1": None, "out2": None}
    state3.input_data = input_data
    state3.output_data = output_data
    state3.start()
    state3.join()


def hierarchy_test():
    state1 = ExecutionState("MyFirstState", path="../../test_scripts", filename="first_state.py")
    state1.add_outcome("MyFirstOutcome", 3)
    state1.add_input_key("MyFirstDataInputPort", "str")
    state1.add_output_key("MyFirstDataOutputPort", "float")

    state3 = HierarchyState("MyFirstHierarchyState", path="../../test_scripts", filename="hierarchy_container.py")
    state3.add_state(state1)
    state3.set_start_state(state1)
    state3.add_outcome("Container_Outcome", 6)
    state3.add_transition(state1.state_id, 3, None, 6)
    state3.add_data_flow(state3, "in1", state1, "MyFirstDataInputPort")
    input_data = {"in1": "input_string", "in2": 2}
    output_data = {"out1": None, "out2": None}
    state3.input_data = input_data
    state3.output_data = output_data
    state3.start()
    #time.sleep(1.0)
    #state3.preempted = True
    state3.join()
    #print "joined thread"

if __name__ == '__main__':

    #hierarchy_test()
    #concurrency_test()
    concurrency_preemption_test()