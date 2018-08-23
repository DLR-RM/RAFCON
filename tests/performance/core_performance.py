# core elements
import rafcon.core.singleton
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from rafcon.core.constants import UNIQUE_DECIDER_STATE_ID
from rafcon.core.state_elements.data_port import InputDataPort, OutputDataPort
from rafcon.core.state_machine import StateMachine

from rafcon.utils.timer import measure_time

import testing_utils


@measure_time
def create_hierarchy_state(number_child_states=10, sleep=False):
    hierarchy = HierarchyState("hierarchy1")
    hierarchy.add_outcome("hierarchy_outcome", 1)
    hierarchy.add_input_data_port("hierarchy_input_port1", "float", 42.0)
    hierarchy.add_output_data_port("hierarchy_output_port1", "float")
    last_state = None

    for i in range(number_child_states):
        if sleep:
            state = ExecutionState("state" + str(i), path=testing_utils.TEST_SCRIPT_PATH, filename="hello_world_sleep.py")
        else:
            state = ExecutionState("state" + str(i))
        hierarchy.add_state(state)
        state.add_input_data_port("input1", "float")
        state.add_output_data_port("output1", "float")

        if not last_state:
            hierarchy.set_start_state(state.state_id)
            hierarchy.add_data_flow(hierarchy.state_id,
                                    hierarchy.get_io_data_port_id_from_name_and_type("hierarchy_input_port1",
                                                                                     InputDataPort),
                                    state.state_id,
                                    state.get_io_data_port_id_from_name_and_type("input1", InputDataPort))
        else:
            hierarchy.add_transition(last_state.state_id, 0, state.state_id, None)
            # connect data ports state 1
            hierarchy.add_data_flow(last_state.state_id,
                                 last_state.get_io_data_port_id_from_name_and_type("output1", OutputDataPort),
                                 state.state_id,
                                 state.get_io_data_port_id_from_name_and_type("input1", InputDataPort))

        last_state = state

    hierarchy.add_data_flow(last_state.state_id,
                            last_state.get_io_data_port_id_from_name_and_type("output1",
                                                                              OutputDataPort),
                            hierarchy.state_id,
                            hierarchy.get_io_data_port_id_from_name_and_type("hierarchy_output_port1",
                                                                             OutputDataPort))

    hierarchy.add_transition(last_state.state_id, 0, hierarchy.state_id, 1)

    return hierarchy


@measure_time
def create_barrier_concurrency_state(number_child_states=10, number_childs_per_child=10):
    barrier_state = BarrierConcurrencyState("barrier_concurrency")

    for i in range(number_child_states):
        hierarchy_state = create_hierarchy_state(number_childs_per_child)
        barrier_state.add_state(hierarchy_state)

    barrier_state.add_transition(barrier_state.states[UNIQUE_DECIDER_STATE_ID].state_id, 0, barrier_state.state_id, 0)
    return barrier_state


def execute_state(root_state):
    state_machine = StateMachine(root_state)
    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    rafcon.core.singleton.state_machine_execution_engine.start()
    rafcon.core.singleton.state_machine_execution_engine.join()
    rafcon.core.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)


@measure_time
def test_hierarchy_state_execution(number_child_states=100):
    hierarchy_state = create_hierarchy_state(number_child_states)
    execute_state(hierarchy_state)


@measure_time
def test_barrier_concurrency_state_execution(number_child_states=10, number_childs_per_child=10):
    barrier_state = create_barrier_concurrency_state(number_child_states, number_childs_per_child)
    execute_state(barrier_state)


@measure_time
def test_preemption_concurrency_state_execution(number_child_states=10, number_childs_per_child=10,
                                                number_of_childs_fast_state=3):

    preemption_state = PreemptiveConcurrencyState("preemption_concurrency")

    for i in range(number_child_states):
        hierarchy_state = create_hierarchy_state(number_childs_per_child)
        preemption_state.add_state(hierarchy_state)
        # preemption_state.add_transition(hierarchy_state.state_id, 0, preemption_state.state_id, 0)

    # add fast state
    hierarchy_state = create_hierarchy_state(number_of_childs_fast_state)
    preemption_state.add_state(hierarchy_state)
    preemption_state.add_transition(hierarchy_state.state_id, 1, preemption_state.state_id, 0)

    execute_state(preemption_state)


if __name__ == '__main__':
    # test_hierarchy_state_execution(10)
    test_hierarchy_state_execution(100)
    # TODO: state creation takes too long (> 100 seconds) => investigate
    # test_hierarchy_state_execution(1000)
    # test_barrier_concurrency_state_execution(10, 10)
    # test_barrier_concurrency_state_execution(100, 100)
    # test_preemption_concurrency_state_execution(50, 20, 3)
