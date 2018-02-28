import testing_utils


def test_default():
    testing_utils.dummy_gui(None)
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.gui.models.container_state import ContainerStateModel
    parent_state = HierarchyState()
    parent_state_m = ContainerStateModel(parent_state)

    child_state_a = ExecutionState("A")

    parent_state.add_state(child_state_a)

    assert len(parent_state_m.states) == 1
    child_state_a_m = parent_state_m.states.values()[0]
    assert child_state_a_m.core_element is child_state_a


def test_single_future_element():
    testing_utils.dummy_gui(None)
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.gui.models.state import StateModel
    from rafcon.gui.models.container_state import ContainerStateModel
    parent_state = HierarchyState()
    parent_state_m = ContainerStateModel(parent_state)

    child_state_a = ExecutionState("A")
    child_state_a_m = StateModel(child_state_a)

    parent_state_m.expected_future_models.add(child_state_a_m)
    parent_state.add_state(child_state_a)

    assert len(parent_state_m.states) == 1
    new_child_state_a_m = parent_state_m.states.values()[0]
    assert new_child_state_a_m is child_state_a_m
    assert new_child_state_a_m.core_element is child_state_a
    assert len(parent_state_m.expected_future_models) == 0


def test_multiple_future_element():
    testing_utils.dummy_gui(None)
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.gui.models.state import StateModel
    from rafcon.gui.models.container_state import ContainerStateModel
    parent_state = HierarchyState()
    parent_state_m = ContainerStateModel(parent_state)

    child_state_a = ExecutionState("A")
    child_state_a_m = StateModel(child_state_a)
    child_state_b = ExecutionState("B")
    child_state_b_m = StateModel(child_state_b)

    parent_state_m.expected_future_models.add(child_state_a_m)
    parent_state_m.expected_future_models.add(child_state_b_m)

    parent_state.add_state(child_state_a)

    assert len(parent_state_m.states) == 1
    new_child_state_a_m = parent_state_m.states.values()[0]
    assert new_child_state_a_m is child_state_a_m
    assert new_child_state_a_m.core_element is child_state_a
    assert len(parent_state_m.expected_future_models) == 1

    parent_state.add_state(child_state_b)

    assert len(parent_state_m.states) == 2
    new_child_states_m = parent_state_m.states.values()
    assert new_child_states_m[0] is child_state_b_m or new_child_states_m[1] is child_state_b_m
    assert new_child_states_m[0].core_element is child_state_b or new_child_states_m[1].core_element is child_state_b
    assert len(parent_state_m.expected_future_models) == 0
