import pytest
import testing_utils


@pytest.mark.parametrize("use_gaphas", [False, True])
def test_state_rel_pos(use_gaphas):
    testing_utils.dummy_gui(None)
    from rafcon.core.states.state import State
    from rafcon.gui.models.state import StateModel
    state = State()
    state_m = StateModel(state, parent=None)
    state_m.meta["gui"]["editor_opengl" if use_gaphas else "editor_gaphas"]["rel_pos"] = (1, 2)
    meta_data = state_m.get_meta_data_editor(for_gaphas=use_gaphas)
    assert meta_data["rel_pos"] == (1, -2)


def test_name():
    testing_utils.dummy_gui(None)
    from rafcon.core.states.state import State
    from rafcon.gui.models.state import StateModel
    state = State()
    state_m = StateModel(state, parent=None)
    state_m.meta["gui"]["editor_opengl"]["size"] = (96, 150)
    meta_data = state_m.get_meta_data_editor(for_gaphas=True)
    assert meta_data["name"]["rel_pos"] == (8, 8)
    assert meta_data["name"]["size"] == (80, 12)


def test_income():
    testing_utils.dummy_gui(None)
    from rafcon.core.states.state import State
    from rafcon.gui.models.state import StateModel
    state = State()
    state_m = StateModel(state, parent=None)
    state_m.meta["gui"]["editor_opengl"]["size"] = (50, 50)
    meta_data = state_m.get_meta_data_editor(for_gaphas=True)
    assert meta_data["income"]["rel_pos"] == (0, 25)


def test_state_property_deletion():
    testing_utils.dummy_gui(None)
    from rafcon.core.states.state import State
    from rafcon.gui.models.state import StateModel
    state = State()
    state_m = StateModel(state, parent=None)
    state_m.meta["gui"]["editor_gaphas"]["income"]["rel_pos"] = (0, 50)
    state_m.meta["gui"]["editor_gaphas"]["name"]["rel_pos"] = (10, 10)
    state_m.meta["gui"]["editor_gaphas"]["name"]["size"] = (200, 100)
    meta_data = state_m.get_meta_data_editor(for_gaphas=False)
    assert "income" not in meta_data
    assert "income" not in state_m.meta["gui"]["editor_gaphas"]
    assert "income" not in state_m.meta["gui"]["editor_opengl"]
    assert "name" not in meta_data
    assert "name" not in state_m.meta["gui"]["editor_gaphas"]
    assert "name" not in state_m.meta["gui"]["editor_opengl"]


def test_3_outcomes():
    testing_utils.dummy_gui(None)
    from rafcon.core.states.state import State
    from rafcon.gui.models.state import StateModel
    state = State()
    state_m = StateModel(state, parent=None)
    state_m.meta["gui"]["editor_opengl"]["size"] = (60, 60)
    state_m.get_meta_data_editor(for_gaphas=True)

    for outcome_m in state_m.outcomes:
        outcome = outcome_m.outcome
        outcome_pos = outcome_m.get_meta_data_editor()["rel_pos"]
        if outcome.outcome_id == -1:
            assert outcome_pos == (54, 0)
        elif outcome.outcome_id == -2:
            assert outcome_pos == (42, 0)
        else:
            assert outcome_pos == (60, 30)


def test_4_outcomes():
    testing_utils.dummy_gui(None)
    from rafcon.core.states.state import State
    from rafcon.gui.models.state import StateModel
    state = State()
    state.add_outcome("success2")
    state_m = StateModel(state, parent=None)
    state_m.meta["gui"]["editor_opengl"]["size"] = (90, 90)
    state_m.get_meta_data_editor(for_gaphas=True)

    for outcome_m in state_m.outcomes:
        outcome = outcome_m.outcome
        outcome_pos = outcome_m.get_meta_data_editor()["rel_pos"]
        if outcome.outcome_id == -1:
            assert outcome_pos == (81, 0)
        elif outcome.outcome_id == -2:
            assert outcome_pos == (63, 0)
        elif outcome.outcome_id == 0:
            assert outcome_pos == (90, 30)
        else:
            assert outcome_pos == (90, 60)


def test_input_opengl2gaphas():
    testing_utils.dummy_gui(None)
    from rafcon.core.states.state import State
    from rafcon.gui.models.state import StateModel
    state = State()
    state.add_input_data_port("in", int, 0)
    state_m = StateModel(state, parent=None)
    state_m.meta["gui"]["editor_opengl"]["size"] = (100, 100)
    state_m.get_meta_data_editor(for_gaphas=True)
    input_m = state_m.input_data_ports[0]
    input_m.meta["gui"]["editor_opengl"]["inner_rel_pos"] = (20, -30)
    rel_pos = input_m.get_meta_data_editor(for_gaphas=True)["rel_pos"]
    assert rel_pos == (0, 30)


def test_input_gaphas2opengl():
    testing_utils.dummy_gui(None)
    from rafcon.core.states.state import State
    from rafcon.gui.models.state import StateModel
    state = State()
    state.add_input_data_port("in", int, 0)
    state_m = StateModel(state, parent=None)
    state_m.meta["gui"]["editor_gaphas"]["size"] = (100, 100)
    state_m.get_meta_data_editor(for_gaphas=False)
    input_m = state_m.input_data_ports[0]
    input_m.meta["gui"]["editor_gaphas"]["rel_pos"] = (0, 30)
    rel_pos = input_m.get_meta_data_editor(for_gaphas=False)["inner_rel_pos"]
    assert rel_pos == (0, -30)


def test_output_opengl2gaphas():
    testing_utils.dummy_gui(None)
    from rafcon.core.states.state import State
    from rafcon.gui.models.state import StateModel
    state = State()
    state.add_output_data_port("out", int, 0)
    state_m = StateModel(state, parent=None)
    state_m.meta["gui"]["editor_opengl"]["size"] = (100, 100)
    state_m.get_meta_data_editor(for_gaphas=True)
    output_m = state_m.output_data_ports[0]
    output_m.meta["gui"]["editor_opengl"]["inner_rel_pos"] = (20, -30)
    rel_pos = output_m.get_meta_data_editor(for_gaphas=True)["rel_pos"]
    assert rel_pos == (100, 30)


def test_output_gaphas2opengl():
    testing_utils.dummy_gui(None)
    from rafcon.core.states.state import State
    from rafcon.gui.models.state import StateModel
    state = State()
    state.add_output_data_port("out", int, 0)
    state_m = StateModel(state, parent=None)
    state_m.meta["gui"]["editor_gaphas"]["size"] = (100, 100)
    state_m.get_meta_data_editor(for_gaphas=False)
    output_m = state_m.output_data_ports[0]
    output_m.meta["gui"]["editor_gaphas"]["rel_pos"] = (100, 50)
    rel_pos = output_m.get_meta_data_editor(for_gaphas=False)["inner_rel_pos"]
    assert rel_pos == (100, -50)


def test_scoped_variable_opengl2gaphas():
    testing_utils.dummy_gui(None)
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.gui.models.container_state import ContainerStateModel
    state = HierarchyState()
    state.add_scoped_variable("sv", int, 0)
    state_m = ContainerStateModel(state, parent=None)
    state_m.meta["gui"]["editor_opengl"]["size"] = (100, 100)
    state_m.get_meta_data_editor(for_gaphas=True)
    scoped_var_m = state_m.scoped_variables[0]
    scoped_var_m.meta["gui"]["editor_opengl"]["inner_rel_pos"] = (70, 30)
    rel_pos = scoped_var_m.get_meta_data_editor(for_gaphas=True)["rel_pos"]
    assert rel_pos == (70, 0)


def test_scoped_variable_gaphas2opengl():
    testing_utils.dummy_gui(None)
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.gui.models.container_state import ContainerStateModel
    state = HierarchyState()
    state.add_scoped_variable("sv", int, 0)
    state_m = ContainerStateModel(state, parent=None)
    state_m.meta["gui"]["editor_gaphas"]["size"] = (100, 100)
    state_m.get_meta_data_editor(for_gaphas=False)
    scoped_var_m = state_m.scoped_variables[0]
    scoped_var_m.meta["gui"]["editor_gaphas"]["rel_pos"] = (70, 0)
    rel_pos = scoped_var_m.get_meta_data_editor(for_gaphas=False)["inner_rel_pos"]
    assert rel_pos == (70, 0)


@pytest.mark.parametrize("use_gaphas", [False, True])
def test_transition_waypoints(use_gaphas):
    testing_utils.dummy_gui(None)
    from rafcon.core.state_elements.transition import Transition
    from rafcon.gui.models.transition import TransitionModel
    transition = Transition(None, 0, None, 0, None)
    transition_m = TransitionModel(transition, parent=None)
    transition_m.meta["gui"]["editor_opengl" if use_gaphas else "editor_gaphas"]["waypoints"] = [(1, 2), (-1, 3)]
    meta_data = transition_m.get_meta_data_editor(for_gaphas=use_gaphas)
    assert meta_data["waypoints"] == [(1, -2), (-1, -3)]
