import pytest

from rafcon.core.states.state import State
from rafcon.gui.models.state import StateModel

from rafcon.core.state_elements.transition import Transition
from rafcon.gui.models.transition import TransitionModel


@pytest.mark.parametrize("use_gaphas", [False, True])
def test_state_rel_pos(use_gaphas):
    state = State()
    state_m = StateModel(state, parent=None)
    state_m.meta["gui"]["editor_opengl" if use_gaphas else "editor_gaphas"]["rel_pos"] = (1, 2)
    meta_data = state_m.get_meta_data_editor(for_gaphas=use_gaphas)
    assert meta_data["rel_pos"] == (1, -2)


def test_state_income():
    state = State()
    state_m = StateModel(state, parent=None)
    state_m.meta["gui"]["editor_opengl"]["size"] = (50, 50)
    meta_data = state_m.get_meta_data_editor(for_gaphas=True)
    assert meta_data["income"]["rel_pos"] == (0, 25)


@pytest.mark.parametrize("use_gaphas", [False, True])
def test_transition_waypoints(use_gaphas):
    transition = Transition(None, 0, None, 0, None)
    transition_m = TransitionModel(transition, parent=None)
    transition_m.meta["gui"]["editor_opengl" if use_gaphas else "editor_gaphas"]["waypoints"] = [(1, 2), (-1, 3)]
    meta_data = transition_m.get_meta_data_editor(for_gaphas=use_gaphas)
    assert meta_data["waypoints"] == [(1, -2), (-1, -3)]
