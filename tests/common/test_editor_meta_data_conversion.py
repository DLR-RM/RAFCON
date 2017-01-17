import pytest

from rafcon.core.states.state import State
from rafcon.gui.models.state import StateModel


@pytest.mark.parametrize("use_gaphas", [False, True])
def test_state_rel_pos(use_gaphas):
    state = State()
    meta_m = StateModel(state, parent=None)
    meta_m.meta["gui"]["editor_opengl" if use_gaphas else "editor_gaphas"]["rel_pos"] = (1, 2)
    meta_data = meta_m.get_meta_data_editor(for_gaphas=use_gaphas)
    assert meta_data["rel_pos"] == (1, -2)
