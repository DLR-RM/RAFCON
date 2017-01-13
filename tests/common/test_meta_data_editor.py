import pytest

from rafcon.gui.models.state_element import StateElementModel


def assert_single_editor_meta_data(model, gaphas):
    editor_keys = model.meta["gui"].keys()
    if gaphas:
        assert 'editor_opengl' not in editor_keys
    else:
        assert 'editor_gaphas' not in editor_keys


@pytest.mark.parametrize("use_gaphas", [False, True])
def test_state_element(use_gaphas):
    state_element_m = StateElementModel(parent=None, meta=None)

    meta_data = state_element_m.get_meta_data_editor(for_gaphas=use_gaphas)
    assert isinstance(meta_data, dict)
    assert len(meta_data) == 0

    state_element_m.set_meta_data_editor("test_key", (1, 2), from_gaphas=use_gaphas)
    meta_data = state_element_m.get_meta_data_editor(for_gaphas=use_gaphas)
    assert meta_data["test_key"] == (1, 2)

    state_element_m.set_meta_data_editor("key1.key2", (2, 1), from_gaphas=use_gaphas)
    meta_data = state_element_m.get_meta_data_editor(for_gaphas=use_gaphas)
    assert meta_data["key1"]["key2"] == (2, 1)

    assert_single_editor_meta_data(state_element_m, gaphas=use_gaphas)

    assert state_element_m.meta["gui"]["editor_gaphas" if use_gaphas else "editor_opengl"]["test_key"] == (1, 2)
    assert state_element_m.meta["gui"]["editor_gaphas" if use_gaphas else "editor_opengl"]["key1"]["key2"] == (2, 1)


@pytest.mark.parametrize("use_gaphas", [False, True])
def test_state_element_conversion(use_gaphas):
    state_element_m = StateElementModel(parent=None, meta=None)
    state_element_m.meta["gui"]["editor_opengl" if use_gaphas else "editor_gaphas"]["test"] = (1, 2)
    meta_data = state_element_m.get_meta_data_editor(for_gaphas=use_gaphas)
    assert meta_data["test"] == (1, 2)

    assert_single_editor_meta_data(state_element_m, gaphas=use_gaphas)
    assert state_element_m.meta["gui"]["editor_gaphas" if use_gaphas else "editor_opengl"]["test"] == (1, 2)

