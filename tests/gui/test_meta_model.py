import pytest


def assert_single_editor_meta_data(model, gaphas):
    editor_keys = model.meta["gui"].keys()
    if gaphas:
        assert 'editor_opengl' not in editor_keys
    else:
        assert 'editor_gaphas' not in editor_keys


def test_meta_initialization():
    from rafcon.gui.models.meta import MetaModel
    meta_m = MetaModel(meta={"1": 2, "2": 1})
    assert meta_m.meta == {"1": 2, "2": 1}

    meta_m = MetaModel(meta={'gui': {'editor_gaphas': {"1": 2, "2": 1}}})
    meta_data = meta_m.get_meta_data_editor()
    assert meta_data == {"1": 2, "2": 1}


def test_meta_setter_return_value():
    from rafcon.gui.models.meta import MetaModel
    meta_m = MetaModel()
    meta_data = meta_m.set_meta_data_editor("key", "value")
    assert meta_data["key"] == "value"

    meta_data = meta_m.set_meta_data_editor("key2", "value2")
    assert meta_data["key"] == "value"
    assert meta_data["key2"] == "value2"


@pytest.mark.parametrize("use_gaphas", [False, True])
def test_editor_setter_getter(use_gaphas):
    from rafcon.gui.models.meta import MetaModel
    meta_m = MetaModel()

    meta_data = meta_m.get_meta_data_editor(for_gaphas=use_gaphas)
    assert isinstance(meta_data, dict)
    assert len(meta_data) == 0

    meta_m.set_meta_data_editor("test_key", (1, 2), from_gaphas=use_gaphas)
    meta_data = meta_m.get_meta_data_editor(for_gaphas=use_gaphas)
    assert meta_data["test_key"] == (1, 2)

    meta_m.set_meta_data_editor("key1.key2", (2, 1), from_gaphas=use_gaphas)
    meta_data = meta_m.get_meta_data_editor(for_gaphas=use_gaphas)
    assert meta_data["key1"]["key2"] == (2, 1)

    assert_single_editor_meta_data(meta_m, gaphas=use_gaphas)

    assert meta_m.meta["gui"]["editor_gaphas" if use_gaphas else "editor_opengl"]["test_key"] == (1, 2)
    assert meta_m.meta["gui"]["editor_gaphas" if use_gaphas else "editor_opengl"]["key1"]["key2"] == (2, 1)


@pytest.mark.parametrize("use_gaphas", [False, True])
def test_editor_setter_getter_conversion(use_gaphas):
    from rafcon.gui.models.meta import MetaModel
    meta_m = MetaModel()
    meta_m.meta["gui"]["editor_opengl" if use_gaphas else "editor_gaphas"]["test"] = (1, 2)
    meta_data = meta_m.get_meta_data_editor(for_gaphas=use_gaphas)
    assert meta_data["test"] == (1, 2)

    assert_single_editor_meta_data(meta_m, gaphas=use_gaphas)
    assert meta_m.meta["gui"]["editor_gaphas" if use_gaphas else "editor_opengl"]["test"] == (1, 2)


def test_meta_list_modification():
    from rafcon.gui.models.meta import MetaModel
    meta_m = MetaModel()
    meta_data = meta_m.set_meta_data_editor("list", [1, 2, 3])
    assert meta_data["list"] == [1, 2, 3]
    meta_data = meta_m.set_meta_data_editor("list.0", 4)
    assert meta_data["list"] == [4, 2, 3]
