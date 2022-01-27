import os
import time
import pytest

from tests import utils as testing_utils
from tests.utils import call_gui_callback, wait_for_gui


sm_path_recursive_resize = os.path.join(testing_utils.TEST_ASSETS_PATH, "unit_test_state_machines", "recursive_resize")

state_path_root = "YCBQQV"
state_path_P = "YCBQQV/IZCVSG"
state_path_PC = "YCBQQV/IZCVSG/PAMWNB"
state_path_e = "YCBQQV/IZCVSG/PAMWNB/YRGGWX"
state_path_A = "YCBQQV/IZCVSG/QUVLJG"
state_path_Hi = "YCBQQV/EUPXUC"
state_path_Ex = "YCBQQV/PBUVVY"

config_options = {
"gui_config":  {
    'HISTORY_ENABLED': True,
    'GAPHAS_EDITOR_AUTO_FOCUS_OF_ROOT_STATE': False
},
# If the GUI widget becomes too small, the resize tests will fail; thus, all sidebars are hidden in order
# that the gui will have enough space
"runtime_config": {
    'MAIN_WINDOW_MAXIMIZED': False,
    'MAIN_WINDOW_SIZE': (1500, 800),
    'MAIN_WINDOW_POS': (0, 0),
    'LEFT_BAR_DOCKED_POS': 400,
    'RIGHT_BAR_DOCKED_POS': 800,
    'CONSOLE_DOCKED_POS': 600,
    'LEFT_BAR_WINDOW_UNDOCKED': False,
    'RIGHT_BAR_WINDOW_UNDOCKED': False,
    'CONSOLE_WINDOW_UNDOCKED': False,
    'LEFT_BAR_HIDDEN': False,
    'RIGHT_BAR_HIDDEN': False,
    'CONSOLE_HIDDEN': False
}}


def open_test_state_machine(gui):
    import rafcon.gui.singleton

    smm_m = rafcon.gui.singleton.state_machine_manager_model
    main_window_controller = rafcon.gui.singleton.main_window_controller
    menubar_ctrl = main_window_controller.menu_bar_controller
    state_machines_ctrl = main_window_controller.state_machines_editor_ctrl

    gui(menubar_ctrl.on_open_activate, None, None, sm_path_recursive_resize)
    time.sleep(0.5)
    gui(wait_for_gui)  # Wait for gaphas view

    sm_m = smm_m.state_machines[smm_m.selected_state_machine_id]
    sm_id = sm_m.state_machine.state_machine_id
    sm_gaphas_ctrl = state_machines_ctrl.get_controller(sm_id)
    canvas = sm_gaphas_ctrl.canvas
    gaphas_view = sm_gaphas_ctrl.view.editor

    return sm_m, canvas, gaphas_view


def get_state_handle_pos(view, state_v, handle):
    i2v = view.get_matrix_i2v(state_v)
    item_pos_handle = (handle.pos.x.value, handle.pos.y.value)
    view_pos_handle = i2v.transform_point(*item_pos_handle)
    return view_pos_handle


def resize_state(gui, view, state_v, rel_size, num_motion_events, recursive, monkeypatch):
    from gi.repository import Gdk
    from rafcon.gui.mygaphas.tools import MoveHandleTool
    from rafcon.gui.utils.constants import RECURSIVE_RESIZE_MODIFIER
    from gaphas.item import SE, NW

    def get_resize_handle(x, y, distance=None):
        return state_v, state_v.handles()[SE]

    monkeypatch.setattr("rafcon.gui.mygaphas.aspect.StateHandleFinder.get_handle_at_point", get_resize_handle)
    monkeypatch.setattr("rafcon.gui.mygaphas.aspect.ItemHandleFinder.get_handle_at_point", get_resize_handle)
    # Deactivate guides (snapping)
    monkeypatch.setattr("rafcon.gui.mygaphas.guide.GuidedStateMixin.MARGIN", 0)

    resize_tool = gui(MoveHandleTool, view)
    start_pos_handle = gui(get_state_handle_pos, view, state_v, state_v.handles()[SE])

    # Start resize: Press button
    # Gtk TODO: Check if button can be set like this
    button_press_event = Gdk.Event.new(type=Gdk.EventType.BUTTON_PRESS)
    button_press_event.button = 1
    gui(resize_tool.on_button_press, button_press_event)
    # Do resize: Move mouse
    motion_event = Gdk.Event.new(Gdk.EventType.MOTION_NOTIFY)
    motion_event.state = motion_event.get_state()[1] | Gdk.EventMask.BUTTON_PRESS_MASK
    if recursive:
        motion_event.state = motion_event.get_state()[1] | RECURSIVE_RESIZE_MODIFIER
    print("\nsent motion_event.state", motion_event.get_state())
    for i in range(num_motion_events):
        motion_event.x = start_pos_handle[0] + rel_size[0] * (float(i + 1) / num_motion_events)
        motion_event.y = start_pos_handle[1] + rel_size[1] * (float(i + 1) / num_motion_events)
        gui(resize_tool.on_motion_notify, motion_event)

    # Stop resize: Release button
    # Gtk TODO: Check if button can be set like this
    button_release_event = Gdk.Event.new(type=Gdk.EventType.BUTTON_RELEASE)
    button_release_event.button = 1
    gui(resize_tool.on_button_release, button_release_event)

    monkeypatch.undo()
    monkeypatch.undo()
    monkeypatch.undo()


def assert_state_size_and_meta_data_consistency(state_m, state_v, size, canvas):
    from rafcon.utils.geometry import equal
    from rafcon.gui.helpers.meta_data import check_gaphas_state_meta_data_consistency
    assert equal(size, (state_v.width, state_v.height), 5), "State {}: view size wrong".format(state_m.state.name)
    assert equal(size, state_m.get_meta_data_editor()["size"], 5), "State {}: meta size wrong".format(state_m.state.name)
    check_gaphas_state_meta_data_consistency(state_m, canvas, recursive=True)


def add_vectors(vec1, vec2):
    return [v1 + v2 for v1, v2 in zip(vec1, vec2)]


def transform_size_v2i(view, state_v, size):
    v2i = view.get_matrix_v2i(state_v)
    item_size = v2i.transform_distance(*size)
    return item_size


def print_state_sizes(state_m, canvas, state_names=None):
    from rafcon.core.states.container_state import ContainerState
    state_v = canvas.get_view_for_model(state_m)
    meta_size = state_m.get_meta_data_editor()["size"]
    view_size = state_v.width, state_v.height
    meta_pos = state_m.get_meta_data_editor()["rel_pos"]
    view_pos = state_v.position
    if state_names is None or state_m.state.name in state_names:
        print("{} size: {} ?= {}".format(state_m.state.name, meta_size, view_size))
        print("{} pos: {} ?= {}".format(state_m.state.name, meta_pos, view_pos))
    if isinstance(state_m.state, ContainerState):
        for child_state_m in state_m.states.values():
            print_state_sizes(child_state_m, canvas)


@pytest.mark.parametrize("gui,state_path,recursive,rel_size", [
    (config_options, state_path_root, False, (40, 40)),
    (config_options, state_path_root, True, (40, 40)),
    (config_options, state_path_P, False, (20, 20)),
    (config_options, state_path_P, True, (20, 20)),
    (config_options, state_path_Hi, False, (20, 20)),
    (config_options, state_path_Hi, True, (20, 20)),
    (config_options, state_path_Ex, False, (20, 20)),
    (config_options, state_path_Ex, True, (20, 20)),
    (config_options, state_path_PC, False, (10, 10)),
    (config_options, state_path_PC, True, (10, 10))
], indirect=["gui"])
def test_simple_state_size_resize(gui, state_path, recursive, rel_size, monkeypatch):
    from rafcon.gui.helpers.meta_data import check_gaphas_state_meta_data_consistency
    sm_m, canvas, view = open_test_state_machine(gui)

    state_m = sm_m.get_state_model_by_path(state_path)
    state_v = canvas.get_view_for_model(state_m)

    orig_state_size = state_m.get_meta_data_editor()["size"]
    check_gaphas_state_meta_data_consistency(state_m, canvas, recursive=True)
    print("\ninitial:")
    print_state_sizes(state_m, canvas, ["C"])

    view_rel_size = transform_size_v2i(view, state_v, rel_size)
    resize_state(gui, view, state_v, rel_size, 3, recursive, monkeypatch)
    new_state_size = add_vectors(orig_state_size, view_rel_size)
    # sometimes (1 out of 10 cases), the initialization of gaphas elements is not correct
    # in these cases the vector entries are something around 4000 => the next loop catches these errors
    # TODO: probably fixed by setting GAPHAS_EDITOR_AUTO_FOCUS_OF_ROOT_STATE=False, remove if no longer occurring
    for elem in new_state_size:
        if elem > 1000:
            raise RuntimeError("graphical editor was probably not yet ready")

    print_state_sizes(state_m, canvas, ["C"])
    assert_state_size_and_meta_data_consistency(state_m, state_v, new_state_size, canvas)

    rel_size = (-rel_size[0], -rel_size[1])
    view_rel_size = transform_size_v2i(view, state_v, rel_size)
    resize_state(gui, view, state_v, rel_size, 3, recursive, monkeypatch)
    print("\nsecond resize:")
    print_state_sizes(state_m, canvas, ["C"])
    assert_state_size_and_meta_data_consistency(state_m, state_v, orig_state_size, canvas)

    gui(sm_m.history.undo)
    print("\nfirst undo:")
    print_state_sizes(state_m, canvas, ["C"])
    assert_state_size_and_meta_data_consistency(state_m, state_v, new_state_size, canvas)

    gui(sm_m.history.undo)
    print("\nsecond undo:")
    print_state_sizes(state_m, canvas, ["C"])
    assert_state_size_and_meta_data_consistency(state_m, state_v, orig_state_size, canvas)

    gui(sm_m.history.redo)
    assert_state_size_and_meta_data_consistency(state_m, state_v, new_state_size, canvas)

    gui(sm_m.history.redo)
    assert_state_size_and_meta_data_consistency(state_m, state_v, orig_state_size, canvas)


if __name__ == '__main__':
    import pytest
    pytest.main([__file__, '-xs'])
