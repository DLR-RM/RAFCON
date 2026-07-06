import os
import time
import pytest

import gi
gi.require_version('Gdk', '4.0')
gi.require_version('Gtk', '4.0')

from tests import utils as testing_utils

'''
NOTICE:
WRITE UNI-TESTS AND INTEGRATION TESTS IN SEPARATE FILES. 
GUI singleton models are imported in both tests (e.g. in unit tests by rafcon.mygaphas.tools -> 
rafcon.gui.controllers.right_click_menu.state -> rafcon.gui.singleton and integration tests directly rafcon.gui.singleton),
leading to THREADING ISSUES when running both tests sequentially in the same file.
Reason:
The singletons get created at frst import. 
With unit tests importing it first, singletons gets bound to the MainThread, resulting the integration tests 
crashing, because the singleton objects are now not on the GtkThread (GUI) available, on which the integration 
tests are running.
Solution:
To prevent this confusing multi-threading constellation, just run the tests in seperate scripts.
'''

# ----------- intergration tests ----------------
sm_path_autoscroll = os.path.join(testing_utils.TEST_ASSETS_PATH, "unit_test_state_machines", "autoscroll_test")

root_state = "GLNOWX"
hierarchy_state_1 = "GLNOWX/FQNEQG"
execution_state_1 = "GLNOWX/FQNEQG/MNNCKT"
execution_state_2 = "GLNOWX/PHVUSJ"

config_options = {
"gui_config":  {
    'HISTORY_ENABLED': True,
    'GAPHAS_EDITOR_AUTO_FOCUS_OF_ROOT_STATE': False
},
# the GUI widget needs to stay in a defined, reproducable size for the autoscroll tests.
# the editor view size here is (400, 600) 
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

    gui(menubar_ctrl.on_open_activate, None, None, sm_path_autoscroll)
    time.sleep(0.5)
    testing_utils.wait_for_gui()

    sm_m = smm_m.state_machines[smm_m.selected_state_machine_id]
    sm_id = sm_m.state_machine.state_machine_id
    sm_gaphas_ctrl = state_machines_ctrl.get_controller(sm_id)
    canvas = sm_gaphas_ctrl.canvas
    gaphas_view = sm_gaphas_ctrl.view.editor

    return sm_m, canvas, gaphas_view

def close_test_state_machine(gui):
    import rafcon.gui.singleton
    menubar_ctrl = rafcon.gui.singleton.main_window_controller.menu_bar_controller
    gui(menubar_ctrl.on_stop_activate, None)
    gui(menubar_ctrl.on_close_all_activate, None, None)
    testing_utils.wait_for_gui()

def _center_state_in_view(view, state_v):
    view_center_x = view.get_width()/2
    view_center_y = view.get_height()/2
    # pos state in view coordinates
    i_x, i_y = view.get_matrix_i2v(state_v).transform_point(state_v.width/2, state_v.height/2)
    d_x = view_center_x - i_x
    d_y = view_center_y - i_y
    #move view so state is centered:
    view.matrix.translate(d_x / view.matrix[0], d_y / view.matrix[3])
    for item in view.canvas.get_all_items():
        view.canvas.request_update(item)
    view.update()
    cx, cy = view.get_matrix_i2v(state_v).transform_point(state_v.width/2, state_v.height/2)
    return cx, cy

def _assert_centered(view, state_v, tol=2.0):
    cx, cy = view.get_matrix_i2v(state_v).transform_point(
        state_v.width / 2.0, state_v.height / 2.0)
    center_x = view.get_width() / 2.0
    center_y = view.get_height() / 2.0
    assert abs(cx - center_x) <= tol, f"x off by {cx - center_x:.1f}px"
    assert abs(cy - center_y) <= tol, f"y off by {cy - center_y:.1f}px"

def _get_handle_pos(view, state_v, handle):
    i2v = view.get_matrix_i2v(state_v)
    item_pos_handle = (handle.pos.x.value, handle.pos.y.value)
    view_pos_handle = i2v.transform_point(*item_pos_handle)
    return view_pos_handle

def _zoom_into_view(gui, view, x, y):
    """Zoom into the view at position (x, y) until the canvas is larger than the viewport

    GTK4/gaphas 5 has no ZoomTool anymore (zooming happens in a scroll event controller),
    so the view matrix is scaled directly, like the zoom handler does.
    """
    def zoom():
        while view.matrix[0] <= 15.0:
            zx, zy = view.matrix.inverse().transform_point(x, y)
            view.matrix.translate(zx, zy)
            view.matrix.scale(1.25, 1.25)
            view.matrix.translate(-zx, -zy)
        for item in view.canvas.get_all_items():
            view.canvas.request_update(item)
        view.update()
    gui(zoom)
    testing_utils.wait_for_gui()
    _assert_overflow(view)


def _assert_overflow(view, axis="h"):
    '''Self-checking precondition: there must be off-screen canvas to scroll into.
    Returns the relevant adjustment so the test can read/set scroll position.'''
    adj = view.hadjustment if axis == "h" else view.vadjustment
    assert adj.get_upper() > adj.get_page_size(), (
        "root state is not larger than the viewport; nothing for autoscroll to "
        "scroll into (the _enlarge_root_state setup failed)"
    )
    return adj

def _item_drag_into_autoscroll_and_stop(gui, view, state_v, monkeypatch, stop_condition = "BUTTON_RELEASE"):
    from rafcon.gui.mygaphas.tools import MoveItemMode

    mode = gui(MoveItemMode, view)

    monkeypatch.setattr("rafcon.gui.mygaphas.guide.GuidedStateMixin.MARGIN", 0)   # disable snapping

    gui(view.unselect_all); gui(view.select_item, state_v)
    gui(setattr, view, "hovered_item", state_v)    # deterministic item grab
    testing_utils.wait_for_gui()

    cx, cy = view.get_matrix_i2v(state_v).transform_point(state_v.width/2, state_v.height/2)
    assert gui(mode.begin, cx, cy, 0), "mode did not accept the drag"
    testing_utils.wait_for_gui()

    assert mode._item is not None, "mode did not select the state item"

    ticks = view.get_width()/2 - 5   # 608 - 5 = 603 ticks -> should trigger autoscroll at x = 1019
    x, y = cx, cy
    for i in range(0, int(ticks), 5):
        x, y = cx + i, cy
        gui(mode.update, x, y, 0)
        testing_utils.wait_for_gui()
        assert mode._is_dragging(), "item drag did not start"
        if mode._scroll_tick_id > 0:
            break
    if stop_condition == "BUTTON_RELEASE":
        gui(mode.end, x, y, 0)
        testing_utils.wait_for_gui()
    elif stop_condition == "DRAG_BACK":
        gui(mode.update, cx, cy, 0)
        testing_utils.wait_for_gui()


def _handle_drag_into_autoscroll_and_stop(gui, view, state_v, monkeypatch, stop_condition="BUTTON_RELEASE"):
    from rafcon.gui.mygaphas.tools import HandleMoveMode
    from gaphas.item import SE

    mode = gui(HandleMoveMode, view)

    # Deactivate guides (snapping)
    monkeypatch.setattr("rafcon.gui.mygaphas.guide.GuidedStateMixin.MARGIN", 0)

    gui(view.unselect_all); gui(view.select_item, state_v)
    testing_utils.wait_for_gui()

    # deterministic handle grab (bypasses the HandleFinder in begin())
    mode.grabbed_item = state_v
    mode.grabbed_handle = state_v.handles()[SE]

    # get curent position of handle
    cx, cy = _get_handle_pos(view, state_v, mode.grabbed_handle)

    ticks = view.get_width() - (cx + 5)   # 608 - 5 = 603 ticks -> should trigger autoscroll at x = 1019
    x, y = cx, cy
    for i in range(0, int(ticks), 5):
        x, y = cx + i, cy
        gui(mode.update, x, y, 0)
        testing_utils.wait_for_gui()
        assert mode._is_dragging(), "handle drag did not start"
        if mode._scroll_tick_id > 0:
            break
    if stop_condition == "BUTTON_RELEASE":
        gui(mode.end, x, y, 0)
        testing_utils.wait_for_gui()
    elif stop_condition == "DRAG_BACK":
        gui(mode.update, cx, cy, 0)
        testing_utils.wait_for_gui()

def _connection_drag_into_autoscroll_and_stop(gui, view, state_v, monkeypatch, stop_condition="BUTTON_RELEASE"):
    from rafcon.gui.mygaphas.tools import ConnectionCreationMode

    def _get_output_port_handle(state_v, port_name=None):
        # state_v.outputs holds the OutputPortView objects
        for port_v in state_v.outputs:
            if port_name is None or port_v.model.data_port.name == port_name:
                return port_v, port_v.handle   # the port view and its handle
        raise AssertionError(f"output port {port_name!r} not found on state")

    mode = gui(ConnectionCreationMode, view)

    port_v, handle = _get_output_port_handle(state_v, "output_1")
    # deterministic port grab (bypasses the HandleFinder in begin())
    mode._start_port_v = port_v
    mode._is_transition = False
    mode._parent_state_v = port_v.parent.parent

    # get curent position of the port handle
    cx, cy = _get_handle_pos(view, port_v, handle)

    ticks = view.get_width() - (cx + 5)   # 608 - 5 = 603 ticks -> should trigger autoscroll at x = 1019
    x, y = cx, cy
    for i in range(0, int(ticks), 5):
        x, y = cx + i, cy
        gui(mode.update, x, y, 0)
        testing_utils.wait_for_gui()
        assert mode._is_dragging(), "connection drag did not start"
        if mode._scroll_tick_id > 0:
            break
    if stop_condition == "BUTTON_RELEASE":
        # The release position may lie on a port (e.g. the start port after the view was
        # scrolled), which would attempt a connection creation - that is not under test here
        mode._current_sink = None
        gui(mode.end, x, y, 0)
        testing_utils.wait_for_gui()
    elif stop_condition == "DRAG_BACK":
        gui(mode.update, cx, cy, 0)
        testing_utils.wait_for_gui()
    
@pytest.mark.parametrize("gui", [config_options], indirect=True)
def test_autoscroll_item_release_button(gui, monkeypatch):
    '''drag item into the autoscroll zone, perform one cycle of autoscroll and then release the mouse button
    to stop the autoscrolling'''
    from gi.repository import Gdk

    sm_m, canvas, view = open_test_state_machine(gui)
    state_m = sm_m.get_state_model_by_path(execution_state_2)
    state_v = canvas.get_view_for_model(state_m)

    x, y = _center_state_in_view(view=view, state_v=state_v); _assert_centered(view, state_v)
    _zoom_into_view(gui, view, x, y)
    # release button in autoscroll zone to stop autoscroll
    _item_drag_into_autoscroll_and_stop(gui, view, state_v, monkeypatch, "BUTTON_RELEASE")

@pytest.mark.parametrize("gui", [config_options], indirect=True)
def test_autoscroll_item_drag_back(gui, monkeypatch):
    '''drag item into the autoscroll zone, perform one cycle of autoscroll and then drag the item back
    to stop the autoscrolling'''
    from gi.repository import Gdk

    sm_m, canvas, view = open_test_state_machine(gui)
    state_m = sm_m.get_state_model_by_path(execution_state_2)
    state_v = canvas.get_view_for_model(state_m)

    x, y = _center_state_in_view(view=view, state_v=state_v); _assert_centered(view, state_v)
    _zoom_into_view(gui, view, x, y)
    # drag item back out of autoscroll zone to stop autoscrolling
    _item_drag_into_autoscroll_and_stop(gui, view, state_v, monkeypatch, "DRAG_BACK")

@pytest.mark.parametrize("gui", [config_options], indirect=True)
def test_autoscroll_handle_release_button(gui, monkeypatch):
    '''drag state handle into the autoscroll zone, perform one cycle of autoscroll and then elease the mouse button
    to stop the autoscrolling'''
    from gi.repository import Gdk
    from gaphas.item import SE, NW

    sm_m, canvas, view = open_test_state_machine(gui)
    state_m = sm_m.get_state_model_by_path(execution_state_2)
    state_v = canvas.get_view_for_model(state_m)

    x, y = _center_state_in_view(view=view, state_v=state_v); _assert_centered(view, state_v)
    _zoom_into_view(gui, view, x, y)
    _handle_drag_into_autoscroll_and_stop(gui, view, state_v, monkeypatch, "BUTTON_RELEASE")

@pytest.mark.parametrize("gui", [config_options], indirect=True)
def test_autoscroll_handle_drag_back(gui, monkeypatch):
    '''drag state handle into the autoscroll zone, perform one cycle of autoscroll and then drag the handle back
    to stop the autoscrolling'''
    from gi.repository import Gdk
    from gaphas.item import SE, NW

    sm_m, canvas, view = open_test_state_machine(gui)
    state_m = sm_m.get_state_model_by_path(execution_state_2)
    state_v = canvas.get_view_for_model(state_m)

    x, y = _center_state_in_view(view=view, state_v=state_v); _assert_centered(view, state_v)
    _zoom_into_view(gui, view, x, y)
    _handle_drag_into_autoscroll_and_stop(gui, view, state_v, monkeypatch, "DRAG_BACK")

@pytest.mark.parametrize("gui", [config_options], indirect=True)
def test_autoscroll_connection_release_button(gui, monkeypatch):
    '''drag connection handle into the autoscroll zone, perform one cycle of autoscroll and then elease the mouse button
    to stop the autoscrolling'''
    from gi.repository import Gdk
    from gaphas.item import SE, NW

    sm_m, canvas, view = open_test_state_machine(gui)
    state_m = sm_m.get_state_model_by_path(execution_state_2)
    state_v = canvas.get_view_for_model(state_m)

    x, y = _center_state_in_view(view=view, state_v=state_v); _assert_centered(view, state_v)
    _zoom_into_view(gui, view, x, y)
    _connection_drag_into_autoscroll_and_stop(gui, view, state_v, monkeypatch, "BUTTON_RELEASE")

@pytest.mark.parametrize("gui", [config_options], indirect=True)
def test_autoscroll_connection_drag_back(gui, monkeypatch):
    '''drag connection handle into the autoscroll zone, perform one cycle of autoscroll and then drag the handle back
    to stop the autoscrolling'''
    from gi.repository import Gdk
    from gaphas.item import SE, NW

    sm_m, canvas, view = open_test_state_machine(gui)
    state_m = sm_m.get_state_model_by_path(execution_state_2)
    state_v = canvas.get_view_for_model(state_m)

    x, y = _center_state_in_view(view=view, state_v=state_v); _assert_centered(view, state_v)
    _zoom_into_view(gui, view, x, y)
    _connection_drag_into_autoscroll_and_stop(gui, view, state_v, monkeypatch, "DRAG_BACK")

if __name__ == '__main__':
    pytest.main([__file__, '-xs'])