import os
os.environ["GDK_SCALE"] = "1"
os.environ["GDK_DPI_SCALE"] = "1"
os.environ.setdefault("GDK_BACKEND", "x11")   # avoid Wayland fractional scaling

import time
import pytest

import gi
gi.require_version('Gdk', '3.0')
gi.require_version('Gtk', '3.0')

from tests import utils as testing_utils
from rafcon.utils import log

logger = log.get_logger(__name__)

'''
NOTICE 1:
THE GLOBAL `auto-maximize` SETTING OF THE WM MUST BE SET TO `FALSE`, otherwise the runtime_config of rafcon is overwritten
by the WM global settings. The upcoming gui is the forced to full-window size by the WM, which leads the GUI widget into a
undefined state, depending on the screen resolution settings.
For GNOME WM the auto-maximize setting can be checked with: `gsettings get org.gnome.mutter auto-maximize`
It can be set to false with: `gsettings set org.gnome.mutter auto-maximize false`
---------------------------------------------------------------------------------------------------------------------------
NOTICE 2:
WRITE UNIT-TESTS AND INTEGRATION TESTS IN SEPARATE FILES.
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

_MAIN_WINDOW_SIZE = (1500, 800)

config_options = {
"gui_config":  {
    'HISTORY_ENABLED': True,
    'GAPHAS_EDITOR_AUTO_FOCUS_OF_ROOT_STATE': False,
    'GRAPHAS_EDITOR_AUTOSCROLL_SPEED': 100
},
# the GUI widget needs to stay in a defined, reproducable size for the autoscroll tests.
# the editor view size here is (400, 600) 
"runtime_config": {
    'MAIN_WINDOW_MAXIMIZED': False,
    'MAIN_WINDOW_SIZE': _MAIN_WINDOW_SIZE,
    'MAIN_WINDOW_POS': (0, 0),
    'LEFT_BAR_DOCKED_POS': 400,
    'RIGHT_BAR_DOCKED_POS': _MAIN_WINDOW_SIZE[0]-200,
    'CONSOLE_DOCKED_POS': _MAIN_WINDOW_SIZE[1]-100,
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
    from gi.repository import Gdk
    from rafcon.gui.mygaphas.tools import MoveItemTool

    view_center_x = view.get_allocated_width()/2
    view_center_y = view.get_allocated_height()/2
    # pos state in view coordinates
    i_x, i_y = view.get_matrix_i2v(state_v).transform_point(state_v.width/2, state_v.height/2)
    d_x = view_center_x -i_x
    d_y = view_center_y - i_y
    #move view so state is centered:
    view._matrix.translate(d_x / view._matrix[0], d_y / view._matrix[3])
    view.request_update((), view.canvas.get_all_items())
    cx, cy = view.get_matrix_i2v(state_v).transform_point(state_v.width/2, state_v.height/2)
    return cx, cy

def _assert_centered(view, state_v, tol=2.0):
    cx, cy = view.get_matrix_i2v(state_v).transform_point(
        state_v.width / 2.0, state_v.height / 2.0)
    center_x = view.get_allocated_width() / 2.0
    center_y = view.get_allocated_height() / 2.0
    assert abs(cx - center_x) <= tol, f"x off by {cx - center_x:.1f}px"
    assert abs(cy - center_y) <= tol, f"y off by {cy - center_y:.1f}px"

def _get_handle_pos(view, state_v, handle):
    i2v = view.get_matrix_i2v(state_v)
    item_pos_handle = (handle.pos.x.value, handle.pos.y.value)
    view_pos_handle = i2v.transform_point(*item_pos_handle)
    return view_pos_handle

def _make_event(event_type, x, y, button=1, state=None):
    from gi.repository import Gdk
    event = Gdk.Event.new(event_type)
    event.x = float(x)
    event.y = float(y)
    if event_type in (Gdk.EventType.BUTTON_PRESS, Gdk.EventType.BUTTON_RELEASE):
        event.button = button
    if state is not None:
        event.state = state
    return event

def _zoom_into_view(gui, view, x, y):
    from gi.repository import Gdk
    from rafcon.gui.mygaphas.tools import ZoomTool
    tool = gui(ZoomTool, view)
    scroll_event = _make_event(Gdk.EventType.SCROLL, x, y)
    while view._matrix[0] <= 15.0:
        gui(tool.on_scroll, scroll_event)
    _assert_overflow(view)


def _assert_overflow(view, axis="h"):
    '''Self-checking precondition: there must be off-screen canvas to scroll into.
    Returns the relevant adjustment so the test can read/set scroll position.'''
    adj = view.get_hadjustment() if axis == "h" else view.get_vadjustment()
    assert adj.get_upper() > adj.get_page_size(), (
        "root state is not larger than the viewport; nothing for autoscroll to "
        "scroll into (the _enlarge_root_state setup failed)"
    )
    return adj

def _get_handle_pos(view, state_v, handle):
        i2v = view.get_matrix_i2v(state_v)
        item_pos_handle = (handle.pos.x.value, handle.pos.y.value)
        view_pos_handle = i2v.transform_point(*item_pos_handle)
        return view_pos_handle

def _item_drag_into_autoscroll_and_stop(gui, view, state_v, monkeypatch, stop_condition = "BUTTON_RELEASE"):
    from gi.repository import Gdk
    from rafcon.gui.mygaphas.tools import MoveItemTool
    
    tool = gui(MoveItemTool, view)
    
    monkeypatch.setattr(tool, "get_item", lambda: state_v)    # deterministic item grab
    monkeypatch.setattr("rafcon.gui.mygaphas.guide.GuidedStateMixin.MARGIN", 0)   # disable snapping
    
    gui(view.unselect_all); gui(view.select_item, state_v)
    testing_utils.wait_for_gui()
    
    cx, cy = view.get_matrix_i2v(state_v).transform_point(state_v.width/2, state_v.height/2)
    select_event = _make_event(Gdk.EventType.BUTTON_PRESS, cx, cy)
    gui(tool.on_button_press, select_event)
    testing_utils.wait_for_gui()
    
    assert tool._item is not None, "tool did not select the state item"
    
    move_event = _make_event(Gdk.EventType.MOTION_NOTIFY, cx, cy)
    move_event.state = move_event.get_state()[1] | Gdk.EventMask.BUTTON_PRESS_MASK
    ticks = view.get_allocated_width()/2 - 5   # 608 - 5 = 603 ticks -> should trigger autoscroll at x = 1019
    
    for i in range(0, int(ticks), 5):
        move_event.x = cx + i
        move_event.y = cy
        gui(tool.on_motion_notify, move_event)
        testing_utils.wait_for_gui()
        assert tool._is_dragging(), "item drag did not start"
        if tool._scroll_tick_id > 0:
            break
    if stop_condition == "BUTTON_RELEASE":
        stop_event = _make_event(Gdk.EventType.BUTTON_RELEASE, move_event.x, move_event.y)
        gui(tool.on_button_release, stop_event)
        testing_utils.wait_for_gui()
    elif stop_condition == "DRAG_BACK":
        move_event.x = cx 
        move_event.y = cy
        gui(tool.on_motion_notify, move_event)
        testing_utils.wait_for_gui()


def _handle_drag_into_autoscroll_and_stop(gui, view, state_v, monkeypatch, stop_condition="BUTTON_RELEASE"):
    from gi.repository import Gdk
    from rafcon.gui.mygaphas.tools import MoveHandleTool
    from gaphas.item import SE
    
    tool = gui(MoveHandleTool)
    tool.view = view
    
    monkeypatch.setattr(tool, "grabbed_item", lambda: state_v)    # deterministic item grab
    monkeypatch.setattr(tool, "grabbed_handle", lambda: state_v.handles()[SE])
    # Deactivate guides (snapping)
    monkeypatch.setattr("rafcon.gui.mygaphas.guide.GuidedStateMixin.MARGIN", 0)
    
    gui(view.unselect_all); gui(view.select_item, state_v)
    testing_utils.wait_for_gui()

    assert tool.grabbed_handle is not None, "tool did not select the handle"
    # get curent position of handle
    cx, cy = _get_handle_pos(view, state_v, tool.grabbed_handle())
    select_event = _make_event(Gdk.EventType.BUTTON_PRESS, cx, cy)
    gui(tool.on_button_press, select_event)
    testing_utils.wait_for_gui()

    move_event = _make_event(Gdk.EventType.MOTION_NOTIFY, cx, cy)
    move_event.state = move_event.get_state()[1] | Gdk.EventMask.BUTTON_PRESS_MASK

    ticks = view.get_allocated_width() - (cx + 5)   # 608 - 5 = 603 ticks -> should trigger autoscroll at x = 1019
    
    for i in range(0, int(ticks), 5):
        move_event.x = cx + i
        move_event.y = cy
        gui(tool.on_motion_notify, move_event)
        testing_utils.wait_for_gui()
        assert tool._is_dragging(), "item drag did not start"
        if tool._scroll_tick_id > 0:
            break
    if stop_condition == "BUTTON_RELEASE":
        stop_event = _make_event(Gdk.EventType.BUTTON_RELEASE, move_event.x, move_event.y)
        gui(tool.on_button_release, stop_event)
        testing_utils.wait_for_gui()
    elif stop_condition == "DRAG_BACK":
        move_event.x = cx 
        move_event.y = cy
        gui(tool.on_motion_notify, move_event)
        testing_utils.wait_for_gui()

def _connection_drag_into_autoscroll_and_stop(gui, view, state_v, monkeypatch, stop_condition="BUTTON_RELEASE"):
    from gi.repository import Gdk
    from gaphas.aspect import HandleFinder
    from rafcon.gui.mygaphas.tools import ConnectionCreationTool

    def _get_output_port_handle(state_v, port_name=None):
        from rafcon.gui.mygaphas.items.ports import OutputPortView
        # state_v.output_ports (or .outputs) holds the OutputPortView objects
        for port_v in state_v.outputs:
            if port_name is None or port_v.model.data_port.name == port_name:
                return port_v, port_v.handle   # the port view and its handle
        raise AssertionError(f"output port {port_name!r} not found on state")

    tool = gui(ConnectionCreationTool)
    tool.view = view

    port_v, handle = _get_output_port_handle(state_v, "output_1")
    # get curent position of handles
    cx, cy = _get_handle_pos(view, port_v, handle)
    select_event = _make_event(Gdk.EventType.BUTTON_PRESS, cx, cy)
    gui(tool.on_button_press, select_event)
    testing_utils.wait_for_gui()

    move_event = _make_event(Gdk.EventType.MOTION_NOTIFY, cx, cy)
    move_event.state = move_event.get_state()[1] | Gdk.EventMask.BUTTON_PRESS_MASK

    ticks = view.get_allocated_width() - (cx + 5)   # 608 - 5 = 603 ticks -> should trigger autoscroll at x = 1019
    for i in range(0, int(ticks), 5): 
        move_event.x = cx + i
        move_event.y = cy
        gui(tool.on_motion_notify, move_event)
        testing_utils.wait_for_gui()
        assert tool._is_dragging(), "item drag did not start"
        if tool._scroll_tick_id > 0:
            break
    if stop_condition == "BUTTON_RELEASE":
        stop_event = _make_event(Gdk.EventType.BUTTON_RELEASE, move_event.x, move_event.y)
        gui(tool.on_button_release, stop_event)
        testing_utils.wait_for_gui()
    elif stop_condition == "DRAG_BACK":
        move_event.x = cx 
        move_event.y = cy
        gui(tool.on_motion_notify, move_event)
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