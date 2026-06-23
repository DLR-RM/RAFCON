import os
import time
import pytest
from unittest.mock import MagicMock

import gi
gi.require_version('Gdk', '3.0')
gi.require_version('Gtk', '3.0')

from tests import utils as testing_utils


# --------------- unit tests --------------------    
def _make_view(width=800, height=600):
    view = MagicMock()
    view.get_allocated_width.return_value = width
    view.get_allocated_height.return_value = height
    return view

def _make_dummy_tool(view):
    from rafcon.gui.mygaphas.tools import AutoscrollMixin
    class _DummyTool(AutoscrollMixin):
        def __init__(self, v):
            self.view = v
            self.__init_mixin__()
    return _DummyTool(view)

# (left_border, top_border, right_border, bottom_border)
@pytest.mark.parametrize("x, y, expected", [
    (400, 300, (False, False, False, False)), # center, no scroll
    (5, 300, (True, False, False, False)),    # left margin
    (400, 5, (False, True, False, False)),  # top margin
    (795, 300, (False, False, True, False)),  # right margin
    (400, 595, (False, False, False, True)),  # bottom margn
])
def test_should_autoscroll(x, y, expected):
    tool = _make_dummy_tool(_make_view())
    assert tool._should_autoscroll(x, y) == expected

def test_is_dragging_false_when_idle():
    tool = _make_dummy_tool(_make_view())
    assert tool._is_dragging() is False

def test_is_dragging_true_with_movable_items():
    tool = _make_dummy_tool(_make_view())
    tool._movable_items = [MagicMock()]
    assert tool._is_dragging() is True

def test_is_dragging_true_with_motion_handle():
    tool = _make_dummy_tool(_make_view())
    tool.motion_handle = MagicMock()
    assert tool._is_dragging() is True

@pytest.mark.parametrize("drag_attr, drag_val", [
        ("_movable_items", [MagicMock()]),  # drag item (MoveItemTool)
        ("motion_handle", MagicMock()),     # drag handle (MoveHandleTool)
])
def test_handle_autoscroll_arms_timer_once_at_border(monkeypatch, drag_attr, drag_val):
    import rafcon.gui.mygaphas.tools as tools_module
    calls=[]
    monkeypatch.setattr(tools_module.GObject, "timeout_add",
                        lambda intervall, cb: calls.append((intervall, cb)) or 42)
    tool = _make_dummy_tool(_make_view())
    setattr(tool, drag_attr, drag_val)     # drag item and handle
    tool.handle_autoscroll(5, 300)         # left border hit
    assert tool._scroll_timeout_id == 42
    assert len(calls) == 1
    assert calls[0][0] == tool._SCROLL_INTERVALL
    assert calls[0][1] == tool._on_autoscroll

@pytest.mark.parametrize("drag_attr, drag_val", [
        ("_movable_items", [MagicMock()]),  # drag item (MoveItemTool)
        ("motion_handle", MagicMock()),     # drag handle (MoveHandleTool)
])
def test_handle_autoscroll_no_double_arm(monkeypatch, drag_attr, drag_val):
    import rafcon.gui.mygaphas.tools as tools_module
    calls = []
    monkeypatch.setattr(tools_module.GObject, "timeout_add",
                        lambda interval, cb: calls.append(cb) or 42)
    tool = _make_dummy_tool(_make_view())
    setattr(tool, drag_attr, drag_val)       # drag item and handle
    tool.handle_autoscroll(5, 300)
    tool.handle_autoscroll(5, 300)           # redundantly armed
    assert len(calls) == 1                   # not re-armed

@pytest.mark.parametrize("drag_attr, drag_val", [
        ("_movable_items", [MagicMock()]),  # drag item (MoveItemTool)
        ("motion_handle", MagicMock()),     # drag handle (MoveHandleTool)
])
def test_handle_autoscroll_stops_when_not_at_border(monkeypatch, drag_attr, drag_val):
    import rafcon.gui.mygaphas.tools as tools_module
    removed = []
    # create fake stop function for timer
    monkeypatch.setattr(tools_module.GObject, "source_remove", lambda tid: removed.append(tid))
    # start a fake timeout_add to not accidently start a real timer, prevent leakage
    monkeypatch.setattr(tools_module.GObject, "timeout_add", lambda i, cb: 42)
    tool = _make_dummy_tool(_make_view())
    setattr(tool, drag_attr, drag_val)       # drag item and handle
    tool._scroll_timeout_id = 99             # pretend timer is running
    tool.handle_autoscroll(400, 300)         # center -> no border
    assert removed == [99]
    assert tool._scroll_timeout_id == 0

@pytest.mark.parametrize("drag_attr, drag_val", [
        ("_movable_items", [MagicMock()]),  # drag item (MoveItemTool)
        ("motion_handle", MagicMock()),     # drag handle (MoveHandleTool)
])
def test_handle_autoscroll_not_dragging_does_not_arm(monkeypatch, drag_attr, drag_val):
    import rafcon.gui.mygaphas.tools as tools_module
    calls = []
    monkeypatch.setattr(tools_module.GObject, "timeout_add", lambda i, cb: calls.append(cb) or 42)
    tool = _make_dummy_tool(_make_view())
    tool.handle_autoscroll(5, 300)               # at border but NOT dragging
    assert calls == []
    assert tool._scroll_timeout_id == 0

def _dragging_tool_with_stubbed_scroll(monkeypatch, x, y, parent=None):
    tool = _make_dummy_tool(_make_view())        
    item = MagicMock()
    if not parent:
        item.item.parent = None                  # skip parent-border logic
    else:
        item.item.parent = MagicMock
    tool._movable_items = [item]
    tool._last_event_pos = (x, y)
    scrolls = []
    monkeypatch.setattr(tool, "_scroll_view", lambda dx, dy: scrolls.append((dx, dy)))
    return tool, item, scrolls

def test_on_autoscroll_right_border_positive_dx(monkeypatch):
    tool, item, scrolls = _dragging_tool_with_stubbed_scroll(monkeypatch, 795, 300)
    assert tool._on_autoscroll() is True
    assert scrolls == [(tool._speed, 0)]            # right -> +x, no y
    item.move.assert_called_once()

def test_on_autoscroll_top_border_negative_dy(monkeypatch):
    tool, item, scrolls = _dragging_tool_with_stubbed_scroll(monkeypatch, 400, 5)
    tool._on_autoscroll()
    assert scrolls == [(0, -tool._speed)]           # top -> no x, -y

def test_on_autoscroll_corner_diagonal(monkeypatch):
    tool, item, scrolls = _dragging_tool_with_stubbed_scroll(monkeypatch, 795, 595)
    tool._on_autoscroll()
    assert scrolls == [(tool._speed, tool._speed)]  # bottom-right -> +x,+y (diagonal)

def test_on_autoscroll_stops_when_not_dragging(monkeypatch):
    import rafcon.gui.mygaphas.tools as tools_module
    monkeypatch.setattr(tools_module.GObject, "source_remove", lambda tid: None)
    tool = _make_dummy_tool(_make_view())
    tool._last_event_pos = (795, 300)
    assert tool._on_autoscroll() is False           # not dragging -> returns False and exit


# ----------- intergration tests ----------------
