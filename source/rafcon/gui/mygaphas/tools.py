# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""RAFCON graphical editor tools as GTK4 event controllers (gaphas 5)

The gaphas 2.x ToolChain with its grab/ungrab semantics is gone. All button-1
interactions (moving handles, creating/modifying connections, moving items,
rubberband selection) are dispatched by a single Gtk.GestureDrag, mirroring the
old tool chain priorities. Hovering, scrolling/zooming, panning and the right
click menu are separate event controllers.
"""

from gi.repository import Gdk, Gtk
from gaphas.item import NW
from gaphas.cursor import ELEMENT_CURSORS
from gaphas.tool.rubberband import RubberbandState

from rafcon.gui.controllers.right_click_menu.state import StateRightClickMenuGaphas
import rafcon.gui.helpers.state_machine as gui_helper_state_machine
from rafcon.gui.mygaphas.aspect import HandleFinder, HandleInMotion, merge_connection_segments
from rafcon.gui.mygaphas.guide import InMotion
from rafcon.gui.mygaphas.items.connection import ConnectionView, TransitionPlaceholderView, DataFlowPlaceholderView, \
    TransitionView, DataFlowView
from rafcon.gui.mygaphas.items.ports import InputPortView, PortView
from rafcon.gui.mygaphas.items.state import StateView, NameView
from rafcon.gui.mygaphas.utils import gap_helper
from rafcon.gui.utils import constants
from rafcon.utils import log
from rafcon.utils.decorators import avoid_parallel_execution
from rafcon.gui.config import global_gui_config

logger = log.get_logger(__name__)


class ButtonEventShim(object):
    """Minimal GTK3-style event object for the right click menu controllers"""

    def __init__(self, x, y, button=3):
        self.type = Gdk.EventType.BUTTON_PRESS
        self.x = x
        self.y = y
        self._button = button

    def get_button(self):
        return True, self._button


def add_tools_to_view(view):
    """Create and attach all graphical editor tools to the given view"""
    drag_dispatcher = PrimaryDragDispatcher(view)
    hover_tool = HoverItemTool(view, drag_dispatcher)
    view.add_controller(hover_tool.controller)
    view.add_controller(drag_dispatcher.gesture)
    view.add_controller(create_pan_gesture(view))
    view.add_controller(create_scroll_controller(view))
    view.add_controller(create_right_click_gesture(view))
    # keep tool objects alive
    view._tools = (drag_dispatcher, hover_tool)
    return drag_dispatcher


class AutoscrollMixin:
    """ mixin class to add autoscroll to a graphical editor tool.
    When an item or handle is dragged against the border of the graphical editor,
    the view is scrolled in that direction. The dragged item follows the curser.

    This mixin class relies on the following attributes provided by the tool class:
        * ``self.view``             - the GtkView object on which the tool operates on
        * ``self._movable_items``   - InMotion objects, which are set by the item move tool
        * ``self.motion_handle``    - HandleInMotion object, which is set by the handle tools
    Call ``self.__init_mixin__()`` in the tool`s ``__init__`` and
    ``self.handle_autoscroll(x, y)`` in the drag-update handler of the tool
    """
    _NOMINAL_FRAME_DT = 1.0 / 60.0   # seed dt for the first frame of a autoscroll cycle (assume 60fps)

    def __init_mixin__(self) -> None:
        self._scroll_tick_id = 0
        self._last_frame_time = 0
        self._last_event_pos = (0, 0)
        self._margin = 30  # px distance to border to trigger autoscroll
        self._speed = global_gui_config.get_config_value("GRAPHAS_EDITOR_AUTOSCROLL_SPEED", 200)  # px scrolled per second

    def handle_autoscroll(self, x: float, y: float) -> None:
        '''Start or stop autoscrolling based on the current curser position'''
        self._last_event_pos = (x, y)
        if self._is_dragging() and any(self._should_autoscroll(x, y)):
            if not self._scroll_tick_id:
                self._last_frame_time = 0
                self._scroll_tick_id = self.view.add_tick_callback(self._on_autoscroll)
        elif self._scroll_tick_id:
            self._stop_autoscroll()

    def _is_dragging(self) -> bool:
        '''True while this tool is dragging an item or handle'''
        return bool(getattr(self, '_movable_items', None)) or \
               bool(getattr(self, 'motion_handle', None))

    def _should_autoscroll(self, x: float, y: float):
        '''checks if border thresholds get hit for autoscrolling.
        -> returns (left, top, right, bottom) boolean for hit borders.
        '''
        width = self.view.get_width()
        height = self.view.get_height()

        left_border_hit = x < self._margin
        top_border_hit = y < self._margin
        right_border_hit = x > width - self._margin
        bottom_border_hit = y > height - self._margin

        return left_border_hit, top_border_hit, right_border_hit, bottom_border_hit

    def _scroll_view(self, dx: float, dy: float) -> None:
        '''Scroll the view by (dx, dy) pixels by adjusting the view object'''
        h_adj = self.view.hadjustment
        v_adj = self.view.vadjustment
        h_adj.set_value(h_adj.get_value() + dx)
        v_adj.set_value(v_adj.get_value() + dy)

    def _on_autoscroll(self, widget, frame_clock) -> bool:
        '''Frame clock callback: scroll the view and drags item(s) along - fires one per rendered frame'''
        if not self._is_dragging():
            self._stop_autoscroll()
            return False

        '''GdkFrameClock returns the timestamp of current frame in microseconds. We use it to calculate
        the time difference between the previous frame and the current frame to encounter different rendering times
        and ensure constant scrolling behavior with different resolutions of the screens.'''
        now = frame_clock.get_frame_time()
        if self._last_frame_time:
            dt = (now - self._last_frame_time) / 1_000_000.0
        else:
            dt = self._NOMINAL_FRAME_DT
        self._last_frame_time = now
        step = self._speed * min(dt, 1.0 / 30.0)  # cap to encounter unforseen frame-clock behavior leading to big jumps

        x, y = self._last_event_pos
        zoom = self.view.matrix[0]
        scroll_directions = self._should_autoscroll(self._last_event_pos[0],
                                                    self._last_event_pos[1])
        dx = (scroll_directions[2]-scroll_directions[0]) * step*zoom
        dy = (scroll_directions[3]-scroll_directions[1]) * step*zoom

        offset_x = x + dx
        offset_y = y + dy

        if dx or dy:
            self._scroll_view(dx, dy)
            if getattr(self, '_movable_items', None):
                for inmotion in self._movable_items:
                    if inmotion.item.parent:
                        parent_border_left = parent_border_top = inmotion.item.parent.border_width
                        parent_border_right = inmotion.item.parent.width - \
                            (inmotion.item.width + inmotion.item.parent.border_width)
                        parent_border_bottom = inmotion.item.parent.height - \
                            (inmotion.item.height + inmotion.item.parent.border_width)

                        rel_x, rel_y = gap_helper.calc_rel_pos_to_parent(self.view.canvas, inmotion.item,
                                                                         inmotion.item.handles()[NW])
                        # check to stop if parent borders are hit
                        if rel_x in (parent_border_left, parent_border_right) or \
                           rel_y in (parent_border_top, parent_border_bottom):
                            self._stop_autoscroll()
                            return False

                    '''*2 factor multiplication to compensate for the shifted view coordinates.
                    The view already shifted here, to keep the item also aligned with the cursor
                    while shifting, we need to add the delta a 2nd time'''
                    inmotion.move((x+2*dx, y+2*dy))
                    inmotion.last_x = offset_x
                    inmotion.last_y = offset_y

            elif getattr(self, 'motion_handle', None):
                self.motion_handle.move((offset_x, offset_y))
            return True
        else:
            self._stop_autoscroll()
            return False

    def _stop_autoscroll(self) -> None:
        if self._scroll_tick_id:
            self.view.remove_tick_callback(self._scroll_tick_id)
            self._scroll_tick_id = 0


class DragMode:
    """Base class for the modes dispatched by the PrimaryDragDispatcher"""

    def __init__(self, view):
        self.view = view

    def begin(self, x, y, modifiers):
        """Return True if this mode handles the drag starting at (x, y)"""
        raise NotImplementedError

    def update(self, x, y, modifiers):
        pass

    def end(self, x, y, modifiers):
        pass


class HandleMoveMode(AutoscrollMixin, DragMode):
    """Move handles around (resize states/names, move ports, move connection waypoints)

    Handles can be moved using click'n'drag. Moving ports requires a modifier key to
    be pressed (defined in ``rafcon.gui.utils.constants``).
    """

    def __init__(self, view):
        super(HandleMoveMode, self).__init__(view)
        self.__init_mixin__()
        self.grabbed_item = None
        self.grabbed_handle = None
        self.motion_handle = None

    def begin(self, x, y, modifiers):
        view = self.view

        if isinstance(view.hovered_item, StateView):
            distance = view.hovered_item.border_width / 2.
            item, handle = HandleFinder(view.hovered_item, view).get_handle_at_point((x, y), distance)
        else:
            item, handle = HandleFinder(view.hovered_item, view).get_handle_at_point((x, y))

        if not handle:
            return False

        # Only move ports when the MOVE_PORT_MODIFIER key is pressed
        if isinstance(item, (StateView, PortView)) and \
                handle in [port.handle for port in item.get_all_ports()] and \
                not (modifiers & constants.MOVE_PORT_MODIFIER):
            return False

        # Do not move from/to handles of connections (only their waypoints)
        if isinstance(item, ConnectionView) and handle in item.end_handles(include_waypoints=True):
            return False

        view.hovered_item = item
        self.grabbed_item = item
        self.grabbed_handle = handle
        self.motion_handle = None
        return True

    def update(self, x, y, modifiers):
        if not self.grabbed_handle:
            return
        item = self.grabbed_item
        resize_recursive = isinstance(item, StateView) and self.grabbed_handle in item.corner_handles and \
            modifiers & constants.RECURSIVE_RESIZE_MODIFIER

        if resize_recursive:
            old_size = (item.width, item.height)

        self.handle_autoscroll(x, y)

        if not self.motion_handle:
            self.motion_handle = HandleInMotion(item, self.grabbed_handle, self.view)
            self.motion_handle.start_move((x, y))
        self.motion_handle.move((x, y))

        if resize_recursive:
            item.resize_all_children(old_size)
        if isinstance(item, StateView):
            item.update_minimum_size_of_children()

    def end(self, x, y, modifiers):
        self._stop_autoscroll()
        if self.grabbed_item:
            item = self.grabbed_item

            # A handle was moved. Store the corresponding data into the meta data.
            if self.motion_handle:
                self.motion_handle.stop_move()
                graphical_editor = self.view.graphical_editor
                if isinstance(item, NameView):
                    gap_helper.update_meta_data_for_name_view(graphical_editor, item, publish=True)
                elif isinstance(item, ConnectionView):
                    merge_connection_segments(self.view, item, self.grabbed_handle)
                    gap_helper.update_meta_data_for_connection_waypoints(graphical_editor, item, None)
                else:  # StateView
                    if self.grabbed_handle in [port.handle for port in item.get_all_ports()]:
                        gap_helper.update_meta_data_for_port(graphical_editor, item, self.grabbed_handle)
                    else:
                        gap_helper.update_meta_data_for_state_view(graphical_editor, item, affects_children=True,
                                                                   publish=True)
            # The handle was not moved. Check if the handle is to be selected.
            else:
                # Only handles belonging to a state (i.e. port handles) can be selected
                if isinstance(item, StateView):
                    corresponding_ports = [port for port in item.get_all_ports() if port.handle is self.grabbed_handle]
                    if corresponding_ports:  # should be exactly one
                        self.view.handle_new_selection(corresponding_ports[0])

        self.grabbed_item = None
        self.grabbed_handle = None
        self.motion_handle = None


class ConnectionToolMode(AutoscrollMixin, DragMode):
    """Base for creating and modifying connections"""

    def __init__(self, view):
        super(ConnectionToolMode, self).__init__(view)
        self.__init_mixin__()
        self._connection_v = None
        self._start_port_v = None
        self._parent_state_v = None
        self._is_transition = False
        self._current_sink = None
        self.grabbed_item = None
        self.grabbed_handle = None
        self.motion_handle = None

    def end(self, x, y, modifiers):
        self._stop_autoscroll()
        self._is_transition = False
        self._connection_v = None
        self._start_port_v = None
        self._parent_state_v = None
        self._current_sink = None
        self.grabbed_item = None
        self.grabbed_handle = None
        self.motion_handle = None

    def _set_motion_handle(self, pos):
        """Sets motion handle to currently grabbed handle"""
        item = self.grabbed_item
        handle = self.grabbed_handle
        self.motion_handle = HandleInMotion(item, handle, self.view)
        self.motion_handle.GLUE_DISTANCE = self._parent_state_v.border_width
        self.motion_handle.start_move(pos)

    def _create_temporary_connection(self):
        """Creates a placeholder connection view

        :return: New placeholder connection
        :rtype: rafcon.gui.mygaphas.items.connection.ConnectionPlaceholderView
        """
        canvas = self.view.canvas
        if self._is_transition:
            self._connection_v = TransitionPlaceholderView(canvas, self._parent_state_v.hierarchy_level)
        else:
            self._connection_v = DataFlowPlaceholderView(canvas, self._parent_state_v.hierarchy_level)
        canvas.add(self._connection_v, self._parent_state_v)

    def _handle_temporary_connection(self, old_sink, new_sink, of_target=True):
        """Connect connection to new_sink

        If new_sink is set, the connection origin or target will be set to new_sink. The connection to old_sink is
        being removed.

        :param old_sink: Old sink (if existing)
        :param new_sink: New sink (if existing)
        :param bool of_target: Whether the origin or target will be reconnected
        """

        def sink_set_and_differs(sink_a, sink_b):
            if not sink_a:
                return False
            if not sink_b:
                return True
            if sink_a.port != sink_b.port:
                return True
            return False

        if sink_set_and_differs(old_sink, new_sink) and old_sink.port is not None:
            sink_port_v = old_sink.port.port_v
            self._disconnect_temporarily(sink_port_v, target=of_target)

        if sink_set_and_differs(new_sink, old_sink) and new_sink.port is not None:
            sink_port_v = new_sink.port.port_v
            self._connect_temporarily(sink_port_v, target=of_target)

    def _connect_temporarily(self, port_v, target=True):
        """Set a connection between the current connection and the given port

        :param rafcon.gui.mygaphas.items.ports.PortView port_v: The port to be connected
        :param bool target: Whether the connection origin or target should be connected
        """
        if target:
            handle = self._connection_v.to_handle()
        else:
            handle = self._connection_v.from_handle()
        port_v.add_connected_handle(handle, self._connection_v, moving=True)
        port_v.tmp_connect(handle, self._connection_v)
        self._connection_v.set_port_for_handle(port_v, handle)
        # Redraw state of port to make hover state visible
        self._redraw_port(port_v)

    def _disconnect_temporarily(self, port_v, target=True):
        """Removes a connection between the current connection and the given port

        :param rafcon.gui.mygaphas.items.ports.PortView port_v: The port that was connected
        :param bool target: Whether the connection origin or target should be disconnected
        """
        if target:
            handle = self._connection_v.to_handle()
        else:
            handle = self._connection_v.from_handle()
        port_v.remove_connected_handle(handle)
        port_v.tmp_disconnect()
        self._connection_v.reset_port_for_handle(handle)
        # Redraw state of port to make hover state visible
        self._redraw_port(port_v)

    def _redraw_port(self, port_v):
        self.view.queue_draw_area(*port_v.get_port_area(self.view))


class ConnectionCreationMode(ConnectionToolMode):
    """Create new connections by dragging from a port"""

    def begin(self, x, y, modifiers):
        view = self.view

        item, handle = HandleFinder(view.hovered_item, view).get_handle_at_point((x, y))

        if not handle:  # Require a handle
            return False

        # Connection handle must belong to a port and the MOVE_PORT_MODIFIER must not be pressed
        if not isinstance(item, StateView) or handle not in [port.handle for port in item.get_all_ports()] or (
                modifiers & constants.MOVE_PORT_MODIFIER):
            return False

        for port in item.get_all_ports():
            if port.handle is handle:
                self._start_port_v = port
                if port in item.get_logic_ports():
                    self._is_transition = True
                if port is item.income or isinstance(port, InputPortView) or port in item.scoped_variables:
                    self._parent_state_v = port.parent
                elif port.parent.parent:
                    self._parent_state_v = port.parent.parent
                else:  # Outgoing port of the root state was clicked on, no connection can be drawn here
                    self._parent_state_v = None

        return True

    def update(self, x, y, modifiers):
        if not self._parent_state_v:
            return

        if not self._connection_v:
            # Create new temporary connection, with origin at the start port and target at the cursor
            self._create_temporary_connection()
            self._start_port_v.parent.connect_connection_to_port(self._connection_v, self._start_port_v,
                                                                 as_target=False)
            self.grabbed_item = self._connection_v
            self.grabbed_handle = self._connection_v.to_handle()
            self._set_motion_handle((x, y))

        last_sink = self._current_sink
        self._current_sink = self.motion_handle.move((x, y))
        self.handle_autoscroll(x, y)

        self._handle_temporary_connection(last_sink, self._current_sink, of_target=True)

    @avoid_parallel_execution
    def end(self, x, y, modifiers):
        # A temporary connection was created. Check if it is a valid one.
        if self._connection_v:
            self.view.canvas.update_now()
            if self._current_sink:
                if self.motion_handle:
                    self.motion_handle.stop_move()
                sink_model = self._current_sink.item.model
                if self._current_sink.port is not None:
                    sink_port_v = self._current_sink.port.port_v
                    sink_model = sink_port_v.model
                    self._disconnect_temporarily(sink_port_v, target=True)
                gap_helper.create_new_connection(self._connection_v.from_port.model, sink_model)

            # remove placeholder from canvas
            if self._connection_v:
                self._connection_v.remove_connection_from_ports()
                self.view.canvas.remove(self._connection_v)

        # No connection was created, but only a handle was clicked on. Check whether it is to be selected
        else:
            self.view.handle_new_selection(self._start_port_v)

        super(ConnectionCreationMode, self).end(x, y, modifiers)


class ConnectionModificationMode(ConnectionToolMode):
    """Modify the origin or target of an existing connection"""

    def __init__(self, view):
        super(ConnectionModificationMode, self).__init__(view)
        self._end_handle = None

    def begin(self, x, y, modifiers):
        view = self.view

        item, handle = HandleFinder(view.hovered_item, view).get_handle_at_point((x, y))

        # Handle must be the end handle of a connection
        if not handle or not isinstance(item, ConnectionView) or handle not in item.end_handles():
            return False

        if handle is item.from_handle():
            self._start_port_v = item.from_port
        else:
            self._start_port_v = item.to_port

        self._parent_state_v = item.parent
        self._end_handle = handle
        if isinstance(item, TransitionView):
            self._is_transition = True
        self._connection_v = item

        return True

    def update(self, x, y, modifiers):
        if not self._parent_state_v:
            return

        modify_target = self._end_handle is self._connection_v.to_handle()

        if not self.grabbed_handle:
            self.view.canvas.disconnect_item(self._connection_v, self._end_handle)
            self._disconnect_temporarily(self._start_port_v, target=modify_target)
            self.grabbed_item = self._connection_v
            self.grabbed_handle = self._end_handle
            self._set_motion_handle((x, y))

        last_sink = self._current_sink
        self._current_sink = self.motion_handle.move((x, y))
        self.handle_autoscroll(x, y)

        self._handle_temporary_connection(last_sink, self._current_sink, modify_target)

    def end(self, x, y, modifiers):
        if not self.grabbed_handle:
            self._end_handle = None
            return super(ConnectionModificationMode, self).end(x, y, modifiers)

        modify_target = self._end_handle is self._connection_v.to_handle()
        self._handle_temporary_connection(self._current_sink, None, of_target=modify_target)

        if not self._current_sink or not self._connection_v.from_port:
            # Reset connection to original status if new connection is not defined properly
            self._reset_connection()
        elif not self._current_sink.port:   # Try to create a port if it was released above a state
            self.view.canvas.update_now()
            if self.motion_handle:
                self.motion_handle.stop_move()
            # Create new connection
            from_port_model = self._connection_v.from_port.model
            to_port_model = self._current_sink.item.model
            if gap_helper.create_new_connection(from_port_model, to_port_model):
                # Remove placeholder from canvas
                connection_v = self._connection_v
                connection_v.remove_connection_from_ports()
                self.view.canvas.remove(connection_v)
                # Delete old connection
                data_flow_container_state = connection_v.from_port.model.parent.state.parent
                from_state_id = connection_v.model.core_element.from_state
                to_state_id = connection_v.model.core_element.to_state
                data_flow_id = data_flow_container_state.get_data_flow_id(from_state_id, to_state_id)
                data_flow_container_state.remove_data_flow(data_flow_id)
            else:
                self._reset_connection()
        else:  # Modify the source/target of the connection
            connection_core_element = self._connection_v.model.core_element
            try:
                port_core_element = self._current_sink.port.port_v.model.core_element
                port_id = port_core_element.state_element_id
                port_state_id = port_core_element.parent.state_id
            except AttributeError:  # Port is an income
                port_id = None
                port_state_id = self._current_sink.port.port_v.parent.model.state.state_id if modify_target else None

            try:
                if modify_target:
                    connection_core_element.modify_target(port_state_id, port_id)
                else:
                    connection_core_element.modify_origin(port_state_id, port_id)
            except ValueError as e:
                self._reset_connection()
                logger.error(e)
        self.view.canvas.update_now()
        self._end_handle = None
        super(ConnectionModificationMode, self).end(x, y, modifiers)

    def _reset_connection(self):
        modify_target = self._end_handle is self._connection_v.to_handle()
        self._start_port_v.parent.connect_connection_to_port(self._connection_v, self._start_port_v,
                                                             as_target=modify_target)
        self._redraw_port(self._start_port_v)


class MoveItemMode(AutoscrollMixin, DragMode):
    """This class is responsible for moving states, names, connections, etc."""

    def __init__(self, view):
        super(MoveItemMode, self).__init__(view)
        self.__init_mixin__()
        self._item = None
        self._move_name_v = False
        self._old_selection = None
        self._movable_items = []
        self._start_pos = (0, 0)

    def movable_items(self):
        """Filter selection

        Filter items of the selection that cannot be moved and return the rest.
        """
        view = self.view

        if self._move_name_v:
            yield InMotion(self._item, view)
        else:
            selected_items = set(view.selected_items)
            for item in selected_items:
                if not isinstance(item, (StateView, NameView, ConnectionView)):
                    continue
                yield InMotion(item, view)

    def begin(self, x, y, modifiers):
        """Select items

        When the mouse button is pressed, the selection is updated.
        """
        self._start_pos = (x, y)
        self._movable_items = []

        # Special case: moving the NameView
        # This is only allowed, if the hovered item is a NameView and the Ctrl-key is pressed and the only selected
        # item is the parental StateView. In this case, the selection and _item will no longer be looked at,
        # but only _move_name_v
        self._item = self.view.hovered_item
        if isinstance(self._item, NameView):
            selected_items = self.view.selected_items
            if modifiers & Gdk.ModifierType.CONTROL_MASK and len(selected_items) == 1 and \
                    next(iter(selected_items)) is self._item.parent:
                self._move_name_v = True
            else:
                self._item = self._item.parent

        if not self._move_name_v:
            self._old_selection = self.view.selected_items
            if self._item not in self.view.selected_items:
                # When items are to be moved, a button-press should not cause any deselection.
                # However, the selection is stored, in case no move operation is performed.
                self.view.handle_new_selection(self._item)

        return True

    def update(self, x, y, modifiers):
        """Autoscroll on drag and move the selected items

        If one or more items are moved against the border of the graphical editor view, the view is moved into
        the direction of the border threshold.
        """
        if not self._movable_items:
            self._movable_items = list(self.movable_items())
            for inmotion in self._movable_items:
                inmotion.start_move(self._start_pos)

        self.handle_autoscroll(x, y)

        for inmotion in self._movable_items:
            inmotion.move((x, y))

    def end(self, x, y, modifiers):
        """Write back changes

        If one or more items have been moved, the new position are stored in the corresponding meta data and a signal
        notifying the change is emitted.
        """
        self._stop_autoscroll()
        affected_models = {}

        for inmotion in self._movable_items:
            inmotion.stop_move()
            rel_pos = gap_helper.calc_rel_pos_to_parent(self.view.canvas, inmotion.item,
                                                        inmotion.item.handles()[NW])
            if isinstance(inmotion.item, StateView):
                state_v = inmotion.item
                state_m = state_v.model
                self.view.canvas.request_update(state_v)
                if state_m.get_meta_data_editor()['rel_pos'] != rel_pos:
                    state_m.set_meta_data_editor('rel_pos', rel_pos)
                    affected_models[state_m] = ("position", True, state_v)
            elif isinstance(inmotion.item, NameView):
                state_v = inmotion.item
                state_m = self.view.canvas.get_parent(state_v).model
                self.view.canvas.request_update(state_v)
                if state_m.get_meta_data_editor()['name']['rel_pos'] != rel_pos:
                    state_m.set_meta_data_editor('name.rel_pos', rel_pos)
                    affected_models[state_m] = ("name_position", False, state_v)
            elif isinstance(inmotion.item, (TransitionView, DataFlowView)):
                connection_v = inmotion.item
                connection_m = connection_v.model
                self.view.canvas.request_update(connection_v)
                current_waypoints = gap_helper.get_relative_positions_of_waypoints(connection_v)
                old_waypoints = connection_m.get_meta_data_editor()['waypoints']
                if current_waypoints != old_waypoints:
                    connection_m.set_meta_data_editor('waypoints', current_waypoints)
                    affected_models[connection_m] = ("waypoints", False, connection_v)

        if len(affected_models) == 1:
            model = next(iter(affected_models))
            change, affects_children, view = affected_models[model]
            self.view.graphical_editor.emit('meta_data_changed', model, change, affects_children)
        elif len(affected_models) > 1:
            # if more than one item has been moved, we need to call the meta_data_changed signal on a common parent
            common_parents = None
            for change, affects_children, view in affected_models.values():
                parents_of_view = set(self.view.canvas.get_ancestors(view))
                if common_parents is None:
                    common_parents = parents_of_view
                else:
                    common_parents = common_parents.intersection(parents_of_view)
            assert len(common_parents) > 0, "The selected elements do not have common parent element"
            for state_v in common_parents:
                # Find most nested state_v
                children_of_state_v = self.view.canvas.get_all_children(state_v)
                if any(common_parent in children_of_state_v for common_parent in common_parents):
                    continue
                self.view.graphical_editor.emit('meta_data_changed', state_v.model, "positions", True)
                break

        if not affected_models and self._old_selection is not None:
            # The selection is handled differently depending on whether states were moved or not:
            # The state the user clicked on is always added to the selection in the `begin` handler, which is
            # fine if the states were moved.
            # If the states were not moved (no `affected_models`), and if the state the user clicked on had already
            # been selected, there are two cases to be considered:
            # 1. extend-selection-modifier is clicked: we need to remove the state from the selection
            # 2. extend-selection-modifier is not clicked: we need to remove all other states from the selection
            from rafcon.gui.models.selection import extend_selection
            if self._item in self._old_selection:
                if extend_selection():
                    self.view.unselect_item(self._item)
                else:
                    for item_v in [item_v for item_v in self._old_selection if item_v is not self._item]:
                        self.view.unselect_item(item_v)

        self._move_name_v = False
        self._old_selection = None
        self._movable_items = []
        self._item = None


class RubberbandMode(DragMode):
    """Rubberband selection of multiple items"""

    def begin(self, x, y, modifiers):
        if not modifiers & constants.RUBBERBAND_MODIFIER:
            return False
        rubberband_state = self.view.rubberband_state
        rubberband_state.x0 = rubberband_state.x1 = x
        rubberband_state.y0 = rubberband_state.y1 = y
        return True

    def update(self, x, y, modifiers):
        rubberband_state = self.view.rubberband_state
        rubberband_state.x1 = x
        rubberband_state.y1 = y
        self.view.update_back_buffer()

    def end(self, x, y, modifiers):
        """Select or deselect rubber banded groups of items"""
        rubberband_state = self.view.rubberband_state
        x0, y0, x1, y1 = rubberband_state.x0, rubberband_state.y0, x, y
        rectangle = (min(x0, x1), min(y0, y1), abs(x1 - x0), abs(y1 - y0))
        selected_items = self.view.get_items_in_rectangle(rectangle, contain=True)
        self.view.handle_new_selection(selected_items)
        rubberband_state.reset()
        self.view.update_back_buffer()


class PrimaryDragDispatcher:
    """Dispatches button-1 drags to the drag mode claiming the press

    The dispatch order mirrors the gaphas 2.x tool chain priority:
    handle move > connection creation > connection modification > item move,
    with rubberband selection taking precedence when its modifier is pressed.
    """

    def __init__(self, view):
        self.view = view
        if not hasattr(view, 'rubberband_state'):
            view.rubberband_state = RubberbandState()
        self._rubberband_mode = RubberbandMode(view)
        self._modes = [
            HandleMoveMode(view),
            ConnectionCreationMode(view),
            ConnectionModificationMode(view),
            MoveItemMode(view),
        ]
        self._active_mode = None
        self._start_pos = (0, 0)

        self.gesture = Gtk.GestureDrag.new()
        self.gesture.set_button(Gdk.BUTTON_PRIMARY)
        self.gesture.connect('drag-begin', self._on_drag_begin)
        self.gesture.connect('drag-update', self._on_drag_update)
        self.gesture.connect('drag-end', self._on_drag_end)

    @property
    def is_dragging(self):
        return self._active_mode is not None

    def _on_drag_begin(self, gesture, start_x, start_y):
        view = self.view
        if not view.is_focus():
            view.grab_focus()
        self._start_pos = (start_x, start_y)
        modifiers = gesture.get_current_event_state()
        self._active_mode = None

        if modifiers & constants.RUBBERBAND_MODIFIER:
            candidates = [self._rubberband_mode]
        else:
            candidates = self._modes

        for mode in candidates:
            try:
                if mode.begin(start_x, start_y, modifiers):
                    self._active_mode = mode
                    break
            except Exception:
                logger.exception("Error in drag mode {0}".format(mode.__class__.__name__))

        if self._active_mode:
            gesture.set_state(Gtk.EventSequenceState.CLAIMED)

    def _on_drag_update(self, gesture, offset_x, offset_y):
        if not self._active_mode:
            return
        x = self._start_pos[0] + offset_x
        y = self._start_pos[1] + offset_y
        modifiers = gesture.get_current_event_state()
        try:
            self._active_mode.update(x, y, modifiers)
        except Exception:
            logger.exception("Error in drag mode {0}".format(self._active_mode.__class__.__name__))

    def _on_drag_end(self, gesture, offset_x, offset_y):
        if not self._active_mode:
            return
        x = self._start_pos[0] + offset_x
        y = self._start_pos[1] + offset_y
        modifiers = gesture.get_current_event_state()
        mode, self._active_mode = self._active_mode, None
        try:
            mode.end(x, y, modifiers)
        except Exception:
            logger.exception("Error in drag mode {0}".format(mode.__class__.__name__))


class HoverItemTool:
    """Sets the hovered item and adapts the mouse cursor"""

    def __init__(self, view, drag_dispatcher=None):
        self.view = view
        self._drag_dispatcher = drag_dispatcher
        self._prev_hovered_item = None
        self.controller = Gtk.EventControllerMotion.new()
        self.controller.connect('motion', self._on_motion)

    @staticmethod
    def dismiss_upper_items(items, item):
        try:
            return items[items.index(item):]
        except ValueError:
            return []

    def _filter_library_state(self, items):
        """Filters out child elements of library state when they cannot be hovered

        Checks if hovered item is within a LibraryState
        * if not, the list is returned unfiltered
        * if so, STATE_SELECTION_INSIDE_LIBRARY_STATE_ENABLED is checked
            * if enabled, the library is selected (instead of the state copy)
            * if not, the upper most library is selected

        :param list items: Sorted list of items beneath the cursor
        :return: filtered items
        :rtype: list
        """
        if not items:
            return items

        top_most_item = items[0]
        # If the hovered item is e.g. a connection, we need to get the parental state
        top_most_state_v = top_most_item if isinstance(top_most_item, StateView) else top_most_item.parent
        state = top_most_state_v.model.state

        config = gui_helper_state_machine.global_gui_config
        if config.get_config_value('STATE_SELECTION_INSIDE_LIBRARY_STATE_ENABLED'):
            # select the library state instead of the library_root_state because it is hidden
            if state.is_root_state_of_library:
                new_topmost_item = self.view.canvas.get_view_for_core_element(state.parent)
                return self.dismiss_upper_items(items, new_topmost_item)
            return items
        else:
            # Find state_copy of uppermost LibraryState
            library_root_state = state.get_uppermost_library_root_state()

            # If the hovered element is a child of a library, make the library the hovered_item
            if library_root_state:
                library_state = library_root_state.parent
                library_state_v = self.view.canvas.get_view_for_core_element(library_state)
                return self.dismiss_upper_items(items, library_state_v)
            return items

    def _filter_hovered_items(self, items, x, y):
        """Filters out items that cannot be hovered

        :param list items: Sorted list of items beneath the cursor
        :return: filtered items
        :rtype: list
        """
        items = self._filter_library_state(items)
        if not items:
            return items
        top_most_item = items[0]
        second_top_most_item = items[1] if len(items) > 1 else None

        # States/Names take precedence over connections if the connections are on the same hierarchy and if there is
        # a port beneath the cursor
        first_state_v = next(filter(lambda item: isinstance(item, (NameView, StateView)), items), None)
        if isinstance(first_state_v, NameView):
            first_state_v = first_state_v.parent
        if first_state_v:
            # There can be several connections above the state/name; skip those and find the first non-connection-item
            item = top_most_item
            for item in items:
                if isinstance(item, ConnectionView):
                    # connection is on the same hierarchy level as the state/name, thus we dismiss it
                    if self.view.canvas.get_parent(top_most_item) is not first_state_v:
                        continue
                break

            # Connections are only dismissed, if there is a port beneath the cursor. Search for ports here:
            port_beneath_cursor = False
            state_ports = first_state_v.get_all_ports()
            position = self.view.get_matrix_v2i(first_state_v).transform_point(x, y)
            i2v_matrix = self.view.get_matrix_i2v(first_state_v)
            for port_v in state_ports:
                item_distance = port_v.port.glue(position)[1]
                view_distance = i2v_matrix.transform_distance(item_distance, 0)[0]
                if view_distance == 0:
                    port_beneath_cursor = True
                    break

            if port_beneath_cursor:
                items = self.dismiss_upper_items(items, item)
                top_most_item = items[0] if items else None
                second_top_most_item = items[1] if len(items) > 1 else None

        # NameView can only be hovered if it or its parent state is selected
        if isinstance(top_most_item, NameView):
            state_v = second_top_most_item  # second item in the list must be the parent state of the NameView
            if state_v not in self.view.selected_items and top_most_item not in self.view.selected_items:
                items = items[1:]

        return items

    def _set_cursor(self, cursor_name):
        self.view.set_cursor(Gdk.Cursor.new_from_name(cursor_name))

    def _on_motion(self, controller, x, y):
        view = self.view
        view._last_motion_pos = (x, y)

        # While a drag operation is in progress, the hovered item must not change
        if self._drag_dispatcher and self._drag_dispatcher.is_dragging:
            return

        modifiers = controller.get_current_event_state()
        hovered_items = view.get_items_at_point((x, y), distance=3)
        hovered_items = self._filter_hovered_items(hovered_items, x, y)

        view.hovered_item = hovered_items[0] if hovered_items else None

        if view.hovered_handle:
            handle = view.hovered_handle
            view.hovered_handle = None
            port_v = self._prev_hovered_item.get_port_for_handle(handle)
            view.queue_draw_area(*port_v.get_port_area(view))
        pos = x, y

        # Reset cursor
        self._set_cursor("default")

        def handle_hover_of_port(hovered_handle):
            view.hovered_handle = hovered_handle
            port_v = state_v.get_port_for_handle(hovered_handle)
            view.queue_draw_area(*port_v.get_port_area(view))
            if modifiers & constants.MOVE_PORT_MODIFIER:
                self._set_cursor(constants.MOVE_CURSOR)
            else:
                self._set_cursor(constants.CREATION_CURSOR)

        if isinstance(view.hovered_item, PortView):
            if modifiers & constants.MOVE_PORT_MODIFIER:
                self._set_cursor(constants.MOVE_CURSOR)
            else:
                self._set_cursor(constants.CREATION_CURSOR)

        elif isinstance(view.hovered_item, StateView):
            distance = view.hovered_item.border_width / 2.
            state_v, hovered_handle = HandleFinder(view.hovered_item, view).get_handle_at_point(pos, distance)

            # Hover over port => show hover state of port and different cursor
            if hovered_handle and hovered_handle not in state_v.corner_handles:
                handle_hover_of_port(hovered_handle)

            # Hover over corner/resize handles => show with cursor
            elif hovered_handle and hovered_handle in state_v.corner_handles:
                index = state_v.handles().index(hovered_handle)
                self._set_cursor(ELEMENT_CURSORS[index])

        # NameView should only be hovered, if its state is selected
        elif isinstance(view.hovered_item, NameView):
            state_v = self.view.canvas.get_parent(view.hovered_item)
            distance = state_v.border_width / 2.
            _, hovered_handle = HandleFinder(state_v, view).get_handle_at_point(pos, distance)

            # Hover over port => show hover state of port and different cursor
            if hovered_handle and hovered_handle not in state_v.corner_handles:
                view.hovered_item = state_v
                handle_hover_of_port(hovered_handle)
            else:
                name_v, hovered_handle = HandleFinder(view.hovered_item, view).get_handle_at_point(pos)
                # Hover over corner/resize handles => show with cursor
                if name_v:
                    index = name_v.handles().index(hovered_handle)
                    self._set_cursor(ELEMENT_CURSORS[index])

        # Change mouse cursor to indicate option to move connection
        elif isinstance(view.hovered_item, ConnectionView):
            # If a connection handle is hovered, show the move cursor
            item, handle = HandleFinder(view.hovered_item, view).get_handle_at_point(pos, split=False)
            if handle:
                self._set_cursor(constants.MOVE_CURSOR)
            # If no handle is hovered, indicate the option for selection with the selection cursor
            else:
                self._set_cursor(constants.SELECT_CURSOR)

        if isinstance(view.hovered_item, StateView):
            self._prev_hovered_item = view.hovered_item


def create_pan_gesture(view):
    """Middle mouse button drag pans the canvas"""
    gesture = Gtk.GestureDrag.new()
    gesture.set_button(Gdk.BUTTON_MIDDLE)
    pan_state = {'x0': 0, 'y0': 0}

    def on_begin(gesture_, start_x, start_y):
        pan_state['x0'] = view.matrix[4]
        pan_state['y0'] = view.matrix[5]
        view.set_cursor(Gdk.Cursor.new_from_name(constants.MOVE_CURSOR))
        gesture_.set_state(Gtk.EventSequenceState.CLAIMED)

    def on_update(gesture_, offset_x, offset_y):
        view.matrix.set(x0=pan_state['x0'] + offset_x, y0=pan_state['y0'] + offset_y)
        view.update_back_buffer()

    def on_end(gesture_, offset_x, offset_y):
        view.set_cursor(Gdk.Cursor.new_from_name("default"))

    gesture.connect('drag-begin', on_begin)
    gesture.connect('drag-update', on_update)
    gesture.connect('drag-end', on_end)
    return gesture


def create_scroll_controller(view):
    """Scroll wheel handling: zooming and scrolling, depending on the ZOOM_WITH_CTRL config"""
    controller = Gtk.EventControllerScroll.new(Gtk.EventControllerScrollFlags.BOTH_AXES)
    scroll_speed = 50.

    def zoom_at_pointer(dy):
        pos = getattr(view, '_last_motion_pos', None)
        if pos is None:
            pos = (view.get_width() / 2., view.get_height() / 2.)
        sx = view.matrix[0]
        sy = view.matrix[3]
        ox = (view.matrix[4] - pos[0]) / sx
        oy = (view.matrix[5] - pos[1]) / sy
        factor = 0.9 if dy > 0 else 1. / 0.9

        current_zoom = view.matrix[0]
        new_zoom = current_zoom * factor
        if new_zoom < 0.0001 or new_zoom > 100000:
            return True

        view.matrix.translate(-ox, -oy)
        view.matrix.scale(factor, factor)
        view.matrix.translate(+ox, +oy)
        # Make sure everything's updated
        if view.model:
            view.request_update(view.model.get_all_items())
        return True

    def on_scroll(controller_, dx, dy):
        zoom_with_control = global_gui_config.get_config_value("ZOOM_WITH_CTRL", False)
        ctrl_key_pressed = bool(controller_.get_current_event_state() & Gdk.ModifierType.CONTROL_MASK)
        if (zoom_with_control and ctrl_key_pressed) or (not zoom_with_control and not ctrl_key_pressed):
            return zoom_at_pointer(dy)
        else:
            shift_pressed = bool(controller_.get_current_event_state() & Gdk.ModifierType.SHIFT_MASK)
            hadj = view.hadjustment
            vadj = view.vadjustment
            if shift_pressed:
                dx, dy = dy, dx
            hadj.set_value(hadj.get_value() + dx * scroll_speed)
            vadj.set_value(vadj.get_value() + dy * scroll_speed)
            return True

    controller.connect('scroll', on_scroll)
    return controller


def create_right_click_gesture(view):
    """Right click opens the state machine right click menu"""
    gesture = Gtk.GestureClick.new()
    gesture.set_button(Gdk.BUTTON_SECONDARY)
    # TODO correct destruction of the StateRightClickMenu-Controller
    sm_right_click_menu = StateRightClickMenuGaphas()

    def on_pressed(gesture_, n_press, x, y):
        event = ButtonEventShim(x, y, button=3)
        sm_right_click_menu.mouse_click(view, event)

    gesture.connect('pressed', on_pressed)
    # keep the menu controller alive
    gesture.sm_right_click_menu = sm_right_click_menu
    return gesture
