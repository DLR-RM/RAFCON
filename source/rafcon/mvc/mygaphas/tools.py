import gtk
from gtk.gdk import CONTROL_MASK
from enum import Enum
from math import pow

from gaphas.aspect import HandleFinder, ItemConnectionSink, Connector, InMotion
from gaphas.tool import Tool, ItemTool, HoverTool, HandleTool, ConnectHandleTool, RubberbandTool
from gaphas.item import NW

from rafcon.mvc.mygaphas.aspect import HandleInMotion, StateHandleFinder
from rafcon.mvc.mygaphas.items.connection import ConnectionView, ConnectionPlaceholderView, \
    TransitionPlaceholderView, DataFlowPlaceholderView, TransitionView, DataFlowView
from rafcon.mvc.mygaphas.items.ports import IncomeView, OutcomeView, InputPortView, OutputPortView, \
    ScopedVariablePortView
from rafcon.mvc.mygaphas.items.state import StateView, NameView
from rafcon.mvc.mygaphas.utils import gap_helper

from rafcon.mvc.controllers.right_click_menu.state import StateRightClickMenuGaphas

from rafcon.mvc import state_machine_helper
from rafcon.mvc.config import global_gui_config
from rafcon.mvc.gui_helper import react_to_event
from rafcon.mvc.utils import constants
from rafcon.utils import log

logger = log.get_logger(__name__)

PortMoved = Enum('PORT', 'FROM TO')


class RemoveItemTool(Tool):
    """This tool is responsible of deleting the selected item
    """

    def __init__(self, graphical_editor_view, view=None):
        super(RemoveItemTool, self).__init__(view)
        self._graphical_editor_view = graphical_editor_view

    def on_key_release(self, event):
        if gtk.gdk.keyval_name(event.keyval) == "Delete":
            # Delete Transition from state machine
            if isinstance(self.view.focused_item, TransitionView):
                state_machine_helper.delete_model(self.view.focused_item.model)
                return True
            # Delete DataFlow from state machine
            if isinstance(self.view.focused_item, DataFlowView):
                state_machine_helper.delete_model(self.view.focused_item.model)
                return True
            # Delete selected state(s) from state machine
            if isinstance(self.view.focused_item, StateView):
                if react_to_event(self.view, self.view, event):
                    self._graphical_editor_view.emit('remove_state_from_state_machine')
                    return True


class MoveItemTool(ItemTool):
    """This class is responsible of moving states, names, connections, etc.
    """

    def __init__(self, graphical_editor_view, view=None, buttons=(1,)):
        super(MoveItemTool, self).__init__(view, buttons)
        self._graphical_editor_view = graphical_editor_view
        self._move_name_v = False

        self._item = None
        self._do_not_unselect = None

    def movable_items(self):
        if self._do_not_unselect:
            self.view.focused_item = self._do_not_unselect

        if self._move_name_v:
            return [InMotion(self._item, self.view)]
        else:
            return super(MoveItemTool, self).movable_items()

    def on_button_press(self, event):
        # print "on_press_button: ", self.__class__.__name__

        if event.button not in self._buttons:
            return False  # Only handle events for registered buttons (left mouse clicks)

        if event.state & gtk.gdk.SHIFT_MASK:
            return False  # Mouse clicks with pressed shift key are handled in another tool

        self._item = self.get_item()

        # NameView can only be moved when the Ctrl-key is pressed
        self._move_name_v = isinstance(self._item, NameView) and event.state & gtk.gdk.CONTROL_MASK
        if self._item in self.view.selected_items and \
                isinstance(self._item, NameView) and event.state & gtk.gdk.CONTROL_MASK:
            # self.view.unselect_item(self._item)
            pass
        else:
            if not event.state & gtk.gdk.CONTROL_MASK and self._item not in self.view.selected_items:
                del self.view.selected_items
            if self._item not in self.view.selected_items:
                # remember items that should not be unselected and maybe focused if movement occur
                self._do_not_unselect = self._item

        if not self.view.is_focus():
            self.view.grab_focus()

        return True

    def on_button_release(self, event):
        # print "on_release_button: ", self.__class__.__name__

        position_changed = False
        for inmotion in self._movable_items:
            inmotion.move((event.x, event.y))
            rel_pos = gap_helper.calc_rel_pos_to_parent(self.view.canvas, inmotion.item,
                                                        inmotion.item.handles()[NW])
            if isinstance(inmotion.item, StateView):
                state_m = inmotion.item.model
                if state_m.meta['gui']['editor_gaphas']['rel_pos'] != rel_pos:
                    position_changed = True
                    state_m.meta['gui']['editor_gaphas']['rel_pos'] = rel_pos
                    state_m.meta['gui']['editor_opengl']['rel_pos'] = (rel_pos[0], -rel_pos[1])
            elif isinstance(inmotion.item, NameView):
                state_m = self.view.canvas.get_parent(inmotion.item).model
                if state_m.meta['gui']['editor_gaphas']['name']['rel_pos'] != rel_pos:
                    state_m.meta['gui']['editor_gaphas']['name']['rel_pos'] = rel_pos
                    position_changed = True

        if isinstance(self._item, StateView):
            self.view.canvas.request_update(self._item)
            if position_changed:
                self._graphical_editor_view.emit('meta_data_changed', self._item.model, "position", True)

        if isinstance(self.view.focused_item, NameView):
            if position_changed:
                self._graphical_editor_view.emit('meta_data_changed', self.view.focused_item.parent.model,
                                                 "name_position", False)

        if not position_changed:
            if self._item in self.view.selected_items and event.state & gtk.gdk.CONTROL_MASK:
                if not self._do_not_unselect is self._item:
                    self.view.unselect_item(self._item)
            else:
                if not event.state & gtk.gdk.CONTROL_MASK:
                    del self.view.selected_items
                self.view.focused_item = self._item
        self._do_not_unselect = None

        return super(MoveItemTool, self).on_button_release(event)


class HoverItemTool(HoverTool):
    def __init__(self, view=None):
        super(HoverItemTool, self).__init__(view)
        self._prev_hovered_item = None

    def on_motion_notify(self, event):
        super(HoverItemTool, self).on_motion_notify(event)
        from gaphas.tool import HandleFinder
        from gaphas.view import DEFAULT_CURSOR
        from gaphas.aspect import ElementHandleSelection

        view = self.view
        if view.hovered_handle:
            handle = view.hovered_handle
            view.hovered_handle = None
            port_v = self._prev_hovered_item.get_port_for_handle(handle)
            view.queue_draw_area(*port_v.get_port_area(view))
        pos = event.x, event.y

        # Reset cursor
        self.view.window.set_cursor(gtk.gdk.Cursor(DEFAULT_CURSOR))

        if isinstance(view.hovered_item, StateView):
            state_v, hovered_handle = HandleFinder(view.hovered_item, view).get_handle_at_point(pos)

            # Hover over port => show hover state of port and different cursor
            if hovered_handle and hovered_handle not in state_v.corner_handles:
                view.hovered_handle = hovered_handle
                port_v = state_v.get_port_for_handle(hovered_handle)
                view.queue_draw_area(*port_v.get_port_area(view))
                if event.state & gtk.gdk.CONTROL_MASK:
                    self.view.window.set_cursor(gtk.gdk.Cursor(constants.MOVE_CURSOR))
                else:
                    self.view.window.set_cursor(gtk.gdk.Cursor(constants.CREATION_CURSOR))

            # Hover over corner/resize handles => show with cursor
            elif hovered_handle and hovered_handle in state_v.corner_handles:
                cursors = ElementHandleSelection.CURSORS
                index = state_v.handles().index(hovered_handle)
                self.view.window.set_cursor(cursors[index])

        # NameView should only be hovered, if its state is selected
        elif isinstance(view.hovered_item, NameView):
            state_v = self.view.canvas.get_parent(view.hovered_item)
            if state_v not in self.view.selected_items and view.hovered_item not in self.view.selected_items:
                view.hovered_item = state_v
            else:
                name_v, hovered_handle = HandleFinder(view.hovered_item, view).get_handle_at_point(pos)
                # Hover over corner/resize handles => show with cursor
                if hovered_handle:
                    index = name_v.handles().index(hovered_handle)
                    cursors = ElementHandleSelection.CURSORS
                    self.view.window.set_cursor(cursors[index])

        # Change mouse cursor to indicate option to move connection
        elif isinstance(view.hovered_item, ConnectionView):
            state_v = view.get_item_at_point_exclude(pos, selected=False, exclude=[view.hovered_item])
            if isinstance(state_v, StateView):
                distance = state_v.border_width / 2. * view.get_zoom_factor()
                connection_v, hovered_handle = StateHandleFinder(state_v, view).get_handle_at_point(pos, distance)
            else:
                # Temporarily unset focused item to prevent segments from being split
                focus = view.focused_item
                view.focused_item = None
                connection_v, hovered_handle = HandleFinder(view.hovered_item, view).get_handle_at_point(pos)
                view.focused_item = focus
            if hovered_handle:
                self.view.window.set_cursor(gtk.gdk.Cursor(constants.MOVE_CURSOR))
            else:
                self.view.window.set_cursor(gtk.gdk.Cursor(constants.SELECT_CURSOR))

        if isinstance(self.view.hovered_item, StateView):
            self._prev_hovered_item = self.view.hovered_item


class MultiSelectionTool(RubberbandTool):
    def __init__(self, graphical_editor_view, view=None):
        super(MultiSelectionTool, self).__init__(view)

        self._graphical_editor_view = graphical_editor_view

    def on_button_press(self, event):
        # print "on_button_press: ", self.__class__.__name__
        if event.state & gtk.gdk.SHIFT_MASK:
            return super(MultiSelectionTool, self).on_button_press(event)
        return False

    def on_motion_notify(self, event):
        if event.state & gtk.gdk.BUTTON_PRESS_MASK and event.state & gtk.gdk.SHIFT_MASK:
            view = self.view
            self.queue_draw(view)
            self.x1, self.y1 = event.x, event.y
            self.queue_draw(view)
            return True

    def on_button_release(self, event):
        """Select or deselect rubber banded groups of items

         The selection of elements is prior and never items are selected or deselected at the same time.
         """
        # print "on_release_press: ", self.__class__.__name__
        self.queue_draw(self.view)
        x0, y0, x1, y1 = self.x0, self.y0, self.x1, self.y1
        # Hold down Ctrl-key to add selection to current selection
        if event.state & gtk.gdk.CONTROL_MASK:
            old_items_selected = []
        else:
            old_items_selected = list(self.view.selected_items)
        for item in old_items_selected:
            if item in self.view.selected_items:
                self.view.unselect_item(item)
        self.view.select_in_rectangle((min(x0, x1), min(y0, y1), abs(x1 - x0), abs(y1 - y0)))

        old_items_in_new_selection = [item in self.view.selected_items for item in old_items_selected]
        actual_items_are_old_selection  = [item in old_items_selected for item in self.view.selected_items]
        rubber_band_selection = list(self.view.selected_items)
        new_selection = old_items_selected
        if any(old_items_in_new_selection) and not all(actual_items_are_old_selection): # reselect elements
            # add new  rubber band selection by preserving old state selection
            for item in rubber_band_selection:
                if item not in old_items_selected:
                    old_items_selected.append(item)
        else:
            if not any(actual_items_are_old_selection):
                # add rubber band selection
                for item in rubber_band_selection:
                    old_items_selected.append(item)
            else:
                # remove rubber band selection
                for item in rubber_band_selection:
                    old_items_selected.remove(item)

        # unselect views that are not representing states or old states -> unselect all
        items_intermediate_selected = list(old_items_selected) + list(rubber_band_selection)
        for item in self.view.selected_items:
            if not isinstance(item, StateView):
                items_intermediate_selected.append(item)

        for item in items_intermediate_selected:
            if item in self.view.selected_items:
                self.view.unselect_item(item)

        # select actual selection
        for item in new_selection:
            self.view.select_item(item)

        return True


class MoveHandleTool(HandleTool):
    """Tool to move handles around

    Handles can be moved using click'n'drag. This is already implemented in the base class `HandleTool`. This class
    extends the behaviour by requiring a modifier key to be pressed when moving ports. It also allows to change the
    modifier key, which are defined in `rafcon.mvc.utils.constants`.
    """

    def on_button_press(self, event):
        """Handle button press events.

        If the (mouse) button is pressed on top of a Handle (item.Handle), that handle is grabbed and can be
        dragged around.
        """
        if not event.button == 1:  # left mouse button
            return False
        view = self.view

        item, handle = HandleFinder(view.hovered_item, view).get_handle_at_point((event.x, event.y))

        if not handle:
            return False

        # Only move ports when the MOVE_PORT_MODIFIER key is pressed
        if isinstance(item, StateView) and handle in [port.handle for port in item.get_all_ports()] and not (
                    event.state & constants.MOVE_PORT_MODIFIER):
            return False

        # Do not move from/to handles of connections (only their waypoints)
        if isinstance(item, ConnectionView) and handle in item.end_handles(include_waypoints=True):
            return False

        if handle:
            # Deselect all items unless EXTEND_SELECTION_MODIFIER or RUBBERBAND_MODIFIER is pressed
            # or the item is already selected.
            if not (event.state & (constants.EXTEND_SELECTION_MODIFIER | constants.RUBBERBAND_MODIFIER)
                    or view.hovered_item in view.selected_items):
                del view.selected_items

            view.hovered_item = item
            view.focused_item = item

            self.motion_handle = None

            self.grab_handle(item, handle)

            return True


class ConnectionTool(ConnectHandleTool):

    def __init__(self):
        super(ConnectionTool, self).__init__()
        self._connection_v = None
        self._start_port_v = None
        self._parent_state_v = None
        self._is_transition = False
        self._current_sink = None

    def on_button_release(self, event):
        self._is_transition = False
        self._connection_v = None
        self._start_port_v = None
        self._parent_state_v = None
        self._current_sink = None
        self.grabbed_item = None
        self.grabbed_handle = None

    def _set_motion_handle(self, event):
        """Sets motion handle to currently grabbed handle
        """
        item = self.grabbed_item
        handle = self.grabbed_handle
        pos = event.x, event.y
        self.motion_handle = HandleInMotion(item, handle, self.view)
        self.motion_handle.GLUE_DISTANCE = self._parent_state_v.border_width
        self.motion_handle.start_move(pos)

    def _create_temporary_connection(self):
        """Creates a placeholder connection view

        :return: New placeholder connection
        :rtype: rafcon.mvc.mygaphas.items.connection.ConnectionPlaceholderView
        """
        if self._is_transition:
            self._connection_v = TransitionPlaceholderView(self._parent_state_v.hierarchy_level)
        else:
            self._connection_v = DataFlowPlaceholderView(self._parent_state_v.hierarchy_level)
        self.view.canvas.add(self._connection_v, self._parent_state_v)

    def _handle_temporary_connection(self, old_sink, new_sink, of_target=True):
        """Connect connection to new_sink

        If new_sink is set, the connection origin or target will be set to new_sink. The connection to old_sink is
        being removed.

        :param gaphas.aspect.ConnectionSink old_sink: Old sink (if existing)
        :param gaphas.aspect.ConnectionSink new_sink: New sink (if existing)
        :param bool of_target: Whether the origin or target will be reconnected
        :return:
        """
        def sink_set_and_differs(sink_a, sink_b):
            if not sink_a:
                return False
            if not sink_b:
                return True
            if sink_a.port != sink_b.port:
                return True
            return False

        if sink_set_and_differs(old_sink, new_sink):
            sink_port_v = old_sink.port.port_v
            self._disconnect_temporarily(sink_port_v, target=of_target)

        if sink_set_and_differs(new_sink, old_sink):
            sink_port_v = new_sink.port.port_v
            self._connect_temporarily(sink_port_v, target=of_target)

    def _connect_temporarily(self, port_v, target=True):
        """Set a connection between the current connection and the given port

        :param rafcon.mvc.mygaphas.items.ports.PortView port_v: The port to be connected
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

        :param rafcon.mvc.mygaphas.items.ports.PortView port_v: The port that was connected
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


class ConnectionCreationTool(ConnectionTool):

    def __init__(self):
        super(ConnectionCreationTool, self).__init__()

    def on_button_press(self, event):
        """Handle button press events.

        If the (mouse) button is pressed on top of a Handle (item.Handle), that handle is grabbed and can be
        dragged around.
        """
        if not event.button == 1:  # left mouse button
            return False
        view = self.view

        item, handle = HandleFinder(view.hovered_item, view).get_handle_at_point((event.x, event.y))

        if not handle:  # Require a handle
            return False

        # Connection handle must belong to a port and the MOVE_PORT_MODIFIER must not be pressed
        if not isinstance(item, StateView) or handle not in [port.handle for port in item.get_all_ports()] or (
                    event.state & constants.MOVE_PORT_MODIFIER):
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
                else:
                    return False

        return True

    def on_motion_notify(self, event):
        if not self._parent_state_v or not event.state & gtk.gdk.BUTTON_PRESS_MASK:
            return False

        if not self._connection_v:
            # Create new temporary connection, with origin at the start port and target at the cursor
            self._create_temporary_connection()
            self._start_port_v.parent.connect_connection_to_port(self._connection_v, self._start_port_v, as_target=False)
            self.grab_handle(self._connection_v, self._connection_v.to_handle())
            self._set_motion_handle(event)

        last_sink = self._current_sink
        self._current_sink = self.motion_handle.move((event.x, event.y))

        self._handle_temporary_connection(last_sink, self._current_sink, of_target=True)

    def on_button_release(self, event):
        if not self._connection_v:
            return False

        self.view.canvas.update_now()
        if self._current_sink:
            if self.motion_handle:
                self.motion_handle.stop_move()
            sink_port_v = self._current_sink.port.port_v
            self._disconnect_temporarily(sink_port_v, target=True)
            gap_helper.create_new_connection(self._connection_v.from_port, sink_port_v)

        # remove placeholder from canvas
        if self._connection_v:
            self._connection_v.remove_connection_from_ports()
            self.view.canvas.remove(self._connection_v)

        super(ConnectionCreationTool, self).on_button_release(event)


class ConnectionModificationTool(ConnectionTool):

    def __init__(self):
        super(ConnectionModificationTool, self).__init__()
        self._end_handle = None

    def on_button_press(self, event):
        """Handle button press events.

        If the (mouse) button is pressed on top of a Handle (item.Handle), that handle is grabbed and can be
        dragged around.
        """
        if not event.button == 1:  # left mouse button
            return False
        view = self.view

        item, handle = HandleFinder(view.hovered_item, view).get_handle_at_point((event.x, event.y))

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

    def on_motion_notify(self, event):
        if not self._parent_state_v or not event.state & gtk.gdk.BUTTON_PRESS_MASK:
            return False

        modify_target = self._end_handle is self._connection_v.to_handle()

        if not self.grabbed_handle:
            self.view.canvas.disconnect_item(self._connection_v, self._end_handle)
            self._disconnect_temporarily(self._start_port_v, target=modify_target)
            self.grab_handle(self._connection_v, self._end_handle)
            self._set_motion_handle(event)

        last_sink = self._current_sink
        self._current_sink = self.motion_handle.move((event.x, event.y))

        self._handle_temporary_connection(last_sink, self._current_sink, modify_target)

    def on_button_release(self, event):
        if not self.grabbed_handle:
            return False

        modify_target = self._end_handle is self._connection_v.to_handle()
        self._handle_temporary_connection(self._current_sink, None, of_target=modify_target)

        if not self._current_sink:  # Reset connection to original status, as it was not released above a port
            self._reset_connection()
        else:  # Modify the source/target of the connection
            if self._is_transition:
                try:
                    port = self._current_sink.port.port_v.model.outcome
                    port_id = port.outcome_id
                except AttributeError:  # Port is an income
                    port = None
                    port_id = None
                connection = self._connection_v.model.transition
            else:
                try:
                    port = self._current_sink.port.port_v.model.data_port
                except AttributeError:  # Port is a scoped variable
                    port = self._current_sink.port.port_v.model.scoped_variable
                port_id = port.data_port_id
                connection = self._connection_v.model.data_flow
            port_state_id = port.parent.state_id if port else None

            try:
                if modify_target:
                    connection.modify_target(port_state_id, port_id)
                else:
                    connection.modify_origin(port_state_id, port_id)
            except ValueError as e:
                self._reset_connection()
                logger.error(e)
        self.view.canvas.update_now()
        super(ConnectionModificationTool, self).on_button_release(event)
        self._end_handle = None

    def _reset_connection(self):
        modify_target = self._end_handle is self._connection_v.to_handle()
        self._start_port_v.parent.connect_connection_to_port(self._connection_v, self._start_port_v,
                                                             as_target=modify_target)
        self._redraw_port(self._start_port_v)


class HandleMoveTool(HandleTool):
    def __init__(self, graphical_editor_view, view=None):
        super(HandleMoveTool, self).__init__(view)

        self._graphical_editor_view = graphical_editor_view

        self._child_resize = False

        self._last_active_port = None
        self._new_connection = None

        self._start_state = None
        self._start_width = None
        self._start_height = None

        self._last_hovered_state = None

        self._active_connection_v = None
        self._active_connection_view_handle = None
        self._start_port = None  # Port where connection view pull starts
        self._check_port = None  # Port of connection view that is not pulled

        self._waypoint_list = None

    def on_button_press(self, event):
        try:
            view = self.view
            item, handle = HandleFinder(view.hovered_item, view).get_handle_at_point((event.x, event.y))

            if isinstance(item, ConnectionView):
                # If moved handles item is a connection save all necessary information (where did the handle start,
                # what is the connections other end)
                if handle is item.handles()[1] or handle is item.handles()[len(item.handles()) - 2]:
                    return False
                self._active_connection_v = item
                self._active_connection_view_handle = handle
                if handle is item.from_handle():
                    self._start_port = item.from_port
                    self._check_port = item.to_port
                elif handle is item.to_handle():
                    self._start_port = item.to_port
                    self._check_port = item.from_port

            if isinstance(item, TransitionView):
                self._waypoint_list = item.model.meta['gui']['editor_gaphas']['waypoints']

            # Set start state
            if isinstance(item, StateView):
                self._start_state = item
                self._start_width = item.width
                self._start_height = item.height

        except Exception as e:
            # Keep this except clause for further investigation, to see if it solves the "connection bug" (issue #5)
            logger.error("An unexpected exception occurred while creating a connection: {0} ({1})".format(e,
                                                                                                    type(e).__name__))
        finally:
            # Code copied from HandleTool, preventing the call to get_handle_at_point twice
            if handle:
                # Deselect all items unless CTRL or SHIFT is pressed
                # or the item is already selected.
                if not (event.state & (gtk.gdk.CONTROL_MASK | gtk.gdk.SHIFT_MASK) or view.hovered_item in
                        view.selected_items):
                    del view.selected_items
    
                view.hovered_item = item
                view.focused_item = item
    
                self.motion_handle = None
    
                self.grab_handle(item, handle)
    
                return True

    def on_button_release(self, event):
        try:
            handle = self._active_connection_view_handle
            connection_v = self._active_connection_v
            handle_is_waypoint = connection_v and handle not in connection_v.end_handles()

            # Create new transition if pull beginning at port occurred
            if self._new_connection:
                # drop_item = self._get_drop_item((event.x, event.y))
                gap_helper.create_new_connection(self._new_connection.from_port,
                                                 self._new_connection.to_port)

                # remove placeholder from canvas
                self._new_connection.remove_connection_from_ports()
                self.view.canvas.remove(self._new_connection)

            # if connection has been pulled to another port, update port
            elif self._last_active_port and self._last_active_port is not self._start_port and not handle_is_waypoint:
                if isinstance(connection_v, TransitionView):
                    self._handle_transition_view_change(connection_v, handle)
                elif isinstance(connection_v, DataFlowView):
                    self._handle_data_flow_view_change(connection_v, handle)
            # if connection has been put back to original position or is released on empty space, reset the connection
            elif (not self._last_active_port or self._last_active_port is self._start_port and connection_v) and not \
                    handle_is_waypoint:
                if isinstance(connection_v, TransitionView):
                    self._reset_transition(connection_v, handle, self._start_port.parent)
                elif isinstance(connection_v, DataFlowView):
                    self._reset_data_flow(connection_v, handle, self._start_port.parent)

            # Check, whether a transition waypoint was moved
            if isinstance(connection_v, TransitionView):
                gap_helper.update_meta_data_for_transition_waypoints(self._graphical_editor_view, connection_v,
                                                                     self._waypoint_list)

            if isinstance(self.grabbed_item, NameView):
                gap_helper.update_meta_data_for_name_view(self._graphical_editor_view, self.grabbed_item)

            elif isinstance(self.grabbed_item, StateView):
                only_ports = self.grabbed_handle not in self.grabbed_item.corner_handles
                if only_ports:
                    gap_helper.update_meta_data_for_port(self._graphical_editor_view, self.grabbed_item,
                                                         self.grabbed_handle)
                else:
                    gap_helper.update_meta_data_for_state_view(self._graphical_editor_view, self.grabbed_item,
                                                               self._child_resize)

            # reset temp variables
            self._last_active_port = None
            self._check_port = None
            self._new_connection = None
            self._start_state = None
            self._start_width = None
            self._start_height = None
            self._active_connection_v = None
            self._active_connection_view_handle = None
            self._waypoint_list = None
            self._last_hovered_state = None
            self._child_resize = False

        except Exception as e:
            # Keep this except clause for further investigation, to see if it solves the "connection bug" (issue #5)
            logger.error("An unexpected exception occurred while finalizing a connection: {0} ({1})".format(e,
                                                                                                     type(e).__name__))
        finally:
            super(HandleMoveTool, self).on_button_release(event)

    def on_motion_notify(self, event):
        """Handle motion events

        If a handle is grabbed: drag it around, else, if the pointer is over a handle, make the owning item the
        hovered-item.
        """
        view = self.view
        # If no new transition exists and the grabbed handle is a port handle a new placeholder connection is
        # inserted into canvas
        # This is the default case if one starts to pull from a port handle
        if (not self._new_connection and self.grabbed_handle and event.state & gtk.gdk.BUTTON_PRESS_MASK and
                isinstance(self.grabbed_item, StateView) and not event.state & gtk.gdk.CONTROL_MASK):
            canvas = view.canvas
            # start_state = self.grabbed_item
            start_state = self._start_state
            start_state_parent = canvas.get_parent(start_state)

            handle = self.grabbed_handle
            start_port = gap_helper.get_port_for_handle(handle, start_state)

            # If the start state has a parent continue (ensure no transition is created from top level state)
            if (start_port and (isinstance(start_state_parent, StateView) or
                                    (start_state_parent is None and isinstance(start_port, (IncomeView, InputPortView,
                                                                                            ScopedVariablePortView))))):

                # Go up one hierarchy_level to match the transitions line width
                transition_placeholder = isinstance(start_port, IncomeView) or isinstance(start_port, OutcomeView)
                placeholder_v = ConnectionPlaceholderView(max(start_state.hierarchy_level - 1, 1),
                                                          transition_placeholder)
                self._new_connection = placeholder_v

                canvas.add(placeholder_v, start_state_parent)

                # Check for start_port type and adjust hierarchy_level as well as connect the from handle to the
                # start port of the state
                if isinstance(start_port, IncomeView):
                    placeholder_v.hierarchy_level = start_state.hierarchy_level
                    start_state.connect_to_income(placeholder_v, placeholder_v.from_handle())
                elif isinstance(start_port, OutcomeView):
                    start_state.connect_to_outcome(start_port.outcome_id, placeholder_v, placeholder_v.from_handle())
                elif isinstance(start_port, InputPortView):
                    start_state.connect_to_input_port(start_port.port_id, placeholder_v, placeholder_v.from_handle())
                elif isinstance(start_port, OutputPortView):
                    start_state.connect_to_output_port(start_port.port_id, placeholder_v, placeholder_v.from_handle())
                elif isinstance(start_port, ScopedVariablePortView):
                    start_state.connect_to_scoped_variable_port(start_port.port_id, placeholder_v,
                                                                placeholder_v.from_handle())
                # Ungrab start port handle and grab new transition's to handle to move, also set motion handle
                # to just grabbed handle
                self.ungrab_handle()
                self.grab_handle(placeholder_v, placeholder_v.to_handle())
                self._set_motion_handle(event)

        # the grabbed handle is moved according to mouse movement
        if self.grabbed_handle and event.state & gtk.gdk.BUTTON_PRESS_MASK:
            item = self.grabbed_item
            handle = self.grabbed_handle
            pos = event.x, event.y

            if not self.motion_handle:
                self._set_motion_handle(event)

            snap_distance = self.view.pixel_to_cairo(global_gui_config.get_config_value("PORT_SNAP_DISTANCE", 5))

            # If current handle is from_handle of a connection view
            if isinstance(item, ConnectionView) and item.from_handle() is handle:
                self._get_port_side_size_for_hovered_state(pos)
                self.check_sink_item(self.motion_handle.move(pos, snap_distance), handle, item)
            # If current handle is to_handle of a connection view
            elif isinstance(item, ConnectionView) and item.to_handle() is handle:
                self._get_port_side_size_for_hovered_state(pos)
                self.check_sink_item(self.motion_handle.move(pos, snap_distance), handle, item)
            elif isinstance(item, TransitionView) and handle not in item.end_handles():
                self.motion_handle.move(pos, 0.)
            # If current handle is port or corner of a state view (for ports it only works if CONTROL key is pressed)
            elif isinstance(item, StateView) and handle in item.corner_handles:
                old_size = (item.width, item.height)
                self.motion_handle.move(pos, 0.)
                if event.state & CONTROL_MASK:
                    self._child_resize = True
                    item.resize_all_children(old_size)
                else:
                    item.update_minimum_size_of_children()
            elif isinstance(item, StateView):
                # Move handles only with ctrl modifier clicked
                if event.state & gtk.gdk.CONTROL_MASK:
                    self.motion_handle.move(pos, 0.)
            # All other handles
            else:
                self.motion_handle.move(pos, 5.0)

            return True

    def _get_port_side_size_for_hovered_state(self, pos):
        item_below = self.view.get_item_at_point(pos, False)
        if isinstance(item_below, NameView):
            item_below = self.view.canvas.get_parent(item_below)
        if isinstance(item_below, StateView) and item_below is not self._last_hovered_state:
            self._last_hovered_state = item_below

    def _handle_data_flow_view_change(self, data_flow_v, handle):
        """Handle the change of a data flow origin or target modification

        The method changes the origin or target of an already existing data flow.

        :param data_flow_v: The data flow view that was changed
        :param handle: The handle of the changed port
        """
        start_parent = self._start_port.parent
        last_active_port_parent_state = self._last_active_port.parent.model.state
        modify_target = self._check_port == data_flow_v.from_port
        data_flow = data_flow_v.model.data_flow

        if modify_target:
            to_state_id = last_active_port_parent_state.state_id
            to_port_id = self._last_active_port.port_id

            try:
                data_flow.modify_target(to_state_id, to_port_id)
            except ValueError as e:
                logger.error(e)
                self._reset_data_flow(data_flow_v, handle, start_parent)
        else:
            from_state_id = last_active_port_parent_state.state_id
            from_port_id = self._last_active_port.port_id

            try:
                data_flow.modify_origin(from_state_id, from_port_id)
            except ValueError as e:
                logger.error(e)
                self._reset_data_flow(data_flow_v, handle, start_parent)

    def _handle_transition_view_change(self, transition_v, handle):
        """Handle the change of a transition origin or target modification

        The method changes the origin or target of an already existing transition.

        :param transition_v: The transition view that was changed
        :param handle: The handle of the changed port
        """
        start_parent = self._start_port.parent
        last_active_port_parent_state = self._last_active_port.parent.model.state
        modify_target = self._check_port == transition_v.from_port
        transition = transition_v.model.transition

        if modify_target:
            to_state_id = last_active_port_parent_state.state_id
            if isinstance(self._last_active_port, IncomeView):
                to_outcome_id = None
            else:
                to_outcome_id = self._last_active_port.outcome_id

            try:
                transition.modify_target(to_state_id, to_outcome_id)
            except ValueError as e:
                logger.error(e)
                self._reset_transition(transition_v, handle, start_parent)
        else:
            if isinstance(self._last_active_port, IncomeView):
                from_state_id = None
                from_outcome_id = None
            else:
                from_state_id = last_active_port_parent_state.state_id
                from_outcome_id = self._last_active_port.outcome_id

            try:
                transition.modify_origin(from_state_id, from_outcome_id)
            except ValueError as e:
                logger.error(e)
                self._reset_transition(transition_v, handle, start_parent)

    def _reset_transition(self, transition_v, handle, start_parent):
        """Reset a transition that has been modified

        :param transition_v: The view of the modified transition
        :param handle: The handle of the transition that has been modified
        :param start_parent: The parent state of the modified transition
        """
        if handle not in transition_v.handles():
            return

        self.disconnect_last_active_port(handle, transition_v)
        self.view.canvas.disconnect_item(transition_v, handle)

        if isinstance(self._start_port, OutcomeView):
            start_outcome_id = self._start_port.outcome_id
            start_parent.connect_to_outcome(start_outcome_id, transition_v, handle)
        else:
            start_parent.connect_to_income(transition_v, handle)

        self.view.canvas.update()

    def _reset_data_flow(self, data_flow_v, handle, start_parent):
        """Reset a data flow that has been modified

        :param data_flow_v: The view of the modified data flow
        :param handle: The handle of the data flow that has been modified
        :param start_parent: The parent state of the modified data flow
        """
        if handle not in data_flow_v.handles():
            return

        self.disconnect_last_active_port(handle, data_flow_v)
        self.view.canvas.disconnect_item(data_flow_v, handle)

        if isinstance(self._start_port, InputPortView):
            start_parent.connect_to_input_port(self._start_port.port_id, data_flow_v, handle)
        elif isinstance(self._start_port, OutputPortView):
            start_parent.connect_to_output_port(self._start_port.port_id, data_flow_v, handle)
        elif isinstance(self._start_port, ScopedVariablePortView):
            start_parent.connect_to_scoped_variable_port(self._start_port.port_id, data_flow_v, handle)

        self.view.canvas.update()

    def get_parents_parent_for_port(self, port):
        """Returns the StateView which is the parent of the StateView containing the port.

        If the ports parent is neither of Type StateView nor ScopedVariableView or the parent is the root state,
        None is returned.

        :param port: Port to return parent's parent
        :return: View containing the parent of the port, None if parent is root state or not of type StateView or
          ScopedVariableView
        """
        port_parent = port.parent
        if isinstance(port_parent, StateView):
            return self.view.canvas.get_parent(port_parent)
        else:
            return None

    def is_state_id_root_state(self, state_id):
        for state_v in self.view.canvas.get_root_items():
            if state_v.model.state.state_id == state_id:
                return True
        return False

    def _set_motion_handle(self, event):
        """Sets motion handle to currently grabbed handle
        """
        item = self.grabbed_item
        handle = self.grabbed_handle
        pos = event.x, event.y
        self.motion_handle = HandleInMotion(item, handle, self.view)
        self.motion_handle.start_move(pos)

    def check_sink_item(self, item, handle, connection):
        """Check for sink item

        Checks if the ConnectionSink's item is a StateView and if so tries for every port (income, outcome, input,
        output) to connect the ConnectionSink's port to the corresponding handle.
        If no matching_port was found or the item is no StateView the last active port is disconnected, as no valid
        connection is currently available for the connection.

        :param item: ItemConnectionSink holding the state and port to connect
        :param handle: Handle to connect port to
        :param connection: Connection containing handle
        """
        if isinstance(item, ItemConnectionSink):
            state = item.item
            if isinstance(state, StateView):
                if (isinstance(connection, TransitionView) or
                        (isinstance(connection, ConnectionPlaceholderView) and connection.transition_placeholder)):
                    if self.set_matching_port(state.get_logic_ports(), item.port, handle, connection):
                        return
                elif (isinstance(connection, DataFlowView) or
                          (isinstance(connection,
                                      ConnectionPlaceholderView) and not connection.transition_placeholder)):
                    if self.set_matching_port(state.get_data_ports(), item.port, handle, connection):
                        return
        self.disconnect_last_active_port(handle, connection)

    def disconnect_last_active_port(self, handle, connection):
        """Disconnects the last active port

        Updates the connected handles in the port as well as removes the port from the connected list in the connection.

        :param handle: Handle to disconnect from
        :param connection: ConnectionView to be disconnected, holding the handle
        """

        if self._last_active_port:
            self._last_active_port.remove_connected_handle(handle)
            self._last_active_port.tmp_disconnect()
            connection.reset_port_for_handle(handle)
            self._last_active_port = None

    def set_matching_port(self, port_list, matching_port, handle, connection):
        """Takes a list of PortViews and sets the port matching the matching_port to connected.

        It also updates the ConnectionView's connected port for the given handle and tells the PortView the new
        connected handle.
        If the matching port was found the last active port is disconnected and set to the matching_port

        :param port_list: List of ports to check
        :param matching_port: Port to look for in list
        :param handle: Handle to connect to matching_port
        :param connection: ConnectionView to be connected, holding the handle
        """
        port_for_handle = None

        for port in port_list:
            if port.port is matching_port:
                port_for_handle = port
                break

        if port_for_handle:
            if self._last_active_port is not port_for_handle:
                self.disconnect_last_active_port(handle, connection)
            port_for_handle.add_connected_handle(handle, connection, moving=True)
            port_for_handle.tmp_connect(handle, connection)
            connection.set_port_for_handle(port_for_handle, handle)
            self._last_active_port = port_for_handle
            # Redraw state of port to make hover state visible
            self.view.queue_draw_area(*port_for_handle.get_port_area(self.view))
            return True

        return False


class ConnectHandleMoveTool(HandleMoveTool):
    """Tool for connecting two items.

    There are two items involved. Handle of connecting item (usually
    a line) is being dragged by an user towards another item (item in
    short). Port of an item is found by the tool and connection is
    established by creating a constraint between line's handle and item's
    port.
    """

    def glue(self, item, handle, vpos):
        """Perform a small glue action to ensure the handle is at a proper location for connecting.
        """

        # glue_distance is the snapping radius
        if item.from_handle() is handle:
            glue_distance = 1.0 / pow(2, item.hierarchy_level)
        else:
            glue_distance = 1.0 / pow(2, item.hierarchy_level - 1)

        if self.motion_handle:
            return self.motion_handle.glue(vpos, glue_distance)
        else:
            return HandleInMotion(item, handle, self.view).glue(vpos, glue_distance)

    def connect(self, item, handle, vpos):
        """Connect a handle of a item to connectable item.

        Connectable item is found by `ConnectHandleTool.glue` method.

        :Parameters:
         item
            Connecting item.
         handle
            Handle of connecting item.
         vpos
            Position to connect to (or near at least)
        """
        connector = Connector(item, handle)

        # find connectable item and its port
        sink = self.glue(item, handle, vpos)

        # no new connectable item, then disconnect and exit
        if sink:
            connector.connect(sink)
        else:
            cinfo = item.canvas.get_connection(handle)
            if cinfo:
                connector.disconnect()

    def on_button_release(self, event):
        item = self.grabbed_item
        handle = self.grabbed_handle
        try:
            if handle and handle.connectable:
                self.connect(item, handle, (event.x, event.y))
        finally:
            return super(ConnectHandleMoveTool, self).on_button_release(event)


class RightClickTool(ItemTool):

    def __init__(self, view=None, buttons=(3,)):
        super(RightClickTool, self).__init__(view, buttons)
        self.sm_right_click_menu = StateRightClickMenuGaphas()

    def on_button_press(self, event):
        self.sm_right_click_menu.mouse_click(None, event)
