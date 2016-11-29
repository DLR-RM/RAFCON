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
                    self.view.graphical_editor.emit('remove_state_from_state_machine')
                    return True


class MoveItemTool(ItemTool):
    """This class is responsible of moving states, names, connections, etc.
    """

    def __init__(self, view=None, buttons=(1,)):
        super(MoveItemTool, self).__init__(view, buttons)
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

        if event.state & constants.RUBBERBAND_MODIFIER:
            return False  # Mouse clicks with pressed shift key are handled in another tool

        self._item = self.get_item()

        # NameView can only be moved when the Ctrl-key is pressed
        self._move_name_v = isinstance(self._item, NameView) and event.state & gtk.gdk.CONTROL_MASK
        if self._item in self.view.selected_items and \
                isinstance(self._item, NameView) and event.state & gtk.gdk.CONTROL_MASK:
            # self.view.unselect_item(self._item)
            pass
        else:
            if not event.state & constants.EXTEND_SELECTION_MODIFIER and self._item not in self.view.selected_items:
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
                self.view.graphical_editor.emit('meta_data_changed', self._item.model, "position", True)

        if isinstance(self.view.focused_item, NameView):
            if position_changed:
                self.view.graphical_editor.emit('meta_data_changed', self.view.focused_item.parent.model,
                                                "name_position", False)

        if not position_changed:
            if self._item in self.view.selected_items and event.state & constants.EXTEND_SELECTION_MODIFIER:
                if self._do_not_unselect is not self._item:
                    self.view.unselect_item(self._item)
            else:
                if not event.state & constants.EXTEND_SELECTION_MODIFIER:
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
            distance = view.hovered_item.border_width / 2.
            state_v, hovered_handle = HandleFinder(view.hovered_item, view).get_handle_at_point(pos, distance)

            # Hover over port => show hover state of port and different cursor
            if hovered_handle and hovered_handle not in state_v.corner_handles:
                view.hovered_handle = hovered_handle
                port_v = state_v.get_port_for_handle(hovered_handle)
                view.queue_draw_area(*port_v.get_port_area(view))
                if event.state & constants.MOVE_PORT_MODIFIER:
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
    def on_button_press(self, event):
        # print "on_button_press: ", self.__class__.__name__
        if event.state & constants.RUBBERBAND_MODIFIER:
            return super(MultiSelectionTool, self).on_button_press(event)
        return False

    def on_motion_notify(self, event):
        if event.state & gtk.gdk.BUTTON_PRESS_MASK and event.state & constants.RUBBERBAND_MODIFIER:
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
        if event.state & constants.EXTEND_SELECTION_MODIFIER:
            old_items_selected = []
        else:
            old_items_selected = list(self.view.selected_items)
        for item in old_items_selected:
            if item in self.view.selected_items:
                self.view.unselect_item(item)
        self.view.select_in_rectangle((min(x0, x1), min(y0, y1), abs(x1 - x0), abs(y1 - y0)))

        old_items_in_new_selection = [item in self.view.selected_items for item in old_items_selected]
        current_items_which_are_old_selection = [item in old_items_selected for item in self.view.selected_items]
        rubber_band_selection = list(self.view.selected_items)
        new_selection = old_items_selected
        if any(old_items_in_new_selection) and not all(current_items_which_are_old_selection):  # reselect elements
            # add new  rubber band selection by preserving old state selection
            for item in rubber_band_selection:
                if item not in old_items_selected:
                    old_items_selected.append(item)
        else:
            if not any(current_items_which_are_old_selection):
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

        if isinstance(view.hovered_item, StateView):
            distance = view.hovered_item.border_width / 2.
            item, handle = HandleFinder(view.hovered_item, view).get_handle_at_point((event.x, event.y), distance)
        else:
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

    def on_motion_notify(self, event):
        if not self.grabbed_handle or not event.state & gtk.gdk.BUTTON_PRESS_MASK:
            return
        item = self.grabbed_item
        resize_recursive = isinstance(item, StateView) and self.grabbed_handle in item.corner_handles and \
                           event.state & constants.RECURSIVE_RESIZE_MODIFIER

        if resize_recursive:
            old_size = (item.width, item.height)

        super(MoveHandleTool, self).on_motion_notify(event)

        if resize_recursive:
            item.resize_all_children(old_size)
        item.update_minimum_size_of_children()

        return True

    def on_button_release(self, event):
        if self.grabbed_item:
            item = self.grabbed_item
            graphical_editor = self.view.graphical_editor
            if isinstance(item, NameView):
                gap_helper.update_meta_data_for_name_view(graphical_editor, item, publish=True)
            elif isinstance(item, ConnectionView):
                gap_helper.update_meta_data_for_transition_waypoints(graphical_editor, item, None)
            else:  # StateView
                if self.grabbed_handle in [port.handle for port in item.get_all_ports()]:
                    gap_helper.update_meta_data_for_port(graphical_editor, item, self.grabbed_handle)
                else:
                    gap_helper.update_meta_data_for_state_view(graphical_editor, item, affects_children=True,
                                                               publish=True)
        super(MoveHandleTool, self).on_button_release(event)


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
            self._start_port_v.parent.connect_connection_to_port(self._connection_v, self._start_port_v,
                                                                 as_target=False)
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
        super(ConnectionModificationTool, self).on_button_release(event)
        self._end_handle = None

    def _reset_connection(self):
        modify_target = self._end_handle is self._connection_v.to_handle()
        self._start_port_v.parent.connect_connection_to_port(self._connection_v, self._start_port_v,
                                                             as_target=modify_target)
        self._redraw_port(self._start_port_v)


class RightClickTool(ItemTool):
    def __init__(self, view=None, buttons=(3,)):
        super(RightClickTool, self).__init__(view, buttons)
        self.sm_right_click_menu = StateRightClickMenuGaphas()

    def on_button_press(self, event):
        self.sm_right_click_menu.mouse_click(None, event)
