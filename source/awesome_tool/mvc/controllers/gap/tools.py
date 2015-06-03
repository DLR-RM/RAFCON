from gaphas.tool import Tool
from gaphas.aspect import HandleInMotion, Connector, HandleFinder, HandleSelection, ItemConnectionSink, Finder

from awesome_tool.mvc.views.gap.connection import ConnectionView, ConnectionPlaceholderView, TransitionView
from awesome_tool.mvc.views.gap.ports import IncomeView, OutcomeView
from awesome_tool.mvc.views.gap.state import StateView

from gaphas.aspect import InMotion, Selection
from awesome_tool.mvc.controllers.gap.aspect import MyHandleInMotion

import gtk

from awesome_tool.mvc.statemachine_helper import StateMachineHelper

from awesome_tool.utils import log
logger = log.get_logger(__name__)


class MyDeleteTool(Tool):
    """
    This tool is responsible of deleting the selected item
    """

    def on_key_release(self, event):
        if gtk.gdk.keyval_name(event.keyval) == "Delete":
            # Delete Transition from state machine
            if isinstance(self.view.focused_item, TransitionView):
                StateMachineHelper.delete_model(self.view.focused_item.transition_m)
                self.view.focused_item.remove_connection_from_ports()
                self.view.canvas.remove(self.view.focused_item)


class MyItemTool(Tool):
    """
    ItemTool does selection and dragging of items. On a button click,
    the currently "hovered item" is selected. If CTRL or SHIFT are pressed,
    already selected items remain selected. The last selected item gets the
    focus (e.g. receives key press events).

    The roles used are Selection (select, unselect) and InMotion (move).
    """

    def __init__(self, graphical_editor_view, view=None, buttons=(1,)):
        super(MyItemTool, self).__init__(view)
        self._buttons = buttons
        self._movable_items = set()
        self._graphical_editor_view = graphical_editor_view

    def get_item(self):
        return self.view.hovered_item

    def movable_items(self):
        """
        Filter the items that should eventually be moved.

        Returns InMotion aspects for the items.
        """
        view = self.view
        get_ancestors = view.canvas.get_ancestors
        selected_items = set(view.selected_items)
        for item in selected_items:
            # Do not move subitems of selected items
            if not set(get_ancestors(item)).intersection(selected_items):
                yield InMotion(item, view)

    def on_button_press(self, event):
        view = self.view
        item = self.get_item()

        if event.button not in self._buttons:
            return False

        # Deselect all items unless CTRL or SHIFT is pressed
        # or the item is already selected.
        if not (event.state & (gtk.gdk.CONTROL_MASK | gtk.gdk.SHIFT_MASK)
                or item in view.selected_items):
            del view.selected_items

        if item:
            if view.hovered_item in view.selected_items and \
                    event.state & gtk.gdk.CONTROL_MASK:
                selection = Selection(item, view)
                selection.unselect()
            else:
                selection = Selection(item, view)
                selection.select()
                self._movable_items.clear()

            self._graphical_editor_view.emit('new_state_selection', self.view.focused_item)
            return True

    def on_button_release(self, event):
        if event.button not in self._buttons:
            return False
        for inmotion in self._movable_items:
            inmotion.stop_move()
        self._movable_items.clear()
        return True

    def on_motion_notify(self, event):
        """
        Normally do nothing.
        If a button is pressed move the items around.
        """
        if event.state & gtk.gdk.BUTTON_PRESS_MASK:

            if not self._movable_items:
                self._movable_items = set(self.movable_items())
                for inmotion in self._movable_items:
                    inmotion.start_move((event.x, event.y))

            for inmotion in self._movable_items:
                inmotion.move((event.x, event.y))

            return True


class MyHoverTool(Tool):
    """
    This tool keeps track of the currently hovered item and sets the "hovered" flag if the item is a State
    """

    def __init__(self, view=None):
        super(MyHoverTool, self).__init__(view)
        self._prev_hovered_item = None

    def on_motion_notify(self, event):
        view = self.view
        pos = event.x, event.y
        view.hovered_item = Finder(view).get_item_at_point(pos)

        if self._prev_hovered_item and view.hovered_item is not self._prev_hovered_item:
            self._prev_hovered_item.hovered = False
        if isinstance(view.hovered_item, StateView):
            view.hovered_item.hovered = True
            self._prev_hovered_item = view.hovered_item

# ------------------------------------------------------------------
# -----------------------------SNAPPING-----------------------------
# ------------------------------------------------------------------


class MyHandleTool(Tool):
    """
    Tool for moving handles around.

    By default this tool does not provide connecting handles to another item
    (see `ConnectHandleTool`).
    """

    def __init__(self, view=None):
        super(MyHandleTool, self).__init__(view)
        self.grabbed_handle = None
        self.grabbed_item = None
        self.motion_handle = None
        self._last_active_port = None
        self._new_transition = None
        self._start_state = None

    def grab_handle(self, item, handle):
        """
        Grab a specific handle. This can be used from the PlacementTool
        to set the state of the handle tool.
        """
        assert item is None and handle is None or handle in item.handles()
        self.grabbed_item = item
        self.grabbed_handle = handle

        selection = HandleSelection(item, handle, self.view)
        selection.select()

    def ungrab_handle(self):
        """
        Reset grabbed_handle and grabbed_item.
        """
        item = self.grabbed_item
        handle = self.grabbed_handle
        self.grabbed_handle = None
        self.grabbed_item = None
        if handle:
            selection = HandleSelection(item, handle, self.view)
            selection.unselect()

    def on_button_press(self, event):
        """
        Handle button press events. If the (mouse) button is pressed on
        top of a Handle (item.Handle), that handle is grabbed and can be
        dragged around.
        """
        view = self.view

        item, handle = HandleFinder(view.hovered_item, view).get_handle_at_point((event.x, event.y))
        # Set start state
        if isinstance(item, StateView):
            self._start_state = item

        if handle:
            # Deselect all items unless CTRL or SHIFT is pressed
            # or the item is already selected.

            if not (event.state & (gtk.gdk.CONTROL_MASK | gtk.gdk.SHIFT_MASK)
                    or view.hovered_item in view.selected_items):
                del view.selected_items

            view.hovered_item = item
            view.focused_item = item

            self.motion_handle = None

            self.grab_handle(item, handle)

            return True

    def on_button_release(self, event):
        """
        Release a grabbed handle.
        """
        # queue extra redraw to make sure the item is drawn properly
        grabbed_handle, grabbed_item = self.grabbed_handle, self.grabbed_item

        # Create new transition if pull beginning at port occurred
        if self._new_transition:
            self._create_new_transition()

            # remove placeholder from canvas
            self._new_transition.remove_connection_from_ports()
            self.view.canvas.remove(self._new_transition)

        # reset temp variables
        self._last_active_port = None
        self._new_transition = None
        self._start_state = None

        if self.motion_handle:
            self.motion_handle.stop_move()
            self.motion_handle = None

        self.ungrab_handle()

        if grabbed_handle:
            grabbed_item.request_update()
        return True

    def on_motion_notify(self, event):
        """
        Handle motion events. If a handle is grabbed: drag it around,
        else, if the pointer is over a handle, make the owning item the
        hovered-item.
        """
        view = self.view

        # If no new transition exists and the grabbed handle is a port handle a new placeholder connection is
        # inserted into canvas
        # This is the default case if one starts to pull from a port handle
        if (not self._new_transition and self.grabbed_handle and event.state & gtk.gdk.BUTTON_PRESS_MASK and
                isinstance(self.grabbed_item, StateView) and not event.state & gtk.gdk.CONTROL_MASK):
            canvas = view.canvas
            # start_state = self.grabbed_item
            start_state = self._start_state
            start_state_parent = canvas.get_parent(start_state)

            # If the start state has a parent continue (ensure no transition is created from top level state)
            if start_state_parent is not None:
                assert isinstance(start_state_parent, StateView)
                handle = self.grabbed_handle
                start_port = self.get_port_for_handle(handle, start_state)
                if start_port:
                    # Go up one hierarchy_level to match the transitions line width
                    transition_v = ConnectionPlaceholderView(start_state.hierarchy_level - 1)
                    self._new_transition = transition_v

                    canvas.add(transition_v, start_state_parent)

                    # Check for start_port type and adjust hierarchy_level as well as connect the from handle to the
                    # start port of the state
                    if isinstance(start_port, IncomeView):
                        transition_v.hierarchy_level = start_state.hierarchy_level
                        start_state.connect_to_income(transition_v, transition_v.from_handle())
                    elif isinstance(start_port, OutcomeView):
                        start_state.connect_to_outcome(start_port.outcome_id, transition_v, transition_v.from_handle())
                    # Ungrab start port handle and grab new transition's to handle to move, also set motion handle
                    # to just grabbed handle
                    self.ungrab_handle()
                    self.grab_handle(transition_v, transition_v.to_handle())
                    self._set_motion_handle(event)

        # the grabbed handle is moved according to mouse movement
        if self.grabbed_handle and event.state & gtk.gdk.BUTTON_PRESS_MASK:
            item = self.grabbed_item
            handle = self.grabbed_handle
            pos = event.x, event.y

            if not self.motion_handle:
                self._set_motion_handle(event)

            # If current handle is from_handle of a connection view
            if isinstance(item, ConnectionView) and item.from_handle() is handle:
                self.check_sink_item(self.motion_handle.move(pos, 5.0 / ((item.hierarchy_level + 1) * 2)), handle, item)
            # If current handle is to_handle of a connection view
            elif isinstance(item, ConnectionView) and item.to_handle() is handle:
                self.check_sink_item(self.motion_handle.move(pos, 5.0 / (item.hierarchy_level * 2)), handle, item)
            # If current handle is port or corner of a state view (for ports it only works if CONTROL key is pressed)
            elif isinstance(item, StateView):
                self.motion_handle.move(pos, 0.)
            # All other handles
            else:
                self.motion_handle.move(pos, 5.0)

            return True

    def _set_motion_handle(self, event):
        """
        Sets motion handle to currently grabbed handle
        """
        item = self.grabbed_item
        handle = self.grabbed_handle
        pos = event.x, event.y
        self.motion_handle = MyHandleInMotion(item, handle, self.view)
        self.motion_handle.start_move(pos)

    def _create_new_transition(self):
        """
        Creates a new transition and adds this transition to the state machine model.
        Only transitions from outcomes are added (transitions from incomes are added via "start_state" checkbox
        """
        nt_from_port = self._new_transition.from_port
        nt_to_port = self._new_transition.to_port

        # Ensure from_port is an outcome and
        # from_port as well as to_port are connected to transition and
        # ports are not the same port
        if isinstance(nt_from_port, OutcomeView) and nt_from_port and nt_to_port and nt_from_port is not nt_to_port:
            # Make sure the ports are not connected to an outcome of the same state
            if (nt_from_port.parent is nt_to_port.parent and isinstance(nt_from_port, OutcomeView) and
                    isinstance(nt_to_port, OutcomeView)):
                logger.warn("Cannot connect to outcomes of the same state")
                return

            canvas = self.view.canvas
            to_state_v = nt_to_port.parent

            # Gather necessary information to create transition
            from_state_id = self._start_state.state_m.state.state_id
            from_outcome_id = nt_from_port.outcome_id
            to_state_id = None
            to_outcome_id = None

            to_state_m = to_state_v.state_m
            responsible_parent_m = None
            if isinstance(nt_to_port, IncomeView):
                to_state_id = to_state_m.state.state_id
                responsible_parent_m = to_state_m.parent
            elif isinstance(nt_to_port, OutcomeView):
                to_outcome_id = nt_to_port.outcome_id
                responsible_parent_m = to_state_m

            if responsible_parent_m:
                try:
                    transition_id = responsible_parent_m.state.add_transition(from_state_id,
                                                                              from_outcome_id,
                                                                              to_state_id,
                                                                              to_outcome_id)
                    transition_m = StateMachineHelper.get_transition_model(responsible_parent_m, transition_id)

                    # Create actual transition view to replace placeholder view
                    transition_v = TransitionView(transition_m, self._start_state.hierarchy_level - 1)
                    canvas.add(transition_v, canvas.get_parent(self._start_state))

                    # Connect new transition view to ports of placeholder
                    self._start_state.connect_to_outcome(nt_from_port.outcome_id, transition_v, transition_v.from_handle())
                    if isinstance(nt_to_port, IncomeView):
                        transition_v.hierarchy_level = self._start_state.hierarchy_level
                        to_state_v.connect_to_income(transition_v, transition_v.to_handle())
                    elif isinstance(nt_to_port, OutcomeView):
                        to_state_v.connect_to_outcome(nt_to_port.outcome_id, transition_v, transition_v.to_handle())
                except AttributeError as e:
                    logger.warn("Transition couldn't be added: {0}".format(e))
                except Exception as e:
                    logger.error("Unexpected exception while creating transition: {0}".format(e))

    @staticmethod
    def get_port_for_handle(handle, state):
        """
        Looks for and returns the PortView to the given handle in the provided state
        :param handle: Handle to look for port
        :param state: State containing handle and port
        :returns: PortView for handle
        """
        if state.income.handle == handle:
            return state.income
        else:
            for outcome in state.outcomes:
                if outcome.handle == handle:
                    return outcome
            for input in state.inputs:
                if input.handle == handle:
                    return input
            for output in state.outputs:
                if output.handle == handle:
                    return output

    def check_sink_item(self, item, handle, connection):
        """
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
                if self.set_matching_port([state.income, ], item.port, handle, connection):
                    return
                elif self.set_matching_port(state.outcomes, item.port, handle, connection):
                    return
                elif self.set_matching_port(state.outputs, item.port, handle, connection):
                    return
                elif self.set_matching_port(state.inputs, item.port, handle, connection):
                    return
        self.disconnect_last_active_port(handle, connection)

    def disconnect_last_active_port(self, handle, connection):
        """
        Disconnects the last active port and updates the connected handles in the port as well as removes the port
        from the connected list in the connection.
        :param handle: Handle to disconnect from
        :param connection: ConnectionView to be disconnected, holding the handle
        """
        if self._last_active_port:
            self._last_active_port.remove_connected_handle(handle)
            connection.reset_port_for_handle(handle)
            self._last_active_port = None

    def set_matching_port(self, port_list, matching_port, handle, connection):
        """
        Takes a list of PortViews and sets the port matching the matching_port to connected.
        It also updates the ConnectionView's connected port for the given handle and tells the PortView the new
        connected handle.
        If the matching port was found the last active port is disconnected and set to the matching_port
        :param port_list: List of ports to check
        :param matching_port: Port to look for in list
        :param handle: Handle to connect to matching_port
        :param connection: ConnectionView to be connected, holding the handle
        """
        for port in port_list:
            if port.port is matching_port:
                if self._last_active_port is not port:
                    self.disconnect_last_active_port(handle, connection)
                port.add_connected_handle(handle)
                connection.set_port_for_handle(port, handle)
                self._last_active_port = port
                return True
        return False


class MyConnectHandleTool(MyHandleTool):
    """
    Tool for connecting two items.

    There are two items involved. Handle of connecting item (usually
    a line) is being dragged by an user towards another item (item in
    short). Port of an item is found by the tool and connection is
    established by creating a constraint between line's handle and item's
    port.
    """

    def glue(self, item, handle, vpos):
        """
        Perform a small glue action to ensure the handle is at a proper
        location for connecting.
        """

        if item.from_handle() is handle:
            glue_distance = 5.0 / ((item.hierarchy_level + 1) * 2)
        else:
            glue_distance = 5.0 / (item.hierarchy_level * 2)

        if self.motion_handle:
            return self.motion_handle.glue(vpos, glue_distance)
        else:
            return HandleInMotion(item, handle, self.view).glue(vpos, glue_distance)

    def connect(self, item, handle, vpos):
        """
        Connect a handle of a item to connectable item.

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

        # no new connectable item, then diconnect and exit
        if sink:
            connector.connect(sink)
        else:
            cinfo = item.canvas.get_connection(handle)
            if cinfo:
                connector.disconnect()

    def on_button_release(self, event):
        view = self.view
        item = self.grabbed_item
        handle = self.grabbed_handle
        try:
            if handle and handle.connectable:
                self.connect(item, handle, (event.x, event.y))
        finally:
            return super(MyConnectHandleTool, self).on_button_release(event)