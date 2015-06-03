from gaphas.tool import Tool
from gaphas.aspect import HandleInMotion, Connector, HandleFinder, HandleSelection, ConnectionSink, ItemConnectionSink, Finder
from simplegeneric import generic

from awesome_tool.mvc.views.gap.connection import ConnectionView, ConnectionPlaceholderView, TransitionView
from awesome_tool.mvc.views.gap.ports import IncomeView, OutcomeView
from awesome_tool.mvc.views.gap.state import StateView

import gtk

from awesome_tool.mvc.statemachine_helper import StateMachineHelper

from awesome_tool.utils import log
logger = log.get_logger(__name__)


class MyDeleteTool(Tool):

    def on_key_release(self, event):
        if gtk.gdk.keyval_name(event.keyval) == "Delete":
            if isinstance(self.view.focused_item, TransitionView):
                StateMachineHelper.delete_model(self.view.focused_item.transition_m)
                self.view.focused_item.remove_connection_from_ports()
                self.view.canvas.remove(self.view.focused_item)


class MyHoverTool(Tool):

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


class MyItemHandleInMotion(object):
    """
    Move a handle (role is applied to the handle)
    """

    GLUE_DISTANCE = 5.0

    def __init__(self, item, handle, view):
        self.item = item
        self.handle = handle
        self.view = view
        self.last_x, self.last_y = None, None
        self._start_port = None
        self._current_port = None

    def start_move(self, pos):
        self.last_x, self.last_y = pos
        canvas = self.item.canvas

        cinfo = canvas.get_connection(self.handle)
        if cinfo:
            canvas.solver.remove_constraint(cinfo.constraint)

    def move(self, pos, distance):
        item = self.item
        handle = self.handle
        view = self.view

        v2i = view.get_matrix_v2i(item)

        x, y = v2i.transform_point(*pos)

        self.handle.pos = (x, y)

        sink = self.glue(pos, distance)

        # do not request matrix update as matrix recalculation will be
        # performed due to item normalization if required
        item.request_update(matrix=False)

        return sink

    def stop_move(self):
        pass

    def glue(self, pos, distance=GLUE_DISTANCE):
        """
        Glue to an item near a specific point.

        Returns a ConnectionSink or None.
        """
        item = self.item
        handle = self.handle
        view = self.view

        if not handle.connectable:
            return None

        connectable, port, glue_pos = \
                view.get_port_at_point(pos, distance=distance, exclude=(item,))

        if not self._start_port:
            self._start_port = port

        # check if item and found item can be connected on closest port
        if port is not None:
            assert connectable is not None

            connector = Connector(self.item, self.handle)
            sink = ConnectionSink(connectable, port)

            if connector.allow(sink):
                # transform coordinates from view space to the item space and
                # update position of item's handle
                v2i = view.get_matrix_v2i(item).transform_point
                handle.pos = v2i(*glue_pos)
                self._current_port = port
                return sink
        return None

    def get_connected_ports_list(self, state):
        already_connected_ports = []
        ports_list = [[state.income, ], state.outcomes, state.inputs, state.outputs]
        for ports in ports_list:
            for port in ports:
                if port.connected and port.port is not self._start_port and port.port is not self._current_port:
                    already_connected_ports.append(port.port)
        return already_connected_ports


MyHandleInMotion = generic(MyItemHandleInMotion)


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

        if self._new_transition:
            self._create_new_transition()

            # remove placeholder from canvas
            self._new_transition.remove_connection_from_ports()
            self.view.canvas.remove(self._new_transition)

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

        if (not self._new_transition and self.grabbed_handle and event.state & gtk.gdk.BUTTON_PRESS_MASK and
                isinstance(self.grabbed_item, StateView) and not event.state & gtk.gdk.CONTROL_MASK):
            canvas = view.canvas
            start_state = self.grabbed_item
            start_state_parent = canvas.get_parent(start_state)

            if start_state_parent is not None:
                assert isinstance(start_state_parent, StateView)
                handle = self.grabbed_handle
                start_port = self.get_port_for_handle(handle, start_state)
                if start_port:
                    transition_v = ConnectionPlaceholderView(start_state.hierarchy_level - 1)
                    self._new_transition = transition_v

                    canvas.add(transition_v, start_state_parent)

                    if isinstance(start_port, IncomeView):
                        transition_v.hierarchy_level = start_state.hierarchy_level
                        start_state.connect_to_income(transition_v, transition_v.from_handle())
                    elif isinstance(start_port, OutcomeView):
                        start_state.connect_to_outcome(start_port.outcome_id, transition_v, transition_v.from_handle())
                    self.ungrab_handle()
                    self.grab_handle(transition_v, transition_v.to_handle())

        if self.grabbed_handle and event.state & gtk.gdk.BUTTON_PRESS_MASK and event.state & gtk.gdk.CONTROL_MASK:
            item = self.grabbed_item
            handle = self.grabbed_handle
            pos = event.x, event.y

            if not self.motion_handle:
                self.motion_handle = MyHandleInMotion(item, handle, self.view)
                self.motion_handle.start_move(pos)
            if isinstance(item, StateView):
                self.motion_handle.move(pos, 0.)
        elif self.grabbed_handle and event.state & gtk.gdk.BUTTON_PRESS_MASK:  # and event.state & gtk.gdk.CONTROL_MASK:
            item = self.grabbed_item
            handle = self.grabbed_handle
            pos = event.x, event.y

            if not self.motion_handle:
                self.motion_handle = MyHandleInMotion(item, handle, self.view)
                self.motion_handle.start_move(pos)
            if isinstance(item, ConnectionView) and item.from_handle() is handle:
                self.check_sink_item(self.motion_handle.move(pos, 5.0 / ((item.hierarchy_level + 1) * 2)), handle, item)
            elif isinstance(item, ConnectionView) and item.to_handle() is handle:
                self.check_sink_item(self.motion_handle.move(pos, 5.0 / (item.hierarchy_level * 2)), handle, item)
            else:
                self.motion_handle.move(pos, 5.0)

            return True

    def _create_new_transition(self):
        nt_from_port = self._new_transition.from_port
        nt_to_port = self._new_transition.to_port

        if (nt_from_port.parent is nt_to_port.parent and isinstance(nt_from_port, OutcomeView) and
                isinstance(nt_to_port, OutcomeView)):
            logger.warn("Cannot connect to outcomes of the same state")
            return 

        if isinstance(nt_from_port, OutcomeView) and nt_from_port and nt_to_port and nt_from_port is not nt_to_port:
            canvas = self.view.canvas
            to_state_v = nt_to_port.parent

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

                    transition_v = TransitionView(transition_m, self._start_state.hierarchy_level)
                    canvas.add(transition_v, canvas.get_parent(self._start_state))

                    self._start_state.connect_to_outcome(nt_from_port.outcome_id, transition_v, transition_v.from_handle())
                    if isinstance(nt_to_port, IncomeView):
                        to_state_v.connect_to_income(transition_v, transition_v.to_handle())
                    elif isinstance(nt_to_port, OutcomeView):
                        to_state_v.connect_to_outcome(nt_to_port.outcome_id, transition_v, transition_v.to_handle())
                except AttributeError as e:
                    logger.warn("Transition couldn't be added: {0}".format(e))
                except Exception as e:
                    logger.error("Unexpected exception while creating transition: {0}".format(e))

    @staticmethod
    def get_port_for_handle(handle, state):
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
        if self._last_active_port:
            self._last_active_port.remove_connected_handle(handle)
            connection.reset_port_for_handle(handle)
            self._last_active_port = None

    def set_matching_port(self, port_list, matching_port, handle, connection):
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