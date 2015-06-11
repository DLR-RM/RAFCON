from gaphas.tool import Tool, ItemTool, HoverTool, HandleTool, RubberbandTool
from gaphas.aspect import Connector, HandleFinder, ItemConnectionSink

from awesome_tool.mvc.views.gap.connection import ConnectionView, ConnectionPlaceholderView, TransitionView, DataFlowView
from awesome_tool.mvc.views.gap.ports import IncomeView, OutcomeView, InputPortView, OutputPortView, \
    ScopedDataInputPortView, ScopedDataOutputPortView
from awesome_tool.mvc.views.gap.state import StateView
from awesome_tool.mvc.views.gap.scope import ScopedVariableView

from awesome_tool.mvc.controllers.gap.aspect import MyHandleInMotion

from awesome_tool.statemachine.states.container_state import ContainerState

import gtk
from enum import Enum

from awesome_tool.mvc.statemachine_helper import StateMachineHelper

from awesome_tool.utils import log
logger = log.get_logger(__name__)


PortMoved = Enum('PORT', 'FROM TO')


class MyDeleteTool(Tool):
    """
    This tool is responsible of deleting the selected item
    """

    def __init__(self, graphical_editor_view, view=None):
        super(MyDeleteTool, self).__init__(view)
        self._graphical_editor_view = graphical_editor_view

    def on_key_release(self, event):
        if gtk.gdk.keyval_name(event.keyval) == "Delete":
            # Delete Transition from state machine
            if isinstance(self.view.focused_item, TransitionView):
                StateMachineHelper.delete_model(self.view.focused_item.transition_m)
                return True
            if isinstance(self.view.focused_item, DataFlowView):
                StateMachineHelper.delete_model(self.view.focused_item.data_flow_m)
                return True
            if isinstance(self.view.focused_item, StateView):
                if self.view.has_focus():
                    self._graphical_editor_view.emit('remove_state_from_state_machine')
                    self.view.focused_item.remove_keep_rect_within_constraint_from_parent()
                    self.view.canvas.remove(self.view.focused_item)
                    return True
            if isinstance(self.view.focused_item, ScopedVariableView):
                if self.view.has_focus():
                    self._graphical_editor_view.emit('remove_scoped_variable_from_state', self.view.focused_item)
                    self.view.focused_item.remove_keep_rect_within_constraint_from_parent()
                    self.view.canvas.remove(self.view.focused_item)
                    return True


class MyItemTool(ItemTool):

    def __init__(self, graphical_editor_view, view=None, buttons=(1,)):
        super(MyItemTool, self).__init__(view, buttons)
        self._graphical_editor_view = graphical_editor_view

    def on_button_press(self, event):
        super(MyItemTool, self).on_button_press(event)

        if not self.view.is_focus():
            self.view.grab_focus()
        self._graphical_editor_view.emit('new_state_selection', self.view.focused_item)


class MyHoverTool(HoverTool):

    def __init__(self, view=None):
        super(MyHoverTool, self).__init__(view)
        self._prev_hovered_item = None

    def on_motion_notify(self, event):
        super(MyHoverTool, self).on_motion_notify(event)

        if self._prev_hovered_item and self.view.hovered_item is not self._prev_hovered_item:
            self._prev_hovered_item.hovered = False
        if isinstance(self.view.hovered_item, StateView):
            self.view.hovered_item.hovered = True
            self._prev_hovered_item = self.view.hovered_item


class MyRubberbandTool(RubberbandTool):

    def on_button_press(self, event):
        if event.state & gtk.gdk.SHIFT_MASK:
            return super(MyRubberbandTool, self).on_button_press(event)
        return False

    def on_motion_notify(self, event):
        if event.state & gtk.gdk.BUTTON_PRESS_MASK and event.state & gtk.gdk.SHIFT_MASK:
            view = self.view
            self.queue_draw(view)
            self.x1, self.y1 = event.x, event.y
            self.queue_draw(view)
            return True

# ------------------------------------------------------------------
# -----------------------------SNAPPING-----------------------------
# ------------------------------------------------------------------


class MyHandleTool(HandleTool):

    def __init__(self, view=None):
        super(MyHandleTool, self).__init__(view)

        self._last_active_port = None
        self._new_transition = None
        self._start_state = None

        self._active_connection_view = None
        self._active_connection_view_handle = None
        self._start_port = None  # Port where connection view pull starts
        self._check_port = None  # Port of connection view that is not pulled

    def on_button_press(self, event):
        view = self.view
        item, handle = HandleFinder(view.hovered_item, view).get_handle_at_point((event.x, event.y))

        if isinstance(item, ConnectionView):
            self._active_connection_view = item
            self._active_connection_view_handle = handle
            if handle is item.from_handle():
                self._start_port = item.from_port
                self._check_port = item.to_port
            elif handle is item.to_handle():
                self._start_port = item.to_port
                self._check_port = item.from_port

        # Set start state
        if isinstance(item, StateView) or isinstance(item, ScopedVariableView):
            self._start_state = item

        super(MyHandleTool, self).on_button_press(event)

    def on_button_release(self, event):
        # Create new transition if pull beginning at port occurred
        if self._new_transition:
            self._create_new_connection()

            # remove placeholder from canvas
            self._new_transition.remove_connection_from_ports()
            self.view.canvas.remove(self._new_transition)

        if self._last_active_port is not self._start_port:
            item = self._active_connection_view
            handle = self._active_connection_view_handle
            if isinstance(item, TransitionView):
                self._handle_transition_view_change(item, handle)
            elif isinstance(item, DataFlowView):
                self._handle_data_flow_view_change(item, handle)

        # reset temp variables
        self._last_active_port = None
        self._check_port = None
        self._new_transition = None
        self._start_state = None
        self._active_connection_view = None
        self._active_connection_view_handle = None

        super(MyHandleTool, self).on_button_release(event)

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
                (isinstance(self.grabbed_item, StateView) or isinstance(self.grabbed_item, ScopedVariableView)) and
                not event.state & gtk.gdk.CONTROL_MASK):
            canvas = view.canvas
            # start_state = self.grabbed_item
            start_state = self._start_state
            start_state_parent = canvas.get_parent(start_state)

            handle = self.grabbed_handle
            start_port = self.get_port_for_handle(handle, start_state)

            # If the start state has a parent continue (ensure no transition is created from top level state)
            if ((isinstance(start_state_parent, StateView) or (start_state_parent is None and isinstance(start_port, IncomeView)) or
                    (start_state_parent is None and isinstance(start_port, InputPortView))) and start_port):

                # Go up one hierarchy_level to match the transitions line width
                transition_placeholder = isinstance(start_port, IncomeView) or isinstance(start_port, OutcomeView)
                placeholder_v = ConnectionPlaceholderView(max(start_state.hierarchy_level - 1, 1),
                                                          transition_placeholder)
                self._new_transition = placeholder_v

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
                elif isinstance(start_port, ScopedDataInputPortView):
                    # It is not possible to create connection beginning from Scoped Data Input Port
                    return False
                elif isinstance(start_port, ScopedDataOutputPortView):
                    start_state_parent.connect_to_scoped_variable_output(start_state.port_id, placeholder_v,
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

    def _handle_data_flow_view_change(self, item, handle):

        def check_output_port(port):
            return isinstance(port, OutputPortView) or isinstance(port, ScopedDataOutputPortView)

        def check_input_port(port):
            return isinstance(port, InputPortView) or isinstance(port, ScopedDataInputPortView)

        start_parent = self._start_port.parent
        last_parent = None
        if self._last_active_port:
            last_parent = self._last_active_port.parent

        # Connection changed: input-to-input to input-to-input
        if (check_input_port(self._check_port) and
                check_input_port(self._start_port) and
                check_input_port(self._last_active_port)):
            self._handle_data_flow_change(item, start_parent, last_parent, iti_to_iti=True)
        # Connection changed: input-to-input to output-to-input
        elif (check_input_port(self._check_port) and
                check_input_port(self._start_port) and
                check_output_port(self._last_active_port)):
            self._handle_data_flow_change(item, start_parent, last_parent, iti_to_oti=True)
        # Connection changed: output-to-input to input-to-input
        elif (check_input_port(self._check_port) and
                check_output_port(self._start_port) and
                check_input_port(self._last_active_port)):
            self._handle_data_flow_change(item, start_parent, last_parent, oti_to_iti=True)
        # Connection changed: output-to-input to output-to-input
        elif ((check_output_port(self._check_port) and
                    check_input_port(self._start_port) and
                    check_input_port(self._last_active_port)) or
                (check_input_port(self._check_port) and
                    check_output_port(self._start_port) and
                    check_output_port(self._last_active_port))):
            self._handle_data_flow_change(item, start_parent, last_parent, oti_to_oti=True)
        # Connection changed: output-to-input to output-to-output
        elif (check_output_port(self._check_port) and
                check_input_port(self._start_port) and
                check_output_port(self._last_active_port)):
            self._handle_data_flow_change(item, start_parent, last_parent, oti_to_oto=True)
        # Connection changed: output-to-output to output-to-input
        elif (check_output_port(self._check_port) and
                check_output_port(self._start_port) and
                check_input_port(self._last_active_port)):
            self._handle_data_flow_change(item, start_parent, last_parent, oto_to_oti=True)
        # Connection changed: output-to-output to output-to-output
        elif (check_output_port(self._check_port) and
                check_output_port(self._start_port) and
                check_output_port(self._last_active_port)):
            self._handle_data_flow_change(item, start_parent, last_parent, oto_to_oto=True)
        # Everything else: Reset to original position
        else:
            self._data_handle_reset_ports(item, handle, start_parent)

    def _handle_transition_view_change(self, item, handle):
        start_parent = self._start_port.parent
        last_parent = None
        if self._last_active_port:
            last_parent = self._last_active_port.parent

        # Connection changed: outcome-to-outcome to outcome-to-outcome
        if (isinstance(self._check_port, OutcomeView) and
                isinstance(self._start_port, OutcomeView) and
                isinstance(self._last_active_port, OutcomeView)):
            self._handle_transition_change(item, start_parent, last_parent, oto_to_oto=True)
        # Connection changed: outcome-to-outcome to outcome-to-income
        elif (isinstance(self._check_port, OutcomeView) and
                isinstance(self._start_port, OutcomeView) and
                isinstance(self._last_active_port, IncomeView)):
            self._handle_transition_change(item, start_parent, last_parent, oto_to_oti=True)
        # Connection changed: outcome-to-income to outcome-to-outcome
        elif (isinstance(self._check_port, OutcomeView) and
                isinstance(self._start_port, IncomeView) and
                isinstance(self._last_active_port, OutcomeView)):
            self._handle_transition_change(item, start_parent, last_parent, oti_to_oto=True)
        # Connection changed: outcome-to-income to outcome-to-income
        elif ((isinstance(self._check_port, IncomeView) and
                    isinstance(self._start_port, OutcomeView) and
                    isinstance(self._last_active_port, OutcomeView)) or
                (isinstance(self._check_port, OutcomeView) and
                    isinstance(self._start_port, IncomeView) and
                    isinstance(self._last_active_port, IncomeView))):
            self._handle_transition_change(item, start_parent, last_parent, oti_to_oti=True)
        # Connection changed: income-to-income to income-to-income (Start State changed)
        elif (isinstance(self._check_port, IncomeView) and
                isinstance(self._start_port, IncomeView) and
                isinstance(self._last_active_port, IncomeView)):
            self._handle_transition_change(item, start_parent, last_parent, iti_to_iti=True)
        else:
            self._handle_reset_ports(item, handle, start_parent)

    @staticmethod
    def _assert_exactly_one_true(bool_list):
        assert isinstance(bool_list, list)
        counter = 0
        for item in bool_list:
            if item:
                counter += 1
        return counter == 1

    def _handle_data_flow_change(self, item, start_parent, last_parent, iti_to_iti=False, iti_to_oti=False,
                                 oti_to_iti=False, oti_to_oti=False, oti_to_oto=False, oto_to_oti=False,
                                 oto_to_oto=False):
        if last_parent is None:
            return
        if not self._assert_exactly_one_true([iti_to_iti, iti_to_oti, oti_to_iti, oti_to_oti, oti_to_oto, oto_to_oti,
                                              oto_to_oto]):
            return

        # DEBUG
        logger.debug("iti_to_iti: %s" % iti_to_iti)
        logger.debug("iti_to_oti: %s" % iti_to_oti)
        logger.debug("oti_to_iti: %s" % oti_to_iti)
        logger.debug("oti_to_oti: %s" % oti_to_oti)
        logger.debug("oti_to_oto: %s" % oti_to_oto)
        logger.debug("oto_to_oti: %s" % oto_to_oti)
        logger.debug("oto_to_oto: %s" % oto_to_oto)

        check = self._check_port
        last = self._last_active_port

        port_moved = None
        if check is item.from_port:
            port_moved = PortMoved.TO
        elif check is item.to_port:
            port_moved = PortMoved.FROM

        def reset_handle():
            if port_moved is PortMoved.TO:
                self._data_handle_reset_ports(item, item.to_handle(), start_parent)
            elif port_moved is PortMoved.FROM:
                self._data_handle_reset_ports(item, item.from_handle(), start_parent)

        if port_moved is PortMoved.TO:
            # If other port in same parent
            if start_parent is last_parent and (iti_to_iti or oti_to_oti or oto_to_oto):
                item.data_flow_m.data_flow.to_key = last.port_id
            # If other port not in same parent
            elif start_parent is not last_parent and (iti_to_iti or oti_to_oti or oti_to_oto or oto_to_oti):
                # Check if InputPortView (not at ScopedVariable) is already connected
                if isinstance(last, InputPortView) and last.has_incoming_connection():
                    reset_handle()
                    return
                # Only connect output to output if to_port is in parent of from_port parent
                elif oti_to_oto and last_parent is not self.get_parents_parent_for_port(item.from_port):
                    reset_handle()
                    return
                to_state_id = self.get_state_id_for_port(last)
                to_key = last.port_id
                item.data_flow_m.data_flow.modify_target(to_state_id, to_key)
            else:
                reset_handle()
                return
        elif port_moved is PortMoved.FROM:
            # If other port in same parent
            if start_parent is last_parent and (iti_to_iti or oti_to_oti or oto_to_oto):
                item.data_flow_m.data_flow.from_key = last.port_id
            # If other port not in same parent
            elif start_parent is not last_parent and (iti_to_oti or oti_to_iti or oti_to_oti or oto_to_oto):
                # Prevent to connect input from state in same hierarchy or below
                if oti_to_iti and last_parent is not self.get_parents_parent_for_port(item.to_port):
                    reset_handle()
                    return
                # Prevent to connect output of parent to input of child
                elif oti_to_oti and last_parent is self.get_parents_parent_for_port(item.to_port):
                    reset_handle()
                    return
                # Prevent to connect output with output of same state
                elif oto_to_oto and last_parent is item.to_port.parent:
                    reset_handle()
                    return
                from_state_id = self.get_state_id_for_port(last)
                from_key = last.port_id
                item.data_flow_m.data_flow.modify_origin(from_state_id, from_key)
            else:
                reset_handle()
                return

    def _data_handle_reset_ports(self, connection, handle, start_parent):

        if handle not in connection.handles():
            return

        self.disconnect_last_active_port(handle, connection)
        self.view.canvas.disconnect_item(connection, handle)

        if isinstance(start_parent, ScopedVariableView):
            if isinstance(self._start_port, ScopedDataInputPortView):
                start_parent.parent_state.connect_to_scoped_variable_input(self._start_port.port_id, connection, handle)
            if isinstance(self._start_port, ScopedDataOutputPortView):
                start_parent.parent_state.connect_to_scoped_variable_output(self._start_port.port_id, connection, handle)
        elif isinstance(start_parent, StateView):
            if isinstance(self._start_port, InputPortView):
                start_parent.connect_to_input_port(self._start_port.port_id, connection, handle)
            if isinstance(self._start_port, OutputPortView):
                start_parent.connect_to_output_port(self._start_port.port_id, connection, handle)

        self.view.canvas.update()

    @staticmethod
    def get_state_id_for_port(port):
        parent = port.parent
        if isinstance(parent, StateView):
            return parent.state_m.state.state_id
        elif isinstance(parent, ScopedVariableView):
            return parent.parent_state.state_m.state.state_id

    def get_parents_parent_for_port(self, port):
        port_parent = port.parent
        if isinstance(port_parent, StateView):
            return self.view.canvas.get_parent(port_parent)
        elif isinstance(port_parent, ScopedVariableView):
            return port_parent.parent_state
        else:
            return None

    def _handle_transition_change(self, item, start_parent, last_parent, oto_to_oto=False, oto_to_oti=False,
                                  oti_to_oto=False, oti_to_oti=False, iti_to_iti=False):
        """
        This method handles the change of transitions and checks if the new transition is valid.
        Exactly one of the optional parameters need be set to True
        :param item: TransitionView holding the changed transition
        :param start_parent: Parent of start_port
        :param last_parent: Parent of last_active_port
        :param oto_to_oto: Indicates check for 'outcome-to-outcome to outcome-to-outcome' change
        :param oto_to_oti: Indicates check for 'outcome-to-outcome to outcome-to-income' change
        :param oti_to_oto: Indicates check for 'outcome-to-income to outcome-to-outcome' change
        :param oti_to_oti: Indicates check for 'outcome-to-income to outcome-to-income' change
        :param iti_to_iti: Indicates check for 'income-to-income to income-to-income' change
        """
        if last_parent is None:
            return
        if not self._assert_exactly_one_true([oto_to_oto, oto_to_oti, oti_to_oto, oti_to_oti, iti_to_iti]):
            return

        start_outcome_id = None
        from_outcome_id = None
        last_outcome_id = None

        if isinstance(self._start_port, OutcomeView):
            start_outcome_id = self._start_port.outcome_id
        if isinstance(self._check_port, OutcomeView):
            from_outcome_id = self._check_port.outcome_id
        if isinstance(self._last_active_port, OutcomeView):
            last_outcome_id = self._last_active_port.outcome_id
        state_id = last_parent.state_m.state.state_id

        if oti_to_oto:
            state_id = start_parent.state_m.state.state_id
            start_parent = self.view.canvas.get_parent(start_parent)

        # if check_port is from_port then to_port has changed
        if self._check_port is item.from_port:
            # if start_port parent is last_active_port parent then connection is still in parent state
            if start_parent is last_parent and (oto_to_oto or oti_to_oto):
                item.transition_m.transition.to_outcome = last_outcome_id
            elif oto_to_oti or oti_to_oto or oti_to_oti or iti_to_iti:
                item.transition_m.transition.to_state = state_id
            # if not in same parent reset transition to initial connection
            else:
                item.transition_m.transition.to_outcome = start_outcome_id
            return
        elif oti_to_oto:
            item.transition_m.transition.from_outcome = from_outcome_id
            return
        elif iti_to_iti:
            self._handle_reset_ports(item, item.from_handle(), start_parent)
            return
        # if check_port is to_port then from_port has changed
        elif self._check_port is item.to_port and (oto_to_oto or oti_to_oti):
            # if start_port parent is last_active_port parent then connection is still in parent state
            if start_parent is last_parent:
                if not self._last_active_port.has_outgoing_connection():
                    item.transition_m.transition.from_outcome = last_outcome_id
                else:
                    item.transition_m.transition.from_outcome = start_outcome_id
                return
            # if not in same parent but other state outcome then modify origin
            else:
                if not self._last_active_port.has_outgoing_connection():
                    if self.is_state_id_root_state(state_id):
                        item.transition_m.transition.from_outcome = start_outcome_id
                    else:
                        item.transition_m.transition.modify_origin(state_id, last_outcome_id)
                    return
                else:
                    item.transition_m.transition.from_outcome = start_outcome_id
                return

    def _handle_reset_ports(self, item, handle, start_parent):

        if handle not in item.handles():
            return

        self.disconnect_last_active_port(handle, item)

        start_outcome_id = None
        if isinstance(self._start_port, OutcomeView):
            start_outcome_id = self._start_port.outcome_id

        self.view.canvas.disconnect_item(item, handle)

        if start_outcome_id is None:
            start_parent.connect_to_income(item, handle)
        else:
            start_parent.connect_to_outcome(start_outcome_id, item, handle)

        self.view.canvas.update()

    def is_state_id_root_state(self, state_id):
        for state_v in self.view.canvas.get_root_items():
            if state_v.state_m.state.state_id == state_id:
                return True
        return False

    def _set_motion_handle(self, event):
        """
        Sets motion handle to currently grabbed handle
        """
        item = self.grabbed_item
        handle = self.grabbed_handle
        pos = event.x, event.y
        self.motion_handle = MyHandleInMotion(item, handle, self.view)
        self.motion_handle.start_move(pos)

    def _create_new_connection(self):
        """
        Creates a new connection and adds this connection to the state machine model.
        If the new connection is transition:
        Only transitions from outcomes are added (transitions from incomes are added via "start_state" checkbox
        """
        nt_from_port = self._new_transition.from_port
        nt_to_port = self._new_transition.to_port

        # Ensure from_port is an outcome and
        # from_port as well as to_port are connected to transition and
        # ports are not the same port
        if nt_from_port and nt_to_port and nt_from_port is not nt_to_port:
            # Make sure the ports are not connected to an outcome of the same state
            if (nt_from_port.parent is nt_to_port.parent and isinstance(nt_from_port, OutcomeView) and
                    isinstance(nt_to_port, OutcomeView)):
                logger.warn("Cannot connect outcome to outcome of the same state")
                return
            if (nt_from_port.parent is nt_to_port.parent and isinstance(nt_from_port, IncomeView) and
                    isinstance(nt_to_port, OutcomeView)):
                logger.warn("Cannot connect income to outcome of the same state")
                return
            if isinstance(nt_from_port, InputPortView) and isinstance(nt_to_port, OutputPortView):
                logger.warn("Cannot connect input to output")
                return

            if ((isinstance(nt_from_port, IncomeView) or isinstance(nt_from_port, OutcomeView)) and
                    (isinstance(nt_to_port, IncomeView) or isinstance(nt_to_port, OutcomeView))):
                self._add_transition(nt_to_port, nt_from_port)
            elif ((isinstance(nt_from_port, InputPortView) or isinstance(nt_from_port, OutputPortView)) and
                    (isinstance(nt_to_port, InputPortView) or isinstance(nt_to_port, OutputPortView))):
                self._add_data_flow(nt_to_port, nt_from_port)
            elif (isinstance(nt_from_port, ScopedDataOutputPortView) and
                    (isinstance(nt_to_port, ScopedDataInputPortView) or isinstance(nt_to_port, InputPortView) or
                        isinstance(nt_to_port, OutputPortView)) or
                    ((isinstance(nt_from_port, OutputPortView) or isinstance(nt_from_port, InputPortView)) and
                     isinstance(nt_to_port, ScopedDataInputPortView))):
                self._add_data_flow_scoped(nt_to_port, nt_from_port)

    def _add_data_flow(self, nt_to_port, nt_from_port):

        from_state_v = nt_from_port.parent
        to_state_v = nt_to_port.parent

        from_state_m = from_state_v.state_m
        to_state_m = to_state_v.state_m

        if (isinstance(nt_from_port, InputPortView) and
                isinstance(nt_to_port, InputPortView) and
                from_state_m is to_state_m.parent):
            responsible_parent_m = from_state_m
            if nt_to_port.has_incoming_connection():
                return
        elif (isinstance(nt_from_port, OutputPortView) and
                isinstance(nt_to_port, OutputPortView) and
                to_state_m is from_state_m.parent):
            responsible_parent_m = to_state_m
            if nt_to_port.has_incoming_connection():
                return
        elif (isinstance(nt_from_port, OutputPortView) and
                isinstance(nt_to_port, InputPortView) and
                to_state_m.parent is from_state_m.parent):
            responsible_parent_m = to_state_m.parent
            if nt_to_port.has_incoming_connection():
                return
        else:
            return

        from_state_id = from_state_m.state.state_id
        from_data_port_id = nt_from_port.port_id
        to_state_id = to_state_m.state.state_id
        to_data_port_id = nt_to_port.port_id

        if isinstance(responsible_parent_m.state, ContainerState):
            try:
                responsible_parent_m.state.add_data_flow(from_state_id, from_data_port_id, to_state_id, to_data_port_id)
            except AttributeError as e:
                logger.warn(e)

    @staticmethod
    def _add_data_flow_scoped(nt_to_port, nt_from_port):
        if isinstance(nt_from_port, ScopedDataOutputPortView):
            scoped_variable_v = nt_from_port.parent
            # if from and to port are part of same scoped variable no connection is created
            if isinstance(nt_to_port, ScopedDataInputPortView) and scoped_variable_v is nt_to_port.parent:
                return
            elif isinstance(nt_to_port, InputPortView) and scoped_variable_v.parent_state is nt_to_port.parent:
                return
            elif isinstance(nt_to_port, OutputPortView) and scoped_variable_v.parent_state is not nt_to_port.parent:
                return
            responsible_parent_m = scoped_variable_v.parent_state.state_m
            from_state_id = scoped_variable_v.parent_state.state_m.state.state_id
            from_data_port_id = scoped_variable_v.port_id
            if isinstance(nt_to_port, ScopedDataInputPortView):
                to_state_id = nt_to_port.parent.parent_state.state_m.state.state_id
                to_data_port_id = nt_to_port.parent.port_id
            else:
                to_state_id = nt_to_port.parent.state_m.state.state_id
                to_data_port_id = nt_to_port.port_id
        elif isinstance(nt_from_port, InputPortView):
            if nt_from_port.parent is not nt_to_port.parent.parent_state:
                return
            responsible_parent_m = nt_from_port.parent.state_m
            from_state_id = nt_from_port.parent.state_m.state.state_id
            from_data_port_id = nt_from_port.port_id
            to_state_id = nt_to_port.parent.parent_state.state_m.state.state_id
            to_data_port_id = nt_to_port.parent.port_id
        elif isinstance(nt_from_port, OutputPortView):
            if nt_from_port.parent is nt_to_port.parent.parent_state:
                return
            responsible_parent_m = nt_to_port.parent.parent_state.state_m
            from_state_id = nt_from_port.parent.state_m.state.state_id
            from_data_port_id = nt_from_port.port_id
            to_state_id = nt_to_port.parent.parent_state.state_m.state.state_id
            to_data_port_id = nt_to_port.parent.port_id
        else:
            return

        if isinstance(responsible_parent_m.state, ContainerState):
            responsible_parent_m.state.add_data_flow(from_state_id, from_data_port_id, to_state_id, to_data_port_id)

    def _add_transition(self, nt_to_port, nt_from_port):
        # canvas = self.view.canvas
        from_state_v = nt_from_port.parent
        to_state_v = nt_to_port.parent

        # Gather necessary information to create transition
        from_state_id = None
        from_outcome_id = None
        to_state_id = None
        to_outcome_id = None

        from_state_m = from_state_v.state_m
        to_state_m = to_state_v.state_m
        responsible_parent_m = None

        if isinstance(nt_from_port, OutcomeView):
            from_state_id = self._start_state.state_m.state.state_id
            from_outcome_id = nt_from_port.outcome_id

        if isinstance(nt_to_port, IncomeView) and isinstance(nt_from_port, IncomeView):
            to_state_id = to_state_m.state.state_id
            responsible_parent_m = from_state_m
        elif isinstance(nt_to_port, IncomeView) and isinstance(nt_from_port, OutcomeView):
            to_state_id = to_state_m.state.state_id
            responsible_parent_m = from_state_m.parent
        elif isinstance(nt_to_port, OutcomeView) and isinstance(nt_from_port, OutcomeView):
            to_outcome_id = nt_to_port.outcome_id
            responsible_parent_m = to_state_m

        if responsible_parent_m:
            try:
                responsible_parent_m.state.add_transition(from_state_id,
                                                          from_outcome_id,
                                                          to_state_id,
                                                          to_outcome_id)
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
        if isinstance(state, StateView):
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
        elif isinstance(state, ScopedVariableView):
            if state.input_port.handle == handle:
                return state.input_port
            elif state.output_port.handle == handle:
                return state.output_port

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
            if isinstance(state, ScopedVariableView):
                state = state.parent_state
            if isinstance(state, StateView):
                if (isinstance(connection, TransitionView) or
                        (isinstance(connection, ConnectionPlaceholderView) and connection.transition_placeholder)):
                    if self.set_matching_port([state.income, ], item.port, handle, connection):
                        return
                    elif self.set_matching_port(state.outcomes, item.port, handle, connection):
                        return
                elif (isinstance(connection, DataFlowView) or
                        (isinstance(connection, ConnectionPlaceholderView) and not connection.transition_placeholder)):
                    if self.set_matching_port(state.outputs, item.port, handle, connection):
                        return
                    elif self.set_matching_port(state.inputs, item.port, handle, connection):
                        return
                    elif self.set_matching_port(state.scoped_variables, item.port, handle, connection):
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
            self._last_active_port.tmp_disconnect()
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
        port_to_handle = None

        for port in port_list:
            if isinstance(port, ScopedVariableView):
                if port.input_port.port is matching_port:
                    port_to_handle = port.input_port
                    break
                elif port.output_port.port is matching_port:
                    port_to_handle = port.output_port
                    break
            elif port.port is matching_port:
                port_to_handle = port
                break

        if port_to_handle:
            if self._last_active_port is not port_to_handle:
                self.disconnect_last_active_port(handle, connection)
            port_to_handle.add_connected_handle(handle, connection, moving=True)
            port_to_handle.tmp_connect()
            connection.set_port_for_handle(port_to_handle, handle)
            self._last_active_port = port_to_handle
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
            return MyHandleInMotion(item, handle, self.view).glue(vpos, glue_distance)

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
        item = self.grabbed_item
        handle = self.grabbed_handle
        try:
            if handle and handle.connectable:
                self.connect(item, handle, (event.x, event.y))
        finally:
            return super(MyConnectHandleTool, self).on_button_release(event)