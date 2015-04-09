from awesome_tool.utils import log
from awesome_tool.utils.geometry import point_in_triangle, dist, point_on_line, deg2rad

logger = log.get_logger(__name__)
import sys
import time

from gtk.gdk import SCROLL_DOWN, SCROLL_UP, SHIFT_MASK, CONTROL_MASK, BUTTON1_MASK, BUTTON2_MASK, BUTTON3_MASK
from gtk.gdk import keyval_name
import gobject
import itertools

from math import sin, cos, atan2
from awesome_tool.statemachine.config import global_config
from awesome_tool.statemachine.enums import StateType
from awesome_tool.statemachine.singleton import global_storage
from awesome_tool.statemachine.states.state_helper import StateHelper
from awesome_tool.statemachine.states.concurrency_state import ConcurrencyState
from awesome_tool.mvc.clipboard import ClipboardType, global_clipboard
from awesome_tool.mvc.statemachine_helper import StateMachineHelper
from awesome_tool.mvc.history import History
from awesome_tool.mvc.controllers.extended_controller import ExtendedController
from awesome_tool.mvc.models import ContainerStateModel, StateModel, TransitionModel, DataFlowModel
from awesome_tool.mvc.models.state_machine import StateMachineModel
from awesome_tool.mvc.models.scoped_variable import ScopedVariableModel
from awesome_tool.mvc.models.data_port import DataPortModel


class GraphicalEditorController(ExtendedController):
    """Controller handling the graphical editor

    :param awesome_tool.mvc.models.state_machine.StateMachineModel model: The state machine model, holding the root
        state and the current selection
    :param awesome_tool.mvc.views.graphical_editor.GraphicalEditorView view: The GTK view having an OpenGL rendering
        element
    """
    __observables__ = ('state_machine', 'root_state')

    def __init__(self, model, view):
        """Constructor
        """
        assert isinstance(model, StateMachineModel)
        ExtendedController.__init__(self, model, view)
        self.root_state_m = model.root_state

        self.timer_id = None

        self.selection = None
        self.selection_start_pos = (0, 0)
        self.mouse_move_start_coords = (0, 0)
        self.last_button_pressed = -1
        self.drag_origin_offset = None
        self.multi_selection_started = False

        self.selected_outcome = None
        self.selected_port_type = None
        self.selected_port_connector = False
        self.selected_waypoint = None
        self.selected_resizer = None

        self.mouse_move_redraw = False
        self.temporary_waypoints = []

        self.shift_modifier = False
        self.alt_modifier = False
        self.ctrl_modifier = False
        self.space_bar = False

        view.editor.connect('expose_event', self._on_expose_event)
        view.editor.connect('button-press-event', self._on_mouse_press)
        view.editor.connect('button-release-event', self._on_mouse_release)
        view.editor.connect('motion-notify-event', self._on_mouse_motion)
        view.editor.connect('scroll-event', self._on_scroll)
        view.editor.connect('key-press-event', self._on_key_press)
        view.editor.connect('key-release-event', self._on_key_release)

        self.last_time = time.time()

    def register_view(self, view):
        """Called when the View was registered
        """
        pass

    def register_adapters(self):
        """Adapters should be registered in this method call
        """
        pass

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param awesome_tool.mvc.shortcut_manager.ShortcutManager shortcut_manager:
        """
        shortcut_manager.add_callback_for_action("delete", self._delete_selection)
        shortcut_manager.add_callback_for_action("add", self._add_execution_state)
        shortcut_manager.add_callback_for_action("info", self._toggle_data_flow_visibility)
        shortcut_manager.add_callback_for_action("abort", self._abort)

        shortcut_manager.add_callback_for_action("copy", self._copy_selection)
        shortcut_manager.add_callback_for_action("paste", self._paste_clipboard)
        shortcut_manager.add_callback_for_action("cut", self._cut_selection)

    @ExtendedController.observe("state_machine", after=True)
    def state_machine_change(self, model, prop_name, info):
        """Called on any change within th state machine

        This method is called, when any state, transition, data flow, etc. within the state machine changes. This
        then typically requires a redraw of the graphical editor, to display these changes immediately.

        :param model: The state machine model
        :param prop_name: The property that was changed
        :param info: Information about the change
        """
        if 'method_name' in info and info['method_name'] == 'root_state_after_change':
            if 'method_name' in info['kwargs']['info']:
                cause = info['kwargs']['info']['method_name']
                root_cause_is_state = False
            else:
                cause = info['kwargs']['method_name']
                root_cause_is_state = True

            # logger.debug("Change in SM, cause: " + cause + " (root cause was state: " + str(root_cause_is_state) + ")")

            self._redraw()

    @ExtendedController.observe("root_state", after=True)
    def root_state_change(self, model, prop_name, info):
        """Called when the root state was exchanged

        Exchanges the local reference to the root state and redraws.

        :param model: The state machine model
        :param prop_name: The root state
        :param info: Information about the change
        """
        if self.root_state_m is not model.root_state:
            logger.debug("The root state was exchanged")
            self.root_state_m = model.root_state
            self._redraw()


    @ExtendedController.observe("selection", after=True)
    def selection_change(self, model, prop_name, info):
        """Called when the selection was changed externally

        Updates the local selection and redraws.

        :param model: The state machine model
        :param prop_name: The selection
        :param info: Information about the change
        """
        selection = None
        for selection in self.model.selection:
            pass
        if self.selection != selection:
            self.selection = selection
            self._redraw()

    def _on_expose_event(self, *args):
        """Redraw the graphical editor

        This method is called typically when the editor window is resized or something triggers are redraw. This
        controller class handles the logic for redrawing, while the corresponding view handles the design.

        :param args: console arguments, not used
        """

        # Prepare the drawing process
        self.view.editor.expose_init(args)
        # The whole logic of drawing is triggered by calling the root state to be drawn
        self.draw_state(self.root_state_m)
        self.draw_state_machine()
        # Finish the drawing process (e.g. swap buffers)
        self.view.editor.expose_finish(args)

    def _redraw(self, timer_triggered=False):
        """Force the graphical editor to be redrawn

        First triggers the configure event to cause the perspective to be updated, then trigger the actual expose
        event to redraw.
        """
        redraw_after = 1 / 50.  # sec
        # Check if initialized
        # and whether the last redraw was more than redraw_after ago
        if hasattr(self.view, "editor") and (time.time() - self.last_time > redraw_after):
            # Remove any existing timer id
            self.timer_id = None
            self.view.editor.emit("configure_event", None)
            self.view.editor.emit("expose_event", None)
            self.last_time = time.time()
            return False  # Causes the periodic timer to stop
        # If the last redraw was less than redraw_after ago or the view is not initialized, yet, set a timer to try
        # again later
        else:
            # Only set the timer, if no timer is existing
            if self.timer_id is None:
                self.timer_id = gobject.timeout_add(int(redraw_after * 1000), self._redraw, True)
            else:
                return True  # Causes the periodic timer to continue

    def _on_key_press(self, widget, event):
        key_name = keyval_name(event.keyval)
        if key_name == "Control_L" or key_name == "Control_R":
            self.ctrl_modifier = True
        elif key_name == "Alt_L":
            self.alt_modifier = True
        elif key_name == "Shift_L" or key_name == "Shift_R":
            self.shift_modifier = True
        elif key_name == "space":
            self.space_bar = True

    def _on_key_release(self, widget, event):
        key_name = keyval_name(event.keyval)
        if key_name == "Control_L" or key_name == "Control_R":
            self.ctrl_modifier = False
        elif key_name == "Alt_L":
            self.alt_modifier = False
        elif key_name == "Shift_L" or key_name == "Shift_R":
            self.shift_modifier = False
        elif key_name == "space":
            self.space_bar = False

    def _on_mouse_press(self, widget, event):
        """Triggered when the mouse is pressed

        Different actions can result from a mouse click, e. g. selecting or drag and drop.

        :param widget: The widget beneath the mouse when the click was done
        :param event: Information about the event, e. g. x and y coordinate
        """
        # Set the focus on the graphical editor, as this is not done automatically
        self.view.editor.grab_focus()

        self.last_button_pressed = event.button
        self.selected_waypoint = None  # reset
        # self.selected_outcome = None  # reset
        # self.selected_port_type = None  # reset
        # self.selected_port_connector = False  # reset
        self.selected_resizer = None  # reset
        self.multi_selection_started = False  # reset

        # Store the coordinates of the event
        self.mouse_move_start_pos = (event.x, event.y)
        self.mouse_move_last_pos = (event.x, event.y)
        self.mouse_move_start_coords = self.view.editor.screen_to_opengl_coordinates((event.x, event.y))
        self.mouse_move_last_coords = self.view.editor.screen_to_opengl_coordinates((event.x, event.y))

        # Left mouse button was clicked
        if event.button == 1:

            # Check if something was selected
            new_selection = self._find_selection(event.x, event.y)

            # Check, whether a resizer was clicked on
            self._check_for_resizer_selection(new_selection, self.mouse_move_start_coords)

            # Check, whether a waypoint was clicked on
            self._check_for_waypoint_selection(new_selection, self.mouse_move_start_coords)

            # We do not want to change the current selection while creating a new transition or data flow
            if not self.mouse_move_redraw:
                # Multi selection with shift+click
                if event.state & SHIFT_MASK != 0:
                    # With shift+click, also states are resized and waypoints snapped. Thus we only want to draw a
                    # selection frame, when the user didn't clicked on a resizer or waypoint
                    if self.selected_resizer is None and self.selected_waypoint is None:
                        self.multi_selection_started = True

                # In the case of multi selection, the user can add/remove elements to/from the selection
                # The selection can consist of more than one model
                if self.multi_selection_started:
                    if new_selection is not None:
                        # Remove from selection, if new_selection is already selected
                        if self.model.selection.is_selected(new_selection):
                            self.model.selection.remove(new_selection)
                        # Add new_selection to selection
                        else:
                            self.model.selection.add(new_selection)
                        # Store the last selection locally
                        if self.selection is None:
                            self.selection = new_selection
                # Only do something, if the user didn't click the second time on a specific model
                elif new_selection != self.selection or len(self.model.selection) > 1:
                    # No multi selection, thus we first have to clear the current selection
                    if self.selection is not None:
                        self.model.selection.clear()
                    # Then we both store the selection locally and in the selection class
                    self.selection = new_selection
                    if self.selection is not None:
                        self.model.selection.set(self.selection)
                        # Add this if a click shell toggle the selection
                        # else:
                        # self.model.selection.clear()
                        # self.selection = None

            # If a state was clicked on, store the original position of the selected state for a drag and drop movement
            if self.selection is not None and isinstance(self.selection, StateModel):
                self.selection_start_pos = (self.selection.meta['gui']['editor']['pos_x'],
                                            self.selection.meta['gui']['editor']['pos_y'])

            # Check, whether an outcome was clicked on
            outcome_state, outcome_key = self._check_for_outcome_selection(new_selection, self.mouse_move_start_coords)
            if outcome_state is not None:
                # Store the selected outcome if no outcome was selected before, this is the start of a drag and drop
                # operation to create a new transition
                if self.selected_outcome is None:
                    if outcome_state is not self.root_state_m or outcome_key is None:
                        self.selected_outcome = outcome_state, outcome_key
                        self.mouse_move_redraw = True
                # If there is already a selected outcome, then we create a transition between the previously selected
                # and the new one. This is the end of a drag and drop operation to create a transition.
                else:
                    self._create_new_transition(outcome_state, outcome_key)
            # Another possibility to create a transition is by clicking the state of the transition target when
            # having an outcome selected.
            elif self.selected_outcome is not None and isinstance(new_selection, StateModel) and \
                    ((new_selection.parent is self.selected_outcome[0].parent and
                              self.selected_outcome[1] is not None) or
                    (new_selection.parent is self.selected_outcome[0] and self.selected_outcome[1] is None)):
                self._create_new_transition(new_selection)
            # Allow the user to create waypoints while creating a new transition
            elif self.selected_outcome is not None:
                self._handle_new_waypoint()

            # Check, whether a port (input, output, scope) was clicked on
            if global_config.get_config_value('show_data_flows', True):
                # Check, whether port (connector) was clicked on
                port_model, port_type, is_connector = self._check_for_port_selection(new_selection,
                                                                                     self.mouse_move_start_coords)
                if port_model is not None:
                    # Store the selected port if no port was selected before, this is the start of a drag and drop
                    # operation to create a new data flow
                    if not self.selected_port_connector and is_connector:
                        self.model.selection.set(port_model)
                        self.selected_port_type = port_type
                        self.selected_port_connector = True
                        self.mouse_move_redraw = True
                    # If there is already a selected port, then we create a data flow between the previously selected
                    # and the new one. This is the end of a drag and drop operation to create a data flow.
                    else:
                        self._create_new_data_flow(port_model)
                # Allow the user to create waypoints while creating a new data flow
                elif isinstance(self.selection, (DataPortModel, ScopedVariableModel)):
                    self._handle_new_waypoint()

            self._redraw()

        # Right mouse button was clicked on
        if event.button == 3:

            # Check if something was selected
            click = self.view.editor.screen_to_opengl_coordinates((event.x, event.y))
            clicked_model = self._find_selection(event.x, event.y)

            # When a new transition is created, the creation can be aborted with a right click
            if self.selected_outcome is not None or self.selected_port_connector:
                self._abort()

            # If a connection (transition or data flow) was clicked
            if isinstance(clicked_model, TransitionModel) or isinstance(clicked_model, DataFlowModel):

                # If the right click was on a waypoint of a connection, the waypoint is removed
                waypoint_removed = self._check_for_waypoint_removal(click, clicked_model)

                # If no waypoint was removed, we want to add one at that position
                if not waypoint_removed:
                    self._add_waypoint(clicked_model, click)

    def _on_mouse_release(self, widget, event):
        """Triggered when a mouse button is being released

        :param widget: The widget beneath the mouse when the release was done
        :param event: Information about the event, e. g. x and y coordinate
        Not used so far
        """
        self.last_button_pressed = None
        self.drag_origin_offset = None

        if self.multi_selection_started:
            self._check_for_multi_selection()

        self._redraw()

    def _on_mouse_motion(self, widget, event):
        """Triggered when the mouse is moved

        When a state is selected, this causes a drag and drop movement.

        :param widget: The widget beneath the mouse when the click was done
        :param event: Information about the event, e. g. x and y coordinate
        """

        # If no mouse button is pressed while the mouse is moving, we only have to change whether another component
        # wants to redraw the editor on mouse move
        if event.state & (BUTTON1_MASK | BUTTON2_MASK | BUTTON3_MASK) == 0:
            if self.mouse_move_redraw:
                mouse_current_coord = self.view.editor.screen_to_opengl_coordinates((event.x, event.y))
                self.mouse_move_last_coords = mouse_current_coord
                self._redraw()
            return

        # Move while middle button is clicked moves the view
        if self.last_button_pressed == 2 or (self.space_bar and event.state & BUTTON1_MASK > 0):
            dx = event.x - self.mouse_move_last_pos[0]
            dy = event.y - self.mouse_move_last_pos[1]
            self._move_view(dx, dy)

        mouse_current_coord = self.view.editor.screen_to_opengl_coordinates((event.x, event.y))

        if self.multi_selection_started:
            self._draw_multi_selection_frame()
            self.mouse_move_last_pos = (event.x, event.y)
            self.mouse_move_last_coords = mouse_current_coord
            return

        rel_x_motion = mouse_current_coord[0] - self.mouse_move_start_coords[0]
        rel_y_motion = mouse_current_coord[1] - self.mouse_move_start_coords[1]

        # Translate the mouse movement to OpenGL coordinates
        new_pos_x = self.selection_start_pos[0] + rel_x_motion
        new_pos_y = self.selection_start_pos[1] + rel_y_motion

        # Move the selected state (if there is an appropriate one)
        if isinstance(self.selection, StateModel) and \
                        self.selection != self.root_state_m and \
                        self.last_button_pressed == 1 and \
                        self.selected_outcome is None and \
                        self.selected_resizer is None:
            self._move_state(self.selection, new_pos_x, new_pos_y)

        # Move the selected waypoint (if there is one)
        if self.selected_waypoint is not None:
            # Move selected waypoint within its container state
            self._move_waypoint(new_pos_x, new_pos_y, event.state)

        # Move data port
        if isinstance(self.selection, (DataPortModel, ScopedVariableModel)) and not self.selected_port_connector and \
                        self.last_button_pressed == 1:
            self._move_data_port(self.selection, mouse_current_coord)

        # Redraw to show the new transition/data flow the user is creating with drag and drop
        if self.selected_outcome is not None or self.selected_port_connector:
            self._redraw()

        if self.selected_resizer is not None:
            modifier = event.state
            self._resize_state(self.selection, mouse_current_coord, rel_x_motion, rel_y_motion, modifier)

        self.mouse_move_last_pos = (event.x, event.y)
        self.mouse_move_last_coords = mouse_current_coord

    def _on_scroll(self, widget, event):
        """Triggered when the mouse wheel is turned

        Calls the zooming method.

        :param widget: The widget beneath the mouse when the event was triggered
        :param event: Information about the event, e. g. x and y coordinate and mouse wheel turning direction
        """
        self._handle_zooming((event.x, event.y), event.direction)

    @staticmethod
    def _limit_position_to_state(state, pos_x, pos_y, child_width=0, child_height=0):
        if state is not None:
            if pos_x < state.meta['gui']['editor']['pos_x']:
                pos_x = state.meta['gui']['editor']['pos_x']
            elif pos_x + child_width > state.meta['gui']['editor']['pos_x'] + state.meta['gui']['editor']['width']:
                pos_x = state.meta['gui']['editor']['pos_x'] + state.meta['gui']['editor']['width'] - child_width

            if pos_y < state.meta['gui']['editor']['pos_y']:
                pos_y = state.meta['gui']['editor']['pos_y']
            elif pos_y + child_height > state.meta['gui']['editor']['pos_y'] + state.meta['gui']['editor']['height']:
                pos_y = state.meta['gui']['editor']['pos_y'] + state.meta['gui']['editor']['height'] - child_height
        return pos_x, pos_y

    def _check_for_waypoint_selection(self, selection, coords):
        """Check whether a waypoint was clicked on

        Checks whether the current selection is a transition or data flow and if so looks for a waypoint at the given
        coordinates. If a waypoint is found, it is stored together with its current position.

        :param coords: Coordinates to search for waypoints
        """
        if selection is not None and \
                (isinstance(selection, TransitionModel) or isinstance(selection, DataFlowModel)):
            close_threshold = min(selection.parent.meta['gui']['editor']['height'],
                                  selection.parent.meta['gui']['editor']['width)']) / 50.

            # Check distance between all waypoints of the selected transition/data flows and the given coordinates
            for i, waypoint in enumerate(selection.meta['gui']['editor']['waypoints']):
                # Only if coordinates are stored for the waypoints (always should be the case)
                if waypoint[0] is not None and waypoint[1] is not None:
                    if dist(waypoint, coords) < close_threshold:
                        # As tuples cannot be changed, we have to store the whole list plus the index
                        self.selected_waypoint = (selection, i)
                        self.selection_start_pos = (waypoint[0], waypoint[1])
                        break

    @staticmethod
    def _check_for_outcome_selection(selection, coords):
        """Check whether a port was clicked on

        Checks whether the current selection is a state and if so looks for an outcome at the given coordinates. If an
        outcome is found, it is stored.

        :param coords: Coordinates to search for outcomes
        """
        if isinstance(selection, StateModel):  # and self.selection is not self.root_state_m:
            state_m = selection
            outcomes_close_threshold = state_m.meta['gui']['editor']['outcome_radius']
            outcomes = state_m.meta['gui']['editor']['outcome_pos']
            # Check distance between all outcomes of the selected state and the given coordinate
            for key in outcomes:
                if dist(outcomes[key], coords) < outcomes_close_threshold:
                    return state_m, key
            income_pos = (state_m.meta['gui']['editor']['pos_x'], state_m.meta['gui']['editor']['pos_y'] +
                          state_m.meta['gui']['editor']['height'] / 2)
            if dist(income_pos, coords) < outcomes_close_threshold:
                print "income selected"
                return state_m, None
        return None, None

    @staticmethod
    def _check_for_port_selection(model, coords):
        """Check whether a port was clicked on

        The methods checks whether the user clicked on a connector of a port. If the passed model is a state,
        we have to check the positions of all port connectors of that state. If it is a data port, we only have to
        look at the connector position of that port.

        :param model: The model that was clicked on
        :param coords: Coordinates to search for ports
        """
        if isinstance(model, (DataPortModel, ScopedVariableModel)):
            prefix = '' if isinstance(model, ScopedVariableModel) else 'inner_'
            connector_pos = model.meta['gui']['editor'][prefix + 'connector_pos']
            connector_radius = model.meta['gui']['editor'][prefix + 'connector_radius']
            if dist(connector_pos, coords) < connector_radius:
                selected_port_type = "inner" if isinstance(model, DataPortModel) else "scope"
                return model, selected_port_type, True
                # self.model.selection.set(model)
                # self.selected_port_type = "inner" if isinstance(model, DataPortModel) else "scope"
                # self.selected_port_connector = True
        elif isinstance(model, StateModel):
            state_m = model

            for port_m in itertools.chain(state_m.input_data_ports, state_m.output_data_ports):
                connector_pos = port_m.meta['gui']['editor']['outer_connector_pos']
                connector_radius = port_m.meta['gui']['editor']['outer_connector_radius']
                if dist(connector_pos, coords) < connector_radius:
                    # self.model.selection.set(port_m)
                    # self.selected_port_type = "outer"
                    # self.selected_port_connector = True
                    return port_m, "outer", True
                    # break
        return None, None, False

    def _check_for_resizer_selection(self, selection, coords):
        """Check whether a resizer (handle to resize a state) was clicked on

        Checks whether the current selection is a state and if so looks the given coordinates are within the resizer
        of that state. If so, the resizer (or its state model) is stored.

        :param coords: Coordinates to check for the resizer
        """
        if isinstance(selection, StateModel):
            state_editor_data = selection.meta['gui']['editor']
            # Calculate corner points of resizer
            p1 = (state_editor_data['pos_x'] + state_editor_data['width'], state_editor_data['pos_y'])
            p2 = (p1[0] - state_editor_data['resize_length'], p1[1])
            p3 = (p1[0], p1[1] + state_editor_data['resize_length'])

            # The resizer is triangle. Check whether the given coordinates are within that triangle
            if point_in_triangle(coords, p1, p2, p3):
                self.selected_resizer = selection

    def _check_for_multi_selection(self):
        start = self.mouse_move_start_pos
        end = self.mouse_move_last_pos
        # Only check for selection, if the mouse was moved more than 10px
        if dist(start, end) > 10:
            # Determine the size of the selection frame
            pos_x = min(start[0], end[0])
            pos_y = min(start[1], end[1])
            width = abs(start[0] - end[0])
            height = abs(start[1] - end[1])

            # Determine all models within the selection frame
            selected_models = self._find_selection(pos_x, pos_y, width, height, all=True)

            upper_left = (pos_x, pos_y + height)
            lower_right = (pos_x + width, pos_y)
            upper_left = self.view.editor.screen_to_opengl_coordinates(upper_left)
            lower_right = self.view.editor.screen_to_opengl_coordinates(lower_right)

            frame_left = upper_left[0]
            frame_right = lower_right[0]
            frame_bottom = upper_left[1]
            frame_top = lower_right[1]

            def is_within_frame(model):
                left, right, bottom, top = self.get_boundaries(model)
                if left is not None:
                    if frame_left < left < right < frame_right and frame_bottom < bottom < top < frame_top:
                        return True
                return False

            # Remove models, which are not fully in the selection frame
            models_to_remove = set()
            for model in selected_models:
                if not is_within_frame(model):
                    models_to_remove.add(model)

            selected_models = set(selected_models)
            selected_models.difference_update(models_to_remove)

            if selected_models is not None:
                self.model.selection.clear()
                self.model.selection.append(selected_models)
        # If so, select models beneath frame
        self.multi_selection_started = False
        self.model.meta['gui']['editor']['selection_frame'] = None

    def _check_for_waypoint_removal(self, coords, connection_model):
        """Checks and removes a waypoint if necessary

        Checks whether the coordinates given are close to a waypoint of the given connection model (transition or
        data flow). If so, the waypoint is removed.

        :param coords: Coordinates to check for a waypoint
        :param connection_model: Model of a transition or data flow
        :return: True, if a waypoint was removed, False else
        """
        close_threshold = min(connection_model.parent.meta['gui']['editor']['height'],
                              connection_model.parent.meta['gui']['editor']['width)']) / 70.
        # Check distance between all waypoints of the connection to the given coordinates
        for waypoint in connection_model.meta['gui']['editor']['waypoints']:
            if waypoint[0] is not None and waypoint[1] is not None:
                if dist(waypoint, coords) < close_threshold:
                    connection_model.meta['gui']['editor']['waypoints'].remove(waypoint)
                    logger.debug('Connection waypoint removed')
                    self._redraw()
                    return True
        return False

    def _draw_multi_selection_frame(self):
        corner1 = self.mouse_move_start_coords
        corner2 = self.mouse_move_last_coords
        self.model.meta['gui']['editor']['selection_frame'] = [corner1, corner2]
        self._redraw()


    def _add_waypoint(self, connection_model, coords):
        """Adds a waypoint to the given connection

        The methods adds a waypoint at the given coordinates to the given connection (transition or data flow). If
        the connection also has waypoints, it puts the new one between the correct existing ones.

        :param connection_model: The model of the connection to add a waypoint to
        :param coords: The coordinates of the new waypoint
        """

        # The waypoints should exist as dictionary. If not (for any reason), we have to convert it to one
        if isinstance(connection_model.meta['gui']['editor']['waypoints'], dict):
            logger.warn("Connection waypoints was of type dict, expected list")
            connection_model.meta['gui']['editor']['waypoints'] = connection_model.meta['waypoints'].items()

        # Create a list of all connection points, consisting of start, waypoints and end
        points = [(connection_model.meta['gui']['editor']['from_pos_x'],
                   connection_model.meta['gui']['editor']['from_pos_y'])]
        points.extend(connection_model.meta['gui']['editor']['waypoints'])
        points.append((connection_model.meta['gui']['editor']['to_pos_x'],
                       connection_model.meta['gui']['editor']['to_pos_y']))

        # Insert the waypoint at the correct position
        for i in range(len(points) - 1):
            if point_on_line(coords, points[i], points[i + 1]):
                connection_model.meta['gui']['editor']['waypoints'].insert(i, (coords[0], coords[1]))
        logger.debug('Connection waypoint added at {0:.1f} - {1:.1f}'.format(coords[0], coords[1]))
        self._redraw()

    def _create_new_transition(self, to_state_m, to_outcome_id=None):
        """Tries to create a new transition

        The user can create new transition using drag and drop in the graphical editor. The method uses the stored
        selected outcome as starting point and the passed state model and outcome id as target point for the new
        transition.

        :param to_state_m: The to state model of the new transition
        :param to_outcome_id: The id of the to outcome or None if the transition does not go to the parent state
        """
        from_state_id = self.selected_outcome[0].state.state_id
        from_outcome_id = self.selected_outcome[1]
        to_state_id = to_state_m.state.state_id
        # Prevent accidental creation of transitions with double click on one outcome
        if from_state_id == to_state_id and from_outcome_id == to_outcome_id:
            self._abort()
            return

        # Start transition
        if from_outcome_id is None:
            from_state_id = None

        if to_outcome_id is None:
            responsible_parent_m = to_state_m.parent
        else:
            to_state_id = None
            responsible_parent_m = to_state_m

        try:
            transition_id = responsible_parent_m.state.add_transition(from_state_id, from_outcome_id, to_state_id,
                                                                      to_outcome_id)
            transition_m = StateMachineHelper.get_transition_model(responsible_parent_m, transition_id)
            transition_m.meta['gui']['editor']['waypoints'] = self.temporary_waypoints
        except AttributeError as e:
            logger.debug("Transition couldn't be added: {0}".format(e))
        except Exception as e:
            logger.error("Unexpected exception while creating transition: {0}".format(e))

        self._abort()

    def _create_new_data_flow(self, target_port_m):
        """Tries to create a new data flow

        The user can create new data flow using drag and drop in the graphical editor. The method uses the stored
        selected port as starting point and the passed target port model as target point for the new data flow.

        :param target_port_m: The target port of the data flow
        """
        if target_port_m is not None:
            from_port_m = self.selection
            from_state_id = from_port_m.parent.state.state_id
            from_port_id = from_port_m.data_port.data_port_id if isinstance(from_port_m, DataPortModel) else \
                from_port_m.scoped_variable.data_port_id
            target_state_id = target_port_m.parent.state.state_id
            target_port_id = target_port_m.data_port.data_port_id if isinstance(target_port_m, DataPortModel) else \
                target_port_m.scoped_variable.data_port_id

            if self.selected_port_type in ("inner", "scope"):
                responsible_parent = from_port_m.parent
            else:
                responsible_parent = from_port_m.parent.parent

            try:
                data_flow_id = responsible_parent.state.add_data_flow(from_state_id, from_port_id,
                                                                      target_state_id, target_port_id)
                data_flow_m = StateMachineHelper.get_data_flow_model(responsible_parent, data_flow_id)
                data_flow_m.meta['gui']['editor']['waypoints'] = self.temporary_waypoints
            except AttributeError as e:
                logger.debug("Data flow couldn't be added: {0}".format(e))
            except Exception as e:
                logger.error("Unexpected exception while creating data flow: {0}".format(e))

        self._abort()

    def _move_state(self, state_m, new_pos_x, new_pos_y):
        """Move the state to the given position

        The method moves the state and all its child states with their transitions, data flows and waypoints. The
        state is kept within its parent, thus restricting the movement.

        :param awesome_tool.mvc.models.StateModel state_m: The model of the state to be moved
        :param new_pos_x: The desired new x coordinate
        :param new_pos_y: The desired new y coordinate
        """
        old_pos_x = state_m.meta['gui']['editor']['pos_x']
        old_pos_y = state_m.meta['gui']['editor']['pos_y']

        cur_width = state_m.meta['gui']['editor']['width']
        cur_height = state_m.meta['gui']['editor']['height']

        # Keep the state within its container state
        new_pos_x, new_pos_y = self._limit_position_to_state(state_m.parent, new_pos_x, new_pos_y,
                                                             cur_width, cur_height)

        state_m.meta['gui']['editor']['pos_x'] = new_pos_x
        state_m.meta['gui']['editor']['pos_y'] = new_pos_y

        def move_child_states(state_m, move_x, move_y):
            # Move waypoints
            if self.has_content(state_m):
                for transition in state_m.transitions:
                    for i, waypoint in enumerate(transition.meta['gui']['editor']['waypoints']):
                        new_pos = (waypoint[0] + move_x, waypoint[1] + move_y)
                        transition.meta['gui']['editor']['waypoints'][i] = new_pos
                for data_flow in state_m.data_flows:
                    for i, waypoint in enumerate(data_flow.meta['gui']['editor']['waypoints']):
                        new_pos = (waypoint[0] + move_x, waypoint[1] + move_y)
                        data_flow.meta['gui']['editor']['waypoints'][i] = new_pos
                for port_m in itertools.chain(state_m.input_data_ports, state_m.output_data_ports,
                                              state_m.scoped_variables):
                    old_pos = port_m.meta['gui']['editor']['inner_pos']
                    try:  # If data connections are not shown, not all position may be calculated
                        port_m.meta['gui']['editor']['inner_pos'] = (old_pos[0] + move_x, old_pos[1] + move_y)
                    except TypeError:
                        pass
            # Move child states
            for child_state in state_m.states.itervalues():
                child_state.meta['gui']['editor']['pos_x'] += move_x
                child_state.meta['gui']['editor']['pos_y'] += move_y

                if self.has_content(child_state):
                    move_child_states(child_state, move_x, move_y)

        # Move all child states in accordance with the state, to keep their relative position
        if self.has_content(state_m):
            diff_x = new_pos_x - old_pos_x
            diff_y = new_pos_y - old_pos_y
            move_child_states(state_m, diff_x, diff_y)

        self._publish_changes(state_m, "Move state", affects_children=True)
        self._redraw()

    def _move_data_port(self, port_m, coords):
        """Move the port to the given position

        This method moves the given port to the given coordinates, with respect to the mouse offset to the origin od
        the port and with respect to the size of the container state.

        :param port_m: The port model to be moved
        :param coords: The target position
        """
        port_info = port_m.meta['gui']['editor']
        if self.drag_origin_offset is None:
            self.drag_origin_offset = (coords[0] - port_info['inner_pos'][0], coords[1] - port_info['inner_pos'][1])

        new_pos = (coords[0] - self.drag_origin_offset[0], coords[1] - self.drag_origin_offset[1])
        if port_m in port_m.parent.output_data_ports:
            new_pos = self._limit_position_to_state(port_m.parent, new_pos[0] - port_info['width'], new_pos[1],
                                                    port_info['width'], port_info['height'])
            new_pos = (new_pos[0] + port_info['width'], new_pos[1])
        elif port_m in port_m.parent.input_data_ports:
            new_pos = self._limit_position_to_state(port_m.parent, new_pos[0], new_pos[1],
                                                    port_info['width'], port_info['height'])
        else:  # Scope variable
            arrow_height = port_info['height'] - port_info['rect_height']
            new_pos = self._limit_position_to_state(port_m.parent, new_pos[0], new_pos[1] - arrow_height,
                                                    port_info['width'], port_info['height'])
            new_pos = (new_pos[0], new_pos[1] + arrow_height)
        port_info['inner_pos'] = new_pos
        self._publish_changes(port_m.parent, "Move data port", affects_children=False)
        self._redraw()

    def _move_waypoint(self, new_pos_x, new_pos_y, modifier_keys):
        new_pos_x, new_pos_y = self._limit_position_to_state(self.selection.parent, new_pos_x, new_pos_y)
        selected_model = self.selected_waypoint[0]
        waypoints = selected_model.meta['gui']['editor']['waypoints']
        waypoint_id = self.selected_waypoint[1]

        # With the shift key pressed, try to snap the waypoint such that the connection has a multiple of 45 deg
        if modifier_keys & SHIFT_MASK != 0:
            snap_angle = deg2rad(global_config.get_config_value('WAYPOINT_SNAP_ANGLE', 45.))
            snap_diff = deg2rad(global_config.get_config_value('WAYPOINT_SNAP_MAX_DIFF_ANGLE', 10.))
            max_snap_dist = global_config.get_config_value('WAYPOINT_SNAP_MAX_DIFF_PIXEL', 50.)
            max_snap_dist /= self.view.editor.pixel_to_size_ratio()

            def calculate_snap_point(p1, p2, p3):
                def find_closest_snap_angle(angle):
                    multiple = angle // snap_angle
                    multiple = [multiple - 1, multiple, multiple + 1]
                    diff = map(lambda mul: abs(abs(snap_angle * mul) - abs(angle)), multiple)
                    min_index = diff.index(min(diff))
                    return snap_angle * multiple[min_index]

                alpha = atan2(-(p2[1] - p1[1]), p2[0] - p1[0])
                beta = atan2(-(p2[1] - p3[1]), p2[0] - p3[0])
                closest_angle = find_closest_snap_angle(alpha)
                if abs(alpha - closest_angle) < snap_diff:
                    alpha = closest_angle
                closest_angle = find_closest_snap_angle(beta)
                if abs(beta - closest_angle) < snap_diff:
                    beta = closest_angle

                dx = p3[0] - p1[0]
                dy = -(p3[1] - p1[1])
                denominator = cos(alpha) * sin(beta) - sin(alpha) * cos(beta)
                if denominator == 0:
                    return p2
                s = (dx * sin(beta) - dy * cos(beta)) / denominator
                # t = (dx*sin(alpha) - dy*cos(alpha)) / denominator
                snap_x = p1[0] + cos(alpha) * s
                snap_y = p1[1] - sin(alpha) * s

                if dist(p2, (snap_x, snap_y)) < max_snap_dist:
                    return snap_x, snap_y
                return p2

            if waypoint_id > 0:
                prev_point = waypoints[waypoint_id - 1]
            else:
                prev_point = (selected_model.meta['gui']['editor']['from_pos_x'],
                              selected_model.meta['gui']['editor']['from_pos_y'])
            if waypoint_id < len(waypoints) - 1:
                next_point = waypoints[waypoint_id + 1]
            else:
                next_point = (selected_model.meta['gui']['editor']['to_pos_x'],
                              selected_model.meta['gui']['editor']['to_pos_y'])

            point = (new_pos_x, new_pos_y)
            (new_pos_x, new_pos_y) = calculate_snap_point(prev_point, point, next_point)

        waypoints[waypoint_id] = (new_pos_x, new_pos_y)
        self._publish_changes(selected_model, "Move waypoint", affects_children=False)
        self._redraw()

    def _resize_state(self, state_m, mouse_resize_coords, d_width, d_height, modifier_keys):
        """Resize the state by the given delta width and height

        The resize function checks the child states and keeps the state around the children, thus limiting the minimum
        size. Two modifier keys can be used to alter the resize options:
         - Ctrl also causes the child states to be resized
         - Shift caused the resized states to keep their width to height ratio

        :param mouse_resize_coords: The coordinates of the mouse
        :param d_width: The desired change in width
        :param d_height: The desired change in height
        :param modifier_keys: The current pressed modifier keys (mask)
        """
        state_editor_data = state_m.meta['gui']['editor']
        # Keep size ratio?
        if int(modifier_keys & SHIFT_MASK) > 0:
            state_size_ratio = state_editor_data['width'] / state_editor_data['height']
            if d_width / state_size_ratio < d_height:
                mouse_resize_coords = (mouse_resize_coords[0],
                                       self.mouse_move_start_coords[1] - d_width / state_size_ratio)
            else:
                mouse_resize_coords = (self.mouse_move_start_coords[0] - d_height * state_size_ratio,
                                       mouse_resize_coords[1])
        # User wants to resize content by holding the ctrl keys pressed
        resize_content = int(modifier_keys & CONTROL_MASK) > 0
        width = mouse_resize_coords[0] - state_editor_data['pos_x']
        height_diff = state_editor_data['pos_y'] - mouse_resize_coords[1]
        height = state_editor_data['height'] + height_diff
        min_right_edge = state_editor_data['pos_x']
        max_bottom_edge = state_editor_data['pos_y'] + state_editor_data['height']

        # If the content is not supposed to be resized, with have to calculate the inner edges, which define the
        # minimum size of our state
        if not resize_content and self.has_content(self.selection):
            # Check lower right corner of all child states
            for child_state_m in state_m.states.itervalues():
                _, child_right_edge, child_bottom_edge, _ = self.get_boundaries(child_state_m)
                min_right_edge = child_right_edge if min_right_edge < child_right_edge else min_right_edge
                max_bottom_edge = child_bottom_edge if max_bottom_edge > child_bottom_edge else max_bottom_edge
            # Check position of all waypoints of all transitions
            for transition_m in state_m.transitions:
                for waypoint in transition_m.meta['gui']['editor']['waypoints']:
                    min_right_edge = waypoint[0] if min_right_edge < waypoint[0] else min_right_edge
                    max_bottom_edge = waypoint[1] if max_bottom_edge > waypoint[1] else max_bottom_edge
            # Check position of all waypoints of all data flows
            for data_flow_m in state_m.data_flows:
                for waypoint in data_flow_m.meta['gui']['editor']['waypoints']:
                    min_right_edge = waypoint[0] if min_right_edge < waypoint[0] else min_right_edge
                    max_bottom_edge = waypoint[1] if max_bottom_edge > waypoint[1] else max_bottom_edge
            # Check lower right corner of all ports
            for port_m in itertools.chain(state_m.input_data_ports, state_m.output_data_ports,
                                          state_m.scoped_variables):
                _, port_right_edge, port_bottom_edge, _ = self.get_boundaries(port_m)
                min_right_edge = port_right_edge if min_right_edge < port_right_edge else min_right_edge
                max_bottom_edge = port_bottom_edge if max_bottom_edge > port_bottom_edge else max_bottom_edge

        # Check for parent size limitation
        max_right_edge = sys.maxint
        min_bottom_edge = -sys.maxint - 1
        if state_m.parent is not None:
            max_right_edge = state_m.parent.meta['gui']['editor']['pos_x'] + \
                             state_m.parent.meta['gui']['editor']['width']
            min_bottom_edge = state_m.parent.meta['gui']['editor']['pos_y']

        # Desired new edges
        desired_right_edge = state_editor_data['pos_x'] + width
        desired_bottom_edge = state_editor_data['pos_y'] - height_diff

        # Old values
        old_width = state_editor_data['width']
        old_height = state_editor_data['height']
        old_pos_x = state_editor_data['pos_x']
        old_pos_y = state_editor_data['pos_y']

        # Check for all restrictions
        if width > 0:  # Minimum width
            if desired_right_edge > max_right_edge:  # Keep state in its parent
                state_editor_data['width'] = max_right_edge - state_editor_data['pos_x']
            elif desired_right_edge < min_right_edge:  # Surround all children
                state_editor_data['width'] = min_right_edge - state_editor_data['pos_x']
            else:
                state_editor_data['width'] = width
        if height > 0:  # Minimum height
            if desired_bottom_edge > max_bottom_edge:  # Keep state in its parent
                state_editor_data['height'] += state_editor_data['pos_y'] - max_bottom_edge
                state_editor_data['pos_y'] = max_bottom_edge
            elif desired_bottom_edge < min_bottom_edge:  # Surround all children
                state_editor_data['height'] += state_editor_data['pos_y'] - min_bottom_edge
                state_editor_data['pos_y'] = min_bottom_edge
            else:
                state_editor_data['height'] = height
                state_editor_data['pos_y'] -= height_diff

        # Resize factor for width and height
        width_factor = state_editor_data['width'] / old_width
        height_factor = state_editor_data['height'] / old_height

        # Resize content if the state was resized and the modifier key is pressed
        if (width_factor != 1 or height_factor != 1) and resize_content:

            # Recursive call
            def resize_children(state_m, width_factor, height_factor, old_pos_x, old_pos_y):

                def calc_new_pos(old_parent_pos, new_parent_pos, old_self_pos, factor):
                    """Calculate new position of an object

                    The new position is based on the old a new position of the parent, the stretch factor and the old
                    position of the object
                    :param old_parent_pos: Old position (x or y) of the parent
                    :param new_parent_pos: New position (x or y) of the parent
                    :param old_self_pos: Old position (x or y) of the object
                    :param factor: Resize factor of x or y
                    :return: New position of the object (x or y)
                    """
                    diff_pos = old_self_pos - old_parent_pos
                    diff_pos *= factor
                    return new_parent_pos + diff_pos

                # Only container states have content
                if self.has_content(state_m):
                    # Resize all transitions
                    for transition_m in state_m.transitions:
                        # By repositioning all waypoints
                        for i, waypoint in enumerate(transition_m.meta['gui']['editor']['waypoints']):
                            new_pos_x = calc_new_pos(old_pos_x, state_m.meta['gui']['editor']['pos_x'],
                                                     waypoint[0], width_factor)
                            new_pos_y = calc_new_pos(old_pos_y, state_m.meta['gui']['editor']['pos_y'],
                                                     waypoint[1], height_factor)
                            transition_m.meta['gui']['editor']['waypoints'][i] = (new_pos_x, new_pos_y)
                    # Resize all data flows
                    for data_flow_m in state_m.data_flows:
                        # By repositioning all waypoints
                        for i, waypoint in enumerate(data_flow_m.meta['gui']['editor']['waypoints']):
                            new_pos_x = calc_new_pos(old_pos_x, state_m.meta['gui']['editor']['pos_x'],
                                                     waypoint[0], width_factor)
                            new_pos_y = calc_new_pos(old_pos_y, state_m.meta['gui']['editor']['pos_y'],
                                                     waypoint[1], height_factor)
                            data_flow_m.meta['gui']['editor']['waypoints'][i] = (new_pos_x, new_pos_y)

                    for port_m in itertools.chain(state_m.input_data_ports, state_m.output_data_ports,
                                                  state_m.scoped_variables):
                        new_pos_x = calc_new_pos(old_pos_x, state_m.meta['gui']['editor']['pos_x'],
                                                 port_m.meta['gui']['editor']['inner_pos'][0], width_factor)
                        new_pos_y = calc_new_pos(old_pos_y, state_m.meta['gui']['editor']['pos_y'],
                                                 port_m.meta['gui']['editor']['inner_pos'][1], height_factor)
                        port_m.meta['gui']['editor']['inner_pos'] = (new_pos_x, new_pos_y)

                    # Resize all child states
                    for child_state_m in state_m.states.itervalues():
                        child_state_m.meta['gui']['editor']['width'] *= width_factor
                        child_state_m.meta['gui']['editor']['height'] *= height_factor

                        child_old_pos_x = child_state_m.meta['gui']['editor']['pos_x']
                        new_pos_x = calc_new_pos(old_pos_x, state_m.meta['gui']['editor']['pos_x'],
                                                 child_state_m.meta['gui']['editor']['pos_x'], width_factor)
                        child_state_m.meta['gui']['editor']['pos_x'] = new_pos_x

                        child_old_pos_y = child_state_m.meta['gui']['editor']['pos_y']
                        new_pos_y = calc_new_pos(old_pos_y, state_m.meta['gui']['editor']['pos_y'],
                                                 child_state_m.meta['gui']['editor']['pos_y'], height_factor)
                        child_state_m.meta['gui']['editor']['pos_y'] = new_pos_y

                        if self.has_content(child_state_m):
                            resize_children(child_state_m, width_factor, height_factor,
                                            child_old_pos_x, child_old_pos_y)

            # Start recursive call of the content resize
            resize_children(state_m, width_factor, height_factor, old_pos_x, old_pos_y)

        affects_children = self.has_content(self.selection) and resize_content
        self._publish_changes(state_m, "Resize state", affects_children)
        self._redraw()

    def _move_view(self, rel_x_motion, rel_y_motion, opengl_coords=False):
        """Move the view according to the relative coordinates

        The whole view/scene is moved, causing the state machine to move within the viewport.

        :param rel_x_motion: Distance to move in x direction
        :param rel_y_motion: Distance to move in y direction
        :param opengl_coords: Whether to specified relative coordinates are in OpenGl coordinate system
        """
        if not opengl_coords:
            conversion = self.view.editor.pixel_to_size_ratio()
            rel_x_motion /= conversion
            rel_y_motion /= -conversion
            aspect = self.view.editor.allocation.width / float(self.view.editor.allocation.height)
            if aspect > 1:
                rel_x_motion /= aspect
            else:
                rel_y_motion *= aspect
        self.view.editor.left -= rel_x_motion
        self.view.editor.right -= rel_x_motion
        self.view.editor.bottom -= rel_y_motion
        self.view.editor.top -= rel_y_motion
        self._redraw()

    def _handle_zooming(self, pos, direction):
        """Zooms in or out at a given position

        The method zooms increases or decreases the viewport, resulting in a zoom effect. The zoom keeps the current
        position of the cursor within the state machine, allowing to zoom in/out in specific directions.

        :param pos:
        :param direction:
        """
        zoom_in = direction == SCROLL_DOWN
        zoom_out = direction == SCROLL_UP

        if zoom_in or zoom_out:
            old_mouse_pos = self.view.editor.screen_to_opengl_coordinates(pos)

            zoom = 1.25
            zoom = zoom if zoom_in else 1. / zoom

            # Apply centric zoom
            self.view.editor.left *= zoom
            self.view.editor.right *= zoom
            self.view.editor.bottom *= zoom
            self.view.editor.top *= zoom

            # Determine mouse offset to previous position
            aspect = self.view.editor.allocation.width / float(self.view.editor.allocation.height)
            new_mouse_pos = self.view.editor.screen_to_opengl_coordinates(pos)
            diff_x = new_mouse_pos[0] - old_mouse_pos[0]
            diff_y = new_mouse_pos[1] - old_mouse_pos[1]
            if aspect < 1:
                diff_y *= aspect
            else:
                diff_x /= aspect

            # Move view to keep the previous mouse position in the view
            self._move_view(diff_x, diff_y, opengl_coords=True)

    def draw_state_machine(self):
        """Draws remaining components of the state machine

        This method draws all other components, not directly belonging to a certain state. For a starter, this is the
        selection frame the user draws for a multi selection.
        """

        # Draw the multi selection frame
        frame = self.model.meta['gui']['editor']['selection_frame']
        if isinstance(frame, list):
            self.view.editor.draw_frame(frame[0], frame[1], 10)

    def draw_state(self, state_m, pos_x=0.0, pos_y=0.0, width=100.0, height=100.0, depth=1):
        """Draws a (container) state with all its content

        Mainly contains the logic for drawing (e. g. reading and calculating values). The actual drawing process is
        done in the view, which is called from this method with the appropriate arguments.

        :param state_m: The state to be drawn
        :param pos_x: The default x position if there is no position stored
        :param pos_y: The default y position if there is no position stored
        :param width: The default width if there is no size stored
        :param height: The default height if there is no size stored
        :param depth: The hierarchy level of the state
        """
        assert isinstance(state_m, StateModel)

        # Use default values if no size information is stored
        if not state_m.meta['gui']['editor']['width']:
            state_m.meta['gui']['editor']['width'] = width
        if not state_m.meta['gui']['editor']['height']:
            state_m.meta['gui']['editor']['height'] = height

        width = state_m.meta['gui']['editor']['width']
        height = state_m.meta['gui']['editor']['height']

        # Use default values if no size information is stored
        # Here the possible case of pos_x and posy_y == 0 must be handled
        if not state_m.meta['gui']['editor']['pos_x'] and state_m.meta['gui']['editor']['pos_x'] != 0:
            state_m.meta['gui']['editor']['pos_x'] = pos_x
        if not state_m.meta['gui']['editor']['pos_y'] and state_m.meta['gui']['editor']['pos_y'] != 0:
            state_m.meta['gui']['editor']['pos_y'] = pos_y

        pos_x = state_m.meta['gui']['editor']['pos_x']
        pos_y = state_m.meta['gui']['editor']['pos_y']

        # Was the state selected?
        selected_states = self.model.selection.get_states()
        selected = False if state_m not in selected_states else True

        # Is the state active (executing)?
        active = 0
        if state_m.state.active:
            if self.has_content(state_m) and state_m.state.child_execution:
                active = 0.5
            else:
                active = 1

        # Call the drawing method of the view
        # The view returns the id of the state in OpenGL and the positions of the outcomes, input and output ports
        (opengl_id, outcome_pos, outcome_radius, resize_length) = self.view.editor.draw_state(
            state_m.state.name,
            pos_x, pos_y, width, height,
            state_m.state.outcomes,
            state_m.input_data_ports if global_config.get_config_value('show_data_flows', True) else [],
            state_m.output_data_ports if global_config.get_config_value('show_data_flows', True) else [],
            selected, active, depth)
        state_m.meta['gui']['editor']['id'] = opengl_id
        state_m.meta['gui']['editor']['outcome_pos'] = outcome_pos
        state_m.meta['gui']['editor']['outcome_radius'] = outcome_radius
        state_m.meta['gui']['editor']['resize_length'] = resize_length

        # If the state is a container state, we also have to draw its transitions and data flows as well as
        # recursively its child states
        if self.has_content(state_m):
            state_ctr = 0
            margin = width / 25.

            for child_state in state_m.states.itervalues():
                # Calculate default positions for the child states
                # Make the inset from the top left corner
                state_ctr += 1

                child_width = width / 5.
                child_height = height / 5.

                child_pos_x = pos_x + state_ctr * margin
                child_pos_y = pos_y + height - child_height - state_ctr * margin

                self.draw_state(child_state, child_pos_x, child_pos_y, child_width, child_height,
                                depth + 1)

            if global_config.get_config_value('show_data_flows', True):
                self.draw_inner_data_ports(state_m, depth)

            self.draw_transitions(state_m, depth)

            if global_config.get_config_value('show_data_flows', True):
                self.draw_data_flows(state_m, depth)

        self._handle_new_transition(state_m, depth)

        if global_config.get_config_value('show_data_flows', True):
            self._handle_new_data_flow(state_m, depth)

    def draw_inner_data_ports(self, parent_state_m, parent_depth):
        """Draw the inner ports of a state

        This method draws the ports that are displayed within a container state. The inner ports are the input data
        ports, output data ports and scoped variables.

        :param parent_state_m: The parent state model of the ports
        :param parent_depth: The depth of the parent state
        """
        parent_info = parent_state_m.meta['gui']['editor']
        max_rows = max(20, len(parent_state_m.input_data_ports), len(parent_state_m.output_data_ports))
        port_height = min(parent_info['width'], parent_info['height']) / float(max_rows)
        max_port_width = min(parent_info['width'], parent_info['height']) / 5.

        # Input data ports
        num_input_ports = 0
        for port_m in parent_state_m.input_data_ports:
            port = port_m.data_port
            if not isinstance(port_m.meta['gui']['editor']['inner_pos'], tuple):
                pos_x = parent_info['pos_x']
                pos_y = parent_info['pos_y'] + num_input_ports * port_height
                port_m.meta['gui']['editor']['inner_pos'] = (pos_x, pos_y)
            (pos_x, pos_y) = port_m.meta['gui']['editor']['inner_pos']

            selected = port_m in self.model.selection.get_all()
            opengl_id = self.view.editor.draw_inner_input_data_port(port.name, port_m, pos_x, pos_y, max_port_width,
                                                                    port_height, selected, parent_depth + 0.5)
            port_m.meta['gui']['editor']['id'] = opengl_id
            num_input_ports += 1

        # Output data ports
        num_output_ports = 0
        for port_m in parent_state_m.output_data_ports:
            port = port_m.data_port
            if not isinstance(port_m.meta['gui']['editor']['inner_pos'], tuple):
                pos_x = parent_info['pos_x'] + parent_info['width']
                pos_y = parent_info['pos_y'] + num_output_ports * port_height
                port_m.meta['gui']['editor']['inner_pos'] = (pos_x, pos_y)
            (pos_x, pos_y) = port_m.meta['gui']['editor']['inner_pos']

            selected = port_m in self.model.selection.get_all()
            opengl_id = self.view.editor.draw_inner_output_data_port(port.name, port_m, pos_x, pos_y, max_port_width,
                                                                     port_height, selected, parent_depth + 0.5)
            port_m.meta['gui']['editor']['id'] = opengl_id
            num_output_ports += 1

        # Scoped variables
        if self.has_content(parent_state_m):
            num_scoped_variables = 0
            for port_m in parent_state_m.scoped_variables:
                port = port_m.scoped_variable
                if not isinstance(port_m.meta['gui']['editor']['inner_pos'], tuple):
                    max_cols = parent_info['width'] // max_port_width
                    (row, col) = divmod(num_scoped_variables, max_cols)
                    pos_x = parent_info['pos_x'] + col * max_port_width
                    pos_y = parent_info['pos_y'] + parent_info['height'] - port_height * (2 * row + 1)
                    port_m.meta['gui']['editor']['inner_pos'] = (pos_x, pos_y)
                (pos_x, pos_y) = port_m.meta['gui']['editor']['inner_pos']

                selected = port_m in self.model.selection.get_all()
                opengl_id = self.view.editor.draw_scoped_data_port(port.name, port_m, pos_x, pos_y, max_port_width,
                                                                   port_height, selected, parent_depth + 0.5)
                port_m.meta['gui']['editor']['id'] = opengl_id
                num_scoped_variables += 1

    def draw_transitions(self, parent_state_m, parent_depth):
        """Draws the transitions belonging to a state

        The method takes all transitions from the given state and calculates their start and end point positions.
        Those are passed together with the waypoints to the view of the graphical editor.

        :param parent_state_m: The model of the container state
        :param parent_depth: The depth of the container state
        """
        for transition_m in parent_state_m.transitions:
            # Get id and references to the from and to state
            from_state_id = transition_m.transition.from_state
            if from_state_id is None:
                from_x = parent_state_m.meta['gui']['editor']['pos_x']
                from_y = parent_state_m.meta['gui']['editor']['pos_y'] + \
                         parent_state_m.meta['gui']['editor']['height'] / 2
            else:
                from_state = parent_state_m.states[from_state_id]

                assert isinstance(from_state, StateModel), "Transition from unknown state with ID {id:s}".format(
                    id=from_state_id)

                try:
                    # Set the from coordinates to the outcome coordinates received earlier
                    from_x = parent_state_m.states[from_state_id].meta['gui']['editor']['outcome_pos'][
                        transition_m.transition.from_outcome][0]
                    from_y = parent_state_m.states[from_state_id].meta['gui']['editor']['outcome_pos'][
                        transition_m.transition.from_outcome][1]
                except Exception as e:
                    logger.error("""Outcome position was not found. \
                                maybe the outcome for the transition was not found: {err}""".format(err=e))
                    continue

            to_state_id = transition_m.transition.to_state
            to_state = None if to_state_id is None else parent_state_m.states[to_state_id]

            if to_state is None:  # Transition goes back to parent
                # Set the to coordinates to the outcome coordinates received earlier
                to_x = parent_state_m.meta['gui']['editor']['outcome_pos'][transition_m.transition.to_outcome][0]
                to_y = parent_state_m.meta['gui']['editor']['outcome_pos'][transition_m.transition.to_outcome][1]
            else:
                # Set the to coordinates to the center of the next state
                to_x = to_state.meta['gui']['editor']['pos_x']
                to_y = to_state.meta['gui']['editor']['pos_y'] + to_state.meta['gui']['editor']['height'] / 2

            waypoints = []
            for waypoint in transition_m.meta['gui']['editor']['waypoints']:
                waypoints.append((waypoint[0], waypoint[1]))

            # Let the view draw the transition and store the returned OpenGl object id
            selected = False
            if transition_m in self.model.selection.get_transitions():
                selected = True
            line_width = min(parent_state_m.meta['gui']['editor']['width'],
                             parent_state_m.meta['gui']['editor']['height']) / 25.0
            opengl_id = self.view.editor.draw_transition(from_x, from_y, to_x, to_y, line_width, waypoints,
                                                         selected, parent_depth + 0.5)
            transition_m.meta['gui']['editor']['id'] = opengl_id
            transition_m.meta['gui']['editor']['from_pos_x'] = from_x
            transition_m.meta['gui']['editor']['from_pos_y'] = from_y
            transition_m.meta['gui']['editor']['to_pos_x'] = to_x
            transition_m.meta['gui']['editor']['to_pos_y'] = to_y

    def draw_data_flows(self, parent_state_m, parent_depth):
        """Draw all data flows contained in the given container state

        The method takes all data flows from the given state and calculates their start and end point positions.
        Those are passed together with the waypoints to the view of the graphical editor.

        :param parent_state_m: The model of the container state
        :param parent_depth: The depth pf the container state
        """
        for data_flow_m in parent_state_m.data_flows:
            # Get id and references to the from and to state
            from_state_id = data_flow_m.data_flow.from_state
            to_state_id = data_flow_m.data_flow.to_state
            from_state = parent_state_m if from_state_id == parent_state_m.state.state_id else parent_state_m.states[
                from_state_id]
            to_state = parent_state_m if to_state_id == parent_state_m.state.state_id else parent_state_m.states[
                to_state_id]

            from_key = data_flow_m.data_flow.from_key
            to_key = data_flow_m.data_flow.to_key

            from_port = StateMachineHelper.get_data_port_model(from_state, from_key)
            to_port = StateMachineHelper.get_data_port_model(to_state, to_key)

            if from_port is None:
                logger.warn('Cannot find model of the from data port {0}'.format(from_key))
                continue
            if to_port is None:
                logger.warn('Cannot find model of the to data port {0}'.format(to_key))
                continue

            # For scoped variables, there is no inner and outer connector
            if isinstance(from_port, ScopedVariableModel):
                (from_x, from_y) = from_port.meta['gui']['editor']['connector_pos']
            elif from_state_id == parent_state_m.state.state_id:  # The data flow is connected to the parents input
                (from_x, from_y) = from_port.meta['gui']['editor']['inner_connector_pos']
            else:
                (from_x, from_y) = from_port.meta['gui']['editor']['outer_connector_pos']
            if isinstance(to_port, ScopedVariableModel):
                (to_x, to_y) = to_port.meta['gui']['editor']['connector_pos']
            elif to_state_id == parent_state_m.state.state_id:  # The data flow is connected to the parents output
                (to_x, to_y) = to_port.meta['gui']['editor']['inner_connector_pos']
            else:
                (to_x, to_y) = to_port.meta['gui']['editor']['outer_connector_pos']

            waypoints = []
            for waypoint in data_flow_m.meta['gui']['editor']['waypoints']:
                waypoints.append((waypoint[0], waypoint[1]))

            selected = False
            if data_flow_m in self.model.selection.get_data_flows():
                selected = True
            line_width = min(parent_state_m.meta['gui']['editor']['width'],
                             parent_state_m.meta['gui']['editor']['height']) / 25.0
            opengl_id = self.view.editor.draw_data_flow(from_x, from_y, to_x, to_y, line_width, waypoints,
                                                        selected, parent_depth + 0.5)
            data_flow_m.meta['gui']['editor']['id'] = opengl_id
            data_flow_m.meta['gui']['editor']['from_pos_x'] = from_x
            data_flow_m.meta['gui']['editor']['from_pos_y'] = from_y
            data_flow_m.meta['gui']['editor']['to_pos_x'] = to_x
            data_flow_m.meta['gui']['editor']['to_pos_y'] = to_y

    def _handle_new_waypoint(self):
        """Creates waypoints during the creation of transitions and data flows

        This method creates temporary waypoints while the uses uses drag and drop to create new data flows or
        transitions. The method checks, whether such a connection is currently being created and whether the mouse
        cursor is within the parent state of the new connection.
        """
        if self.selected_outcome is not None:
            if self.selected_outcome[1] is not None:
                parent_state_m = self.selected_outcome[0].parent
            else:
                parent_state_m = self.selected_outcome[0]
        elif self.selected_port_connector:
            parent_state_m = self.selection.parent if self.selected_port_type != "outer" else \
                self.selection.parent.parent
        else:
            return
        restricted_click = self._limit_position_to_state(parent_state_m,
                                                         self.mouse_move_last_coords[0], self.mouse_move_last_coords[1])
        # If the user clicked with the parent state of the selected outcome state
        if restricted_click[0] == self.mouse_move_last_coords[0] and \
                        restricted_click[1] == self.mouse_move_last_coords[1]:
            self.temporary_waypoints.append(restricted_click)

    def _handle_new_transition(self, parent_state_m, parent_depth):
        """Responsible for drawing new transition the user creates

        With drag and drop on outcomes, the user can draw new transitions. Here the transition is temporary drawn in
        the graphical editor.

        :param parent_state_m: Model of the container state
        :param parent_depth: Depth of the container state
        """
        if self.selected_outcome is not None:  # and self.last_button_pressed == 1:
            # self.selected_outcome[0] references the state model of the selected outcome
            if self.selected_outcome[0] == parent_state_m:
                # self.selected_outcome[1] stores the id of the outcome
                # if the outcome id is None, the transition starts at an income
                if self.selected_outcome[1] is None:
                    origin = (parent_state_m.meta['gui']['editor']['pos_x'],
                              parent_state_m.meta['gui']['editor']['pos_y'] +
                              parent_state_m.meta['gui']['editor']['height'] / 2)
                else:
                    outcome = parent_state_m.meta['gui']['editor']['outcome_pos'][self.selected_outcome[1]]
                    origin = outcome
                cur = self.mouse_move_last_coords
                target = self._limit_position_to_state(parent_state_m.parent, cur[0], cur[1])
                parent_state_m = parent_state_m if parent_state_m.parent is None else parent_state_m.parent
                line_width = min(parent_state_m.meta['gui']['editor']['width'],
                                 parent_state_m.meta['gui']['editor'][
                                     'height']) / 25.0
                self.view.editor.draw_transition(origin[0], origin[1], target[0], target[1], line_width,
                                                 self.temporary_waypoints, True, parent_depth + 0.6)

    def _handle_new_data_flow(self, parent_state_m, parent_depth):
        """Responsible for drawing new data flows the user creates

        With drag and drop on ports, the user can draw new data flows. Here the data flow is temporary drawn in the
        graphical editor.

        :param parent_state_m: Model of the container state
        :param parent_depth: Depth of the container state
        """
        if self.selected_port_connector:  # and self.last_button_pressed == 1:
            port_m = self.selection
            if (port_m.parent == parent_state_m and self.selected_port_type in ("inner", "scope")) or \
                    (port_m.parent.parent == parent_state_m and self.selected_port_type == "outer"):
                if self.selected_port_type == "inner":
                    connector = port_m.meta['gui']['editor']['inner_connector_pos']
                elif self.selected_port_type == "outer":
                    connector = port_m.meta['gui']['editor']['outer_connector_pos']
                else:  # scoped variable
                    connector = port_m.meta['gui']['editor']['connector_pos']
                cur = self.mouse_move_last_coords
                target = self._limit_position_to_state(parent_state_m, cur[0], cur[1])
                ref_state = parent_state_m
                line_width = min(ref_state.meta['gui']['editor']['width'],
                                 ref_state.meta['gui']['editor']['height']) / 25.0
                self.view.editor.draw_data_flow(connector[0], connector[1], target[0], target[1], line_width,
                                                self.temporary_waypoints, True, parent_depth + 0.6)

    def _find_selection(self, pos_x, pos_y, width=6, height=6, all=False,
                        find_states=True, find_transitions=True, find_data_flows=True, find_data_ports=True):
        """Returns the model at the given position

        This method is used when the model (state/transition/data flow) the user clicked on is to be found. The
        position is told to OpenGl and the whole scene is redrawn. From the stack ob objects beneath the position,
        the uppermost one is returned.

        :param pos_x: The x coordinate of the position
        :param pos_y: The y coordinate of the position
        :param find_states: Flag whether to find states
        :param find_transitions: Flag whether to find transitions
        :param find_data_flows: Flag whether to find data flows
        :param find_data_ports: Flag whether to find data ports
        :return: The uppermost model beneath the given position, None if nothing was found
        """
        # e.g. sets render mode to GL_SELECT
        self.view.editor.prepare_selection(pos_x, pos_y, width, height)
        # draw again
        self.view.editor.expose_init()
        self.draw_state(self.root_state_m)
        self.view.editor.expose_finish()
        # get result
        hits = self.view.editor.find_selection()

        # extract ids
        selection = None

        def get_id(hit):
            if len(hit[2]) > 1:
                return hit[2][1]
            return None

        try:
            selected_ids = map(get_id, hits)  # Get the OpenGL ids for the hits
            selected_ids = filter(lambda opengl_id: opengl_id is not None, selected_ids)  # Filter out Nones
            (selection, selection_depth) = self._selection_ids_to_model(selected_ids, self.root_state_m, 1, None, 0,
                                                                        all,
                                                                        find_states, find_transitions,
                                                                        find_data_flows, find_data_ports)
        except Exception as e:
            logger.error("Error while finding selection: {err:s}".format(err=e))
            pass
        return selection

    def _selection_ids_to_model(self, ids, search_state, search_state_depth, selection, selection_depth, all=False,
                                find_states=True, find_transitions=True, find_data_flows=True, find_data_ports=True):
        """Searches recursively for objects with the given ids

        The method searches recursively and compares all stored ids with the given ones. It finally returns the
        object with the biggest depth (furthest nested).

        :param ids: The ids to search for
        :param search_state: The state to search in
        :param search_state_depth: The depth the search state is in
        :param selection: The currently found object
        :param selection_depth: The depth of the currently found object
        :return: The selected object and its depth
        """

        def update_selection(selection, model):
            if all:
                if selection is None:
                    return [model]
                elif not isinstance(selection, list):
                    return [selection, model]
                else:
                    selection.append(model)
                    return selection
            else:
                return model

        # Only the element which is furthest down in the hierarchy is selected
        if (search_state_depth > selection_depth or all) and find_states:
            # Check whether the id of the current state matches an id in the selected ids
            if search_state.meta['gui']['editor']['id'] and search_state.meta['gui']['editor']['id'] in ids:
                # if so, add the state to the list of selected states
                selection = update_selection(selection, search_state)
                selection_depth = search_state_depth
                # remove the id from the list to fasten up further searches
                ids.remove(search_state.meta['gui']['editor']['id'])

        # Return if there is nothing more to find
        if len(ids) == 0:
            return selection, selection_depth

        # If it is a container state, check its transitions, data flows and child states
        if self.has_content(search_state):

            for state in search_state.states.itervalues():
                if len(ids) > 0:
                    (selection, selection_depth) = self._selection_ids_to_model(ids, state, search_state_depth + 1,
                                                                                selection, selection_depth, all,
                                                                                find_states, find_transitions,
                                                                                find_data_flows, find_data_ports)

            if len(ids) == 0 or (search_state_depth < selection_depth and not all):
                return selection, selection_depth

            def search_selection_in_model_list(model_list, current_selection):
                for model in model_list:
                    if model.meta['gui']['editor']['id'] and model.meta['gui']['editor']['id'] in ids:
                        ids.remove(model.meta['gui']['editor']['id'])
                        current_selection = update_selection(current_selection, model)
                return current_selection

            if find_transitions:
                selection = search_selection_in_model_list(search_state.transitions, selection)
                if len(ids) == 0:
                    return selection, selection_depth

            if find_data_flows:
                selection = search_selection_in_model_list(search_state.data_flows, selection)
                if len(ids) == 0:
                    return selection, selection_depth

            if find_data_ports:
                selection = search_selection_in_model_list(search_state.input_data_ports, selection)
                selection = search_selection_in_model_list(search_state.output_data_ports, selection)
                selection = search_selection_in_model_list(search_state.scoped_variables, selection)
        return selection, selection_depth

    @staticmethod
    def has_content(state_m):
        if isinstance(state_m, ContainerStateModel) and state_m.state.state_type != StateType.LIBRARY:
            return True
        return False

    @staticmethod
    def get_boundaries(model, include_waypoints=False):
        """Returns the boundaries (in OpenGL) coordinates of the given model

        :param model: Model for which to get the boundaries
        :return: left, right, top, bottom
        """
        meta = model.meta['gui']['editor']
        if isinstance(model, StateModel):
            return meta['pos_x'], meta['pos_x'] + meta['width'], meta['pos_y'], meta['pos_y'] + meta['height']
        if isinstance(model, (TransitionModel, DataFlowModel)):
            x_coordinates = [meta['from_pos_x'], meta['to_pos_x']]
            y_coordinates = [meta['from_pos_y'], meta['to_pos_y']]
            if include_waypoints:
                for waypoint in meta['waypoints']:
                    x_coordinates.append(waypoint[0])
                    y_coordinates.append(waypoint[1])
            return min(x_coordinates), max(x_coordinates), min(y_coordinates), max(y_coordinates)

        if isinstance(model, (DataPortModel, ScopedVariableModel)):
            left = meta['inner_pos'][0]
            right = meta['inner_pos'][0] + meta['width']
            bottom = meta['inner_pos'][1]
            top = meta['inner_pos'][1] + meta['height']

            if model in model.parent.output_data_ports:
                left = meta['inner_pos'][0] - meta['width']
                right = meta['inner_pos'][0]
            if model in model.parent.scoped_variables:
                bottom = meta['inner_pos'][1] - meta['height'] + meta['rect_height']
                top = bottom + meta['height']

            return left, right, bottom, top
        return None, None, None, None

    def _publish_changes(self, model, name="Graphical Editor", affects_children=False):
        self.model.state_machine.marked_dirty = True
        # History.meta_changed_notify_after(self, model, name, affects_children)
        pass

    def _delete_selection(self, *args):
        if self.view.editor.has_focus():
            selection = self.model.selection.get_all()
            if len(selection) > 0:
                StateMachineHelper.delete_models(self.model.selection.get_all())
                self.model.selection.clear()

    def _add_execution_state(self, *args):
        if self.view.editor.has_focus():  # or singleton.global_focus is self:
            selection = self.model.selection.get_all()
            if len(selection) > 0:
                model = selection[0]

                if isinstance(model, StateModel):
                    StateMachineHelper.add_state(model, StateType.EXECUTION)
                if isinstance(model, TransitionModel) or isinstance(model, DataFlowModel):
                    StateMachineHelper.add_state(model.parent, StateType.EXECUTION)

    def _toggle_data_flow_visibility(self, *args):
        if self.view.editor.has_focus():
            global_config.set_config_value('show_data_flows', not global_config.get_config_value("show_data_flows"))
            self._redraw()

    def _abort(self, *args):
        if self.view.editor.has_focus():
            if self.mouse_move_redraw:
                if self.selected_outcome is not None:
                    self.selected_outcome = None
                elif self.selected_port_connector:
                    self.selected_port_connector = False
                self.mouse_move_redraw = False
                self.temporary_waypoints = []
                self._redraw()

    def _copy_selection(self, *args):
        """
        Copies the current selection to the clipboard.
        :return:
        """
        if self.view.editor.has_focus():
            logger.debug("copy selection")
            global_clipboard.copy(self.model.selection)

    def _cut_selection(self, *args):
        """
        Cuts the current selection and copys it to the clipboard.
        :return:
        """
        if self.view.editor.has_focus():
            logger.debug("cut selection")
            global_clipboard.cut(self.model.selection)

    def _paste_clipboard(self, *args):
        """
        Paste the current clipboard into the current selection if the current selection is a container state.
        :return:
        """
        if self.view.editor.has_focus():
            logger.debug("paste selection")
            current_selection = self.model.selection
            # check if the current selection is valid
            if current_selection.get_number_of_selected_items() > 1 or len(current_selection.get_states()) < 1:
                logger.error("Cannot paste clipboard into selection as the selection does not consist of a single "
                             "container state!")
                return
            if len(current_selection.get_states()) == 1 and \
                    not isinstance(current_selection.get_states()[0], ContainerStateModel):
                logger.error("Cannot paste clipboard into selection as the selected state model is not "
                             "a container state model")
                return

            # check if the clipboard is valid
            if global_clipboard.get_number_selected_items() > 1:
                logger.error("Only one single item is allowed to be copied yet!")
                return
            if not len(global_clipboard.get_states()) == 1:
                logger.error("Only states are allowed to be copied yet!")
                return

            # Note: in multi-selection case, a loop over all selected items is necessary instead of the 0 index
            target_state_model = current_selection.get_states()[0]
            target_state = target_state_model.state

            # copy core state
            state_copy = StateHelper.get_state_copy(global_clipboard.state_core_object_copies[0])
            state_copy.change_state_id()
            target_state.add_state(state_copy)

            # copy meta data
            state_copy_model = target_state_model.states[state_copy.state_id]
            state_copy_model.copy_meta_data_from_state_model(global_clipboard.state_model_copies[0])
            new_x_pos = target_state_model.meta["gui"]["editor"]["pos_x"] + \
                        target_state_model.meta["gui"]["editor"]["width"] * 3 / 100
            new_y_pos = target_state_model.meta["gui"]["editor"]["pos_y"] + \
                        target_state_model.meta["gui"]["editor"]["height"] * 97 / 100 - \
                        state_copy_model.meta["gui"]["editor"]["height"]

            self._move_state(state_copy_model, new_x_pos, new_y_pos)
            self._redraw()

            if global_clipboard.clipboard_type is ClipboardType.COPY:
                pass
            elif global_clipboard.clipboard_type is ClipboardType.CUT:
                # delete the original state
                # Note: change this when implementing multi selection
                if len(global_clipboard.selected_state_models) == 1:
                    source_state_id = global_clipboard.selected_state_models[0].state.state_id
                    parent_of_source_state = global_clipboard.selected_state_models[0].state.parent
                    parent_of_source_state.remove_state(source_state_id)
                    global_clipboard.selected_state_models.remove(global_clipboard.selected_state_models[0])
