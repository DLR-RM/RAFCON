from awesome_tool.utils.geometry import point_in_triangle, dist, point_on_line, deg2rad
from awesome_tool.utils import log
logger = log.get_logger(__name__)

import sys
import time

from gtk.gdk import SCROLL_DOWN, SCROLL_UP, SHIFT_MASK, CONTROL_MASK, BUTTON1_MASK, BUTTON2_MASK, BUTTON3_MASK
from gtk.gdk import keyval_name
import gobject
import itertools
from copy import copy
from functools import partial

from math import sin, cos, atan2
from awesome_tool.mvc.config import global_gui_config
from awesome_tool.statemachine.enums import StateType
from awesome_tool.mvc.clipboard import global_clipboard
from awesome_tool.mvc.statemachine_helper import StateMachineHelper
from awesome_tool.mvc.controllers.extended_controller import ExtendedController
from awesome_tool.mvc.models import ContainerStateModel, StateModel, TransitionModel, DataFlowModel
from awesome_tool.mvc.models.state_machine import StateMachineModel
from awesome_tool.mvc.models.scoped_variable import ScopedVariableModel
from awesome_tool.mvc.models.data_port import DataPortModel
from awesome_tool.mvc.views.graphical_editor import Direction


def check_pos(pos):
    if not isinstance(pos, tuple):
        raise ValueError("Position must be of type tuple")
    if len(pos) != 2:
        raise ValueError("Position must have exactly two entries (x and y)")


def add_pos(pos1, pos2):
    check_pos(pos1)
    check_pos(pos2)
    return pos1[0] + pos2[0], pos1[1] + pos2[1]


def subtract_pos(pos1, pos2):
    check_pos(pos1)
    check_pos(pos2)
    return pos1[0] - pos2[0], pos1[1] - pos2[1]


def pos_equal(pos1, pos2):
    check_pos(pos1)
    check_pos(pos2)
    return pos1[0] == pos2[0] and pos1[1] == pos2[1]


class GraphicalEditorController(ExtendedController):
    """Controller handling the graphical editor

    :param awesome_tool.mvc.models.state_machine.StateMachineModel model: The state machine model, holding the root
        state and the current selection
    :param awesome_tool.mvc.views.graphical_editor.GraphicalEditorView view: The GTK view having an OpenGL rendering
        element
    """

    def __init__(self, model, view):
        """Constructor
        """
        assert isinstance(model, StateMachineModel)
        ExtendedController.__init__(self, model, view)
        self.root_state_m = model.root_state

        self.timer_id = None

        self.single_selection = None
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

        shortcut_manager.add_callback_for_action("left", partial(self._move_in_direction, Direction.left))
        shortcut_manager.add_callback_for_action("right", partial(self._move_in_direction, Direction.right))
        shortcut_manager.add_callback_for_action("up", partial(self._move_in_direction, Direction.top))
        shortcut_manager.add_callback_for_action("down", partial(self._move_in_direction, Direction.bottom))

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
            self._redraw()

    @ExtendedController.observe("root_state", assign=True)
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
        if self.single_selection != selection:
            self.single_selection = selection
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

        if hasattr(self.view, "editor") and (time.time() - self.last_time > redraw_after) and \
                        self.model.sm_manager_model.selected_state_machine_id == self.model.state_machine.state_machine_id:
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
        self.selected_resizer = None  # reset
        self.multi_selection_started = False  # reset

        # Store the coordinates of the event
        self.mouse_move_start_pos = (event.x, event.y)
        self.mouse_move_last_pos = (event.x, event.y)
        self.mouse_move_start_coords = self.view.editor.screen_to_opengl_coordinates((event.x, event.y))
        self.mouse_move_last_coords = self.view.editor.screen_to_opengl_coordinates((event.x, event.y))

        # Check if something was selected
        new_selection = self._find_selection(event.x, event.y)

        # Check, whether a resizer was clicked on
        self._check_for_resizer_selection(new_selection, self.mouse_move_start_coords)

        # Check, whether a waypoint was clicked on
        self._check_for_waypoint_selection(new_selection, self.mouse_move_start_coords)

        # Left mouse button was clicked and no multi selection intended
        if event.button == 1 and event.state & SHIFT_MASK == 0:
            if not self.mouse_move_redraw:
                self.single_selection = new_selection

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
            if global_gui_config.get_config_value('show_data_flows', True):
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
                elif self.selected_port_connector and isinstance(new_selection, StateModel):
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

        # Check if something was selected
        new_selection = self._find_selection(event.x, event.y)

        if event.button == 1:
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
                # Only do something, if the user didn't click the second time on a specific model
                elif not self.model.selection.is_selected(new_selection):
                    self.model.selection.clear()
                    self.model.selection.set(new_selection)

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
            delta_pos = subtract_pos((event.x, event.y), self.mouse_move_last_pos)
            self._move_view(delta_pos)

        mouse_current_coord = self.view.editor.screen_to_opengl_coordinates((event.x, event.y))

        if self.multi_selection_started:
            self._draw_multi_selection_frame()
            self.mouse_move_last_pos = (event.x, event.y)
            self.mouse_move_last_coords = mouse_current_coord
            return

        # Move the selected states and data ports
        if len(self.model.selection) > 1 and \
                self.last_button_pressed == 1 and \
                self.selected_outcome is None and self.selected_resizer is None:
            if self.drag_origin_offset is None:
                self.drag_origin_offset = []
                for model in self.model.selection:
                    offset = None
                    original_position = None
                    if isinstance(model, StateModel):
                        offset = self._get_position_relative_to_state(model, self.mouse_move_start_coords)
                        model.temp['gui']['editor']['original_rel_pos'] = copy(model.meta['gui']['editor']['rel_pos'])
                    elif isinstance(model, (DataPortModel, ScopedVariableModel)):
                        offset = subtract_pos(self.mouse_move_start_coords, model.temp['gui']['editor']['inner_pos'])
                        model.temp['gui']['editor']['original_inner_rel_pos'] = \
                            copy(model.meta['gui']['editor']['inner_rel_pos'])
                    self.drag_origin_offset.append(offset)
            for i, model in enumerate(self.model.selection):
                if self.drag_origin_offset[i] is not None:
                    new_pos = subtract_pos(mouse_current_coord, self.drag_origin_offset[i])
                    if isinstance(model, StateModel):
                        self._move_state(model, new_pos)
                    elif isinstance(model, (DataPortModel, ScopedVariableModel)):
                        self._move_data_port(model, new_pos)

        # Move the current state
        elif isinstance(self.single_selection, StateModel) and \
                self.last_button_pressed == 1 and \
                self.selected_outcome is None and self.selected_resizer is None:
            if self.drag_origin_offset is None:
                offset = self._get_position_relative_to_state(self.single_selection, self.mouse_move_start_coords)
                self.drag_origin_offset = offset
                self.single_selection.temp['gui']['editor']['original_rel_pos'] = \
                    copy(self.single_selection.meta['gui']['editor']['rel_pos'])
            new_pos = subtract_pos(mouse_current_coord, self.drag_origin_offset)
            self._move_state(self.single_selection, new_pos)

        # Move current data port
        elif isinstance(self.single_selection, (DataPortModel, ScopedVariableModel)) and \
                not self.selected_port_connector and self.last_button_pressed == 1:
            if self.drag_origin_offset is None:
                self.drag_origin_offset = subtract_pos(self.mouse_move_start_coords,
                                                       self.single_selection.temp['gui']['editor']['inner_pos'])
                self.single_selection.temp['gui']['editor']['original_inner_rel_pos'] = \
                    copy(self.single_selection.meta['gui']['editor']['inner_rel_pos'])
            new_pos = subtract_pos(mouse_current_coord, self.drag_origin_offset)
            self._move_data_port(self.single_selection, new_pos)

        # Move the selected waypoint (if there is one)
        if self.selected_waypoint is not None:
            # Move selected waypoint within its container state
            self._move_waypoint(mouse_current_coord, event.state)

        # Redraw to show the new transition/data flow the user is creating with drag and drop
        if self.selected_outcome is not None or self.selected_port_connector:
            self._redraw()

        if self.selected_resizer is not None:
            state_m = self.selected_resizer
            if self.drag_origin_offset is None:
                lower_right_corner = (state_m.temp['gui']['editor']['pos'][0] +
                                      state_m.meta['gui']['editor']['size'][0],
                                      state_m.temp['gui']['editor']['pos'][1] -
                                      state_m.meta['gui']['editor']['size'][1])
                self.drag_origin_offset = subtract_pos(self.mouse_move_start_coords, lower_right_corner)
            new_pos = subtract_pos(mouse_current_coord, self.drag_origin_offset)
            modifier = event.state
            self._resize_state(state_m, new_pos, modifier)

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
    def _limit_position_to_state(state_m, pos, child_size=(0, 0)):
        pos_x, pos_y = pos
        if state_m is not None:
            state_meta = state_m.meta['gui']['editor']
            state_temp = state_m.temp['gui']['editor']
            if pos_x < state_temp['pos'][0]:
                pos_x = state_temp['pos'][0]
            elif pos_x + child_size[0] > state_temp['pos'][0] + state_meta['size'][0]:
                pos_x = max(state_temp['pos'][0], state_temp['pos'][0] + state_meta['size'][0] - child_size[0])

            if pos_y > state_temp['pos'][1]:
                pos_y = state_temp['pos'][1]
            elif pos_y - child_size[1] < state_temp['pos'][1] - state_meta['size'][1]:
                pos_y = min(state_temp['pos'][1], state_temp['pos'][1] - state_meta['size'][1] + child_size[1])
        return pos_x, pos_y

    def _check_for_waypoint_selection(self, selection, coords):
        """Check whether a waypoint was clicked on

        Checks whether the current selection is a transition or data flow and if so looks for a waypoint at the given
        coordinates. If a waypoint is found, it is stored together with its current position.

        :param coords: Coordinates to search for waypoints
        """
        if isinstance(selection, (TransitionModel, DataFlowModel)):
            connection_m = selection
            parent_state_m = connection_m.parent
            close_threshold = min(parent_state_m.meta['gui']['editor']['size']) / 50.
            # Check distance between all waypoints of the selected transition/data flows and the given coordinates
            for i, waypoint in enumerate(connection_m.meta['gui']['editor']['waypoints']):
                waypoint_pos = self._get_absolute_position(parent_state_m, waypoint)
                # Only if coordinates are stored for the waypoints (always should be the case)
                if dist(waypoint_pos, coords) < close_threshold:
                    # As tuples cannot be changed, we have to store the whole list plus the index
                    self.selected_waypoint = (connection_m, i)
                    break

    @staticmethod
    def _check_for_outcome_selection(selection, coords):
        """Check whether a port was clicked on

        Checks whether the current selection is a state and if so looks for an outcome at the given coordinates. If an
        outcome is found, it is stored.

        :param coords: Coordinates to search for outcomes
        """
        if isinstance(selection, StateModel):  # and self.single_selection is not self.root_state_m:
            state_m = selection
            outcomes_close_threshold = state_m.temp['gui']['editor']['outcome_radius']
            outcomes = state_m.temp['gui']['editor']['outcome_pos']
            # Check distance between all outcomes of the selected state and the given coordinate
            for key in outcomes:
                if dist(outcomes[key], coords) < outcomes_close_threshold:
                    return state_m, key
            income_pos = state_m.temp['gui']['editor']['income_pos']
            if dist(income_pos, coords) < outcomes_close_threshold:
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
            connector_pos = model.temp['gui']['editor'][prefix + 'connector_pos']
            connector_radius = model.temp['gui']['editor'][prefix + 'connector_radius']
            if dist(connector_pos, coords) < connector_radius:
                selected_port_type = "inner" if isinstance(model, DataPortModel) else "scope"
                return model, selected_port_type, True

        elif isinstance(model, StateModel):
            state_m = model

            for port_m in itertools.chain(state_m.input_data_ports, state_m.output_data_ports):
                connector_pos = port_m.temp['gui']['editor']['outer_connector_pos']
                connector_radius = port_m.temp['gui']['editor']['outer_connector_radius']
                if dist(connector_pos, coords) < connector_radius:
                    return port_m, "outer", True
        return None, None, False

    def _check_for_resizer_selection(self, selection, coords):
        """Check whether a resizer (handle to resize a state) was clicked on

        Checks whether the current selection is a state and if so looks the given coordinates are within the resizer
        of that state. If so, the resizer (or its state model) is stored.

        :param coords: Coordinates to check for the resizer
        """
        if isinstance(selection, StateModel):
            state_meta = selection.meta['gui']['editor']
            state_temp = selection.temp['gui']['editor']
            # Calculate corner points of resizer
            p1 = (state_temp['pos'][0] + state_meta['size'][0], state_temp['pos'][1] - state_meta['size'][1])
            p2 = (p1[0] - state_temp['resize_length'], p1[1])
            p3 = (p1[0], p1[1] + state_temp['resize_length'])

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
        self.model.temp['gui']['editor']['selection_frame'] = None

    def _check_for_waypoint_removal(self, coords, connection_model):
        """Checks and removes a waypoint if necessary

        Checks whether the coordinates given are close to a waypoint of the given connection model (transition or
        data flow). If so, the waypoint is removed.

        :param coords: Coordinates to check for a waypoint
        :param connection_model: Model of a transition or data flow
        :return: True, if a waypoint was removed, False else
        """
        close_threshold = min(connection_model.parent.meta['gui']['editor']['size']) / 70.
        # Check distance between all waypoints of the connection to the given coordinates
        for waypoint in connection_model.meta['gui']['editor']['waypoints']:
            waypoint_pos = self._get_absolute_position(connection_model.parent, waypoint)
            if dist(waypoint_pos, coords) < close_threshold:
                connection_model.meta['gui']['editor']['waypoints'].remove(waypoint)
                logger.debug('Connection waypoint removed')
                self._redraw()
                return True
        return False

    def _draw_multi_selection_frame(self):
        corner1 = self.mouse_move_start_coords
        corner2 = self.mouse_move_last_coords
        self.model.temp['gui']['editor']['selection_frame'] = [corner1, corner2]
        self._redraw()


    def _add_waypoint(self, connection_m, coords):
        """Adds a waypoint to the given connection

        The methods adds a waypoint at the given coordinates to the given connection (transition or data flow). If
        the connection also has waypoints, it puts the new one between the correct existing ones.

        :param connection_m: The model of the connection to add a waypoint to
        :param coords: The coordinates of the new waypoint
        """

        connection_temp = connection_m.temp['gui']['editor']
        parent_state_m = connection_m.parent
        # The waypoints should exist as dictionary. If not (for any reason), we have to convert it to one
        if isinstance(connection_m.meta['gui']['editor']['waypoints'], dict):
            # logger.warn("Connection waypoints was of type dict, expected list")
            # connection_m.meta['gui']['editor']['waypoints'] = connection_m.meta['waypoints'].items()
            connection_m.meta['gui']['editor']['waypoints'] = []
        waypoint_list = connection_m.meta['gui']['editor']['waypoints']

        # Create a list of all connection points, consisting of start, waypoints and end
        points = [self._get_position_relative_to_state(parent_state_m, connection_temp['from_pos'])]
        points.extend(waypoint_list)
        points.append(self._get_position_relative_to_state(parent_state_m, connection_temp['to_pos']))

        rel_coords = self._get_position_relative_to_state(parent_state_m, coords)

        # size = self.view.editor.get_size()
        # max_tolerance = max(size) / 50.

        # Insert the waypoint at the correct position
        for i in range(len(points) - 1):
            if point_on_line(rel_coords, points[i], points[i + 1]):
                waypoint_list.insert(i, rel_coords)
        logger.debug('Connection waypoint added at rel pos {0:.1f} | {1:.1f} (abs pos {0:.1f} | {1:.1f})'.format(
            rel_coords[0], rel_coords[1], coords[0], coords[1]))
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
            from_port_m = self.single_selection
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

    def _move_state(self, state_m, new_pos):
        """Move the state to the given position

        The method moves the state and all its child states with their transitions, data flows and waypoints. The
        state is kept within its parent, thus restricting the movement.

        :param awesome_tool.mvc.models.StateModel state_m: The model of the state to be moved
        :param new_pos: The desired new position (x, y)
        """

        if state_m.parent is None:
            return

        cur_size = state_m.meta['gui']['editor']['size']

        # Keep the state within its container state
        new_pos = self._limit_position_to_state(state_m.parent, new_pos, cur_size)

        parent_pos = state_m.parent.temp['gui']['editor']['pos']
        new_rel_pos = subtract_pos(new_pos, parent_pos)

        state_m.meta['gui']['editor']['rel_pos'] = new_rel_pos

        self._publish_changes(state_m, "Move state", affects_children=True)
        self._redraw()

    def _move_data_port(self, port_m, new_pos):
        """Move the port to the given position

        This method moves the given port to the given coordinates, with respect to the mouse offset to the origin od
        the port and with respect to the size of the container state.

        :param port_m: The port model to be moved
        :param coords: The target position
        """

        left, right, bottom, top = self.get_boundaries(port_m)
        size = (right - left, top - bottom)

        # Keep the state within its container state
        if port_m in port_m.parent.output_data_ports:
            # Origin of output data ports is top right corner
            new_pos = (new_pos[0] - right + left, new_pos[1])
            new_pos = self._limit_position_to_state(port_m.parent, new_pos, size)
            new_pos = (new_pos[0] + right - left, new_pos[1])
        else:
            new_pos = self._limit_position_to_state(port_m.parent, new_pos, size)

        parent_pos = port_m.parent.temp['gui']['editor']['pos']
        new_rel_pos = subtract_pos(new_pos, parent_pos)

        port_m.meta['gui']['editor']['inner_rel_pos'] = new_rel_pos

        self._publish_changes(port_m.parent, "Move data port", affects_children=False)
        self._redraw()

    def _move_in_direction(self, direction, key, modifier):
        """Move the current selection into the given direction

        The method is the callback handler for arrow key presses. It moves the current selection into the direction
        of the pressed arrow key.

        :param awesome_tool.mvc.views.graphical_editor.Direction direction: direction into which to move
        :param key: Pressed key
        :param modifier: Pressed modifier key
        """
        def move_pos(pos, parent_size):
            scale_dist = 0.025
            if modifier & SHIFT_MASK:
                scale_dist = 0.125
            if direction == Direction.left:
                return pos[0] - parent_size[0] * scale_dist, pos[1]
            elif direction == Direction.right:
                return pos[0] + parent_size[0] * scale_dist, pos[1]
            elif direction == Direction.top:
                return pos[0], pos[1] + parent_size[1] * scale_dist
            elif direction == Direction.bottom:
                return pos[0], pos[1] - parent_size[1] * scale_dist
            return pos

        def move_state(state_m):
            if state_m.parent is None:
                return
            cur_pos = state_m.temp['gui']['editor']['pos']
            new_pos = move_pos(cur_pos, state_m.parent.meta['gui']['editor']['size'])
            self._move_state(state_m, new_pos)

        def move_port(port_m):
            cur_pos = port_m.temp['gui']['editor']['inner_pos']
            new_pos = move_pos(cur_pos, port_m.parent.meta['gui']['editor']['size'])
            self._move_data_port(port_m, new_pos)

        if self.view.editor.has_focus():
            if len(self.model.selection) > 0:
                for model in self.model.selection:
                    if isinstance(model, StateModel):
                        move_state(model)
                    elif isinstance(model, (DataPortModel, ScopedVariableModel)):
                        move_port(model)
            elif isinstance(self.single_selection, StateModel):
                move_state(self.single_selection)
            elif isinstance(self.single_selection, (DataPortModel, ScopedVariableModel)):
                move_port(self.single_selection)

    def _move_waypoint(self, new_pos, modifier_keys):
        connection_m = self.selected_waypoint[0]
        connection_temp = connection_m.temp['gui']['editor']
        waypoints = connection_m.meta['gui']['editor']['waypoints']
        waypoint_id = self.selected_waypoint[1]
        parent_state_m = connection_m.parent
        new_pos = self._limit_position_to_state(parent_state_m, new_pos)
        new_rel_pos = self._get_position_relative_to_state(parent_state_m, new_pos)

        # With the shift key pressed, try to snap the waypoint such that the connection has a multiple of 45 deg
        if modifier_keys & SHIFT_MASK != 0:
            snap_angle = deg2rad(global_gui_config.get_config_value('WAYPOINT_SNAP_ANGLE', 45.))
            snap_diff = deg2rad(global_gui_config.get_config_value('WAYPOINT_SNAP_MAX_DIFF_ANGLE', 10.))
            max_snap_dist = global_gui_config.get_config_value('WAYPOINT_SNAP_MAX_DIFF_PIXEL', 50.)
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
                prev_point = self._get_position_relative_to_state(connection_m.parent, connection_temp['from_pos'])
            if waypoint_id < len(waypoints) - 1:
                next_point = waypoints[waypoint_id + 1]
            else:
                next_point = self._get_position_relative_to_state(connection_m.parent, connection_temp['to_pos'])

            new_rel_pos = calculate_snap_point(prev_point, new_rel_pos, next_point)

        waypoints[waypoint_id] = new_rel_pos
        self._publish_changes(connection_m, "Move waypoint", affects_children=False)
        self._redraw()

    def _resize_state(self, state_m, new_corner_pos, modifier_keys):
        """Resize the state by the given delta width and height

        The resize function checks the child states and keeps the state around the children, thus limiting the minimum
        size. Two modifier keys can be used to alter the resize options:
         - Ctrl also causes the child states to be resized
         - Shift caused the resized states to keep their width to height ratio

        :param state_m: The model of the state to be resized
        :param new_corner_pos: The absolute coordinates of the new desired lower right corner
        :param modifier_keys: The current pressed modifier keys (mask)
        """
        state_temp = state_m.temp['gui']['editor']
        state_meta = state_m.meta['gui']['editor']

        new_width = new_corner_pos[0] - state_temp['pos'][0]
        new_height = abs(new_corner_pos[1] - state_temp['pos'][1])

        # Keep size ratio?
        if int(modifier_keys & SHIFT_MASK) > 0:
            state_size_ratio = state_meta['size'][0] / state_meta['size'][1]
            new_state_size_ratio = new_width / new_height

            if new_state_size_ratio < state_size_ratio:
                new_height = new_width / state_size_ratio
            else:
                new_width = new_height * state_size_ratio

        # User wants to resize content by holding the ctrl keys pressed
        resize_content = int(modifier_keys & CONTROL_MASK) > 0

        min_right_edge = state_temp['pos'][0]
        max_bottom_edge = state_temp['pos'][1]

        # If the content is not supposed to be resized, with have to calculate the inner edges, which define the
        # minimum size of our state
        if not resize_content and self.has_content(state_m):
            # Check lower right corner of all child states
            for child_state_m in state_m.states.itervalues():
                _, child_right_edge, child_bottom_edge, _ = self.get_boundaries(child_state_m)
                if child_right_edge is not None and child_bottom_edge is not None:
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
                if port_right_edge is not None and port_bottom_edge is not None:
                    min_right_edge = port_right_edge if min_right_edge < port_right_edge else min_right_edge
                    max_bottom_edge = port_bottom_edge if max_bottom_edge > port_bottom_edge else max_bottom_edge

        # Check for parent size limitation
        max_right_edge = sys.maxint
        min_bottom_edge = -sys.maxint - 1
        if state_m.parent is not None:
            max_right_edge = state_m.parent.temp['gui']['editor']['pos'][0] + \
                             state_m.parent.meta['gui']['editor']['size'][0]
            min_bottom_edge = state_m.parent.temp['gui']['editor']['pos'][1] - \
                              state_m.parent.meta['gui']['editor']['size'][1]

        # Desired new edges
        desired_right_edge = state_temp['pos'][0] + new_width
        desired_bottom_edge = state_temp['pos'][1] - new_height

        # Old values
        old_size = state_meta['size']

        # Check for all restrictions
        if new_width > 0:  # Minimum width
            if desired_right_edge > max_right_edge:  # Keep state in its parent
                new_width = max_right_edge - state_temp['pos'][0]
            elif desired_right_edge < min_right_edge:  # Surround all children
                new_width = min_right_edge - state_temp['pos'][0]
        if new_height > 0:  # Minimum height
            if desired_bottom_edge > max_bottom_edge:  # Keep state in its parent
                new_height = state_temp['pos'][1] - max_bottom_edge
            elif desired_bottom_edge < min_bottom_edge:  # Surround all children
                new_height = state_temp['pos'][1] - min_bottom_edge

        state_meta['size'] = (new_width, new_height)

        # Resize content if the state was resized and the modifier key is pressed
        if resize_content and (state_meta['size'][0] != old_size[0] or state_meta['size'][1] != old_size[1]):

            # Recursive call
            def resize_children(state_m, old_size, new_size):
                width_factor = new_size[0] / old_size[0]
                height_factor = new_size[1] / old_size[1]

                def calc_new_rel_pos(old_rel_pos, old_parent_size, new_parent_size):
                    old_rel_pos_x_rel = old_rel_pos[0] / old_parent_size[0]
                    old_rel_pos_y_rel = old_rel_pos[1] / old_parent_size[1]
                    new_rel_pos_x = new_parent_size[0] * old_rel_pos_x_rel
                    new_rel_pos_y = new_parent_size[1] * old_rel_pos_y_rel
                    return new_rel_pos_x, new_rel_pos_y

                # Only container states have content
                if self.has_content(state_m):
                    # Resize all transitions
                    for transition_m in state_m.transitions:
                        # By repositioning all waypoints
                        for i, waypoint in enumerate(transition_m.meta['gui']['editor']['waypoints']):
                            new_rel_pos = calc_new_rel_pos(waypoint, old_size, new_size)
                            transition_m.meta['gui']['editor']['waypoints'][i] = new_rel_pos
                    # Resize all data flows
                    for data_flow_m in state_m.data_flows:
                        # By repositioning all waypoints
                        for i, waypoint in enumerate(data_flow_m.meta['gui']['editor']['waypoints']):
                            new_rel_pos = calc_new_rel_pos(waypoint, old_size, new_size)
                            data_flow_m.meta['gui']['editor']['waypoints'][i] = new_rel_pos

                    for port_m in itertools.chain(state_m.input_data_ports, state_m.output_data_ports,
                                                  state_m.scoped_variables):
                        old_rel_pos = port_m.meta['gui']['editor']['inner_rel_pos']
                        port_m.meta['gui']['editor']['inner_rel_pos'] = calc_new_rel_pos(old_rel_pos, old_size,
                                                                                         new_size)

                    # Resize all child states
                    for child_state_m in state_m.states.itervalues():
                        old_rel_pos = child_state_m.meta['gui']['editor']['rel_pos']
                        child_state_m.meta['gui']['editor']['rel_pos'] = calc_new_rel_pos(old_rel_pos, old_size,
                                                                                          new_size)

                        old_size = child_state_m.meta['gui']['editor']['size']
                        new_size = (old_size[0] * width_factor, old_size[1] * height_factor)
                        child_state_m.meta['gui']['editor']['size'] = new_size

                        if self.has_content(child_state_m):
                            resize_children(child_state_m, old_size, new_size)

            # Start recursive call of the content resize
            resize_children(state_m, old_size, state_meta['size'])

        affects_children = self.has_content(self.single_selection) and resize_content
        self._publish_changes(state_m, "Resize state", affects_children)
        self._redraw()

    def _move_view(self, rel_motion, opengl_coords=False):
        """Move the view according to the relative coordinates

        The whole view/scene is moved, causing the state machine to move within the viewport.

        :param rel_x_motion: Distance to move in x direction
        :param rel_y_motion: Distance to move in y direction
        :param opengl_coords: Whether to specified relative coordinates are in OpenGl coordinate system
        """
        if not opengl_coords:
            conversion = self.view.editor.pixel_to_size_ratio()
            rel_motion = (rel_motion[0] / conversion, -rel_motion[1] / conversion)
            aspect = self.view.editor.allocation.width / float(self.view.editor.allocation.height)
            if aspect > 1:
                rel_motion = (rel_motion[0] / aspect, rel_motion[1])
            else:
                rel_motion = (rel_motion[0], rel_motion[1] * aspect)
        self.view.editor.left -= rel_motion[0]
        self.view.editor.right -= rel_motion[0]
        self.view.editor.bottom -= rel_motion[1]
        self.view.editor.top -= rel_motion[1]
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
            diff = subtract_pos(new_mouse_pos, old_mouse_pos)
            if aspect < 1:
                diff = (diff[0], diff[1] * aspect)
            else:
                diff = (diff[0] / aspect, diff[1])

            # Move view to keep the previous mouse position in the view
            self._move_view(diff, opengl_coords=True)

    def draw_state_machine(self):
        """Draws remaining components of the state machine

        This method draws all other components, not directly belonging to a certain state. For a starter, this is the
        selection frame the user draws for a multi selection.
        """

        # Draw the multi selection frame
        frame = self.model.temp['gui']['editor']['selection_frame']
        if isinstance(frame, list):
            self.view.editor.draw_frame(frame[0], frame[1], 10)

    def draw_state(self, state_m, rel_pos=(0, 0), size=(100, 100), depth=1):
        """Draws a (container) state with all its content

        Mainly contains the logic for drawing (e. g. reading and calculating values). The actual drawing process is
        done in the view, which is called from this method with the appropriate arguments.

        :param state_m: The state to be drawn
        :param rel_pos: The default relative position (x, y) if there is no relative position stored
        :param size: The default size (width, height) if there is no size stored
        :param depth: The hierarchy level of the state
        """
        assert isinstance(state_m, StateModel)

        # Use default values if no size information is stored
        if not isinstance(state_m.meta['gui']['editor']['size'], tuple):
            state_m.meta['gui']['editor']['size'] = size

        size = state_m.meta['gui']['editor']['size']

        # Root state is always in the origin
        if state_m.parent is None:
            pos = (0, 0)
        else:
            # Use default values if no size information is stored
            # Here the possible case of pos_x and posy_y == 0 must be handled
            if not isinstance(state_m.meta['gui']['editor']['rel_pos'], tuple):
                state_m.meta['gui']['editor']['rel_pos'] = rel_pos

            rel_pos = state_m.meta['gui']['editor']['rel_pos']
            pos = self._get_absolute_position(state_m.parent, rel_pos)

        state_m.temp['gui']['editor']['pos'] = pos

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
        (opengl_id, income_pos, outcome_pos, outcome_radius, resize_length) = self.view.editor.draw_state(
            state_m.state.name, pos, size,
            state_m.state.outcomes,
            state_m.input_data_ports if global_gui_config.get_config_value('show_data_flows', True) else [],
            state_m.output_data_ports if global_gui_config.get_config_value('show_data_flows', True) else [],
            selected, active, depth)
        state_m.temp['gui']['editor']['id'] = opengl_id
        state_m.temp['gui']['editor']['income_pos'] = income_pos
        state_m.temp['gui']['editor']['outcome_pos'] = outcome_pos
        state_m.temp['gui']['editor']['outcome_radius'] = outcome_radius
        state_m.temp['gui']['editor']['resize_length'] = resize_length

        # If the state is a container state, we also have to draw its transitions and data flows as well as
        # recursively its child states
        if self.has_content(state_m):
            num_child_state = 0
            width = size[0]
            height = size[1]

            for child_state in state_m.states.itervalues():
                # Calculate default positions for the child states
                # Make the inset from the top left corner

                child_width = width / 5.
                child_height = height / 5.
                child_size = (child_width, child_height)
                child_spacing = max(child_size) * 1.2

                max_cols = width // child_spacing
                (row, col) = divmod(num_child_state, max_cols)
                child_rel_pos_x = col * child_spacing + child_spacing - child_width
                child_rel_pos_y = -child_spacing * (1.5 * row + 1)
                child_rel_pos = (child_rel_pos_x, child_rel_pos_y)
                num_child_state += 1

                self.draw_state(child_state, child_rel_pos, child_size, depth + 1)

            if global_gui_config.get_config_value('show_data_flows', True):
                self.draw_inner_data_ports(state_m, depth)

            self.draw_transitions(state_m, depth)

            if global_gui_config.get_config_value('show_data_flows', True):
                self.draw_data_flows(state_m, depth)

        self._handle_new_transition(state_m, depth)

        if global_gui_config.get_config_value('show_data_flows', True):
            self._handle_new_data_flow(state_m, depth)

    def draw_inner_data_ports(self, parent_state_m, parent_depth):
        """Draw the inner ports of a state

        This method draws the ports that are displayed within a container state. The inner ports are the input data
        ports, output data ports and scoped variables.

        :param parent_state_m: The parent state model of the ports
        :param parent_depth: The depth of the parent state
        """
        parent_meta = parent_state_m.meta['gui']['editor']
        parent_temp = parent_state_m.temp['gui']['editor']
        max_rows = max(20, len(parent_state_m.input_data_ports), len(parent_state_m.output_data_ports))
        port_height = min(parent_meta['size']) / float(max_rows)
        max_port_width = min(parent_meta['size']) / 5.
        size = (max_port_width, port_height)

        # Input data ports
        num_input_ports = 1
        for port_m in parent_state_m.input_data_ports:
            port = port_m.data_port
            if not isinstance(port_m.meta['gui']['editor']['inner_rel_pos'], tuple):
                # Put input ports by default in the lower left corner
                rel_pos = (0, -parent_meta['size'][1] + num_input_ports * port_height)
                port_m.meta['gui']['editor']['inner_rel_pos'] = rel_pos
            pos = self._get_absolute_position(parent_state_m, port_m.meta['gui']['editor']['inner_rel_pos'])
            port_m.temp['gui']['editor']['inner_pos'] = pos

            selected = port_m in self.model.selection.get_all()
            opengl_id = self.view.editor.draw_inner_input_data_port(port.name, port_m, pos, size, selected,
                                                                    parent_depth + 0.5)
            port_m.temp['gui']['editor']['id'] = opengl_id
            num_input_ports += 1

        # Output data ports
        num_output_ports = 1
        for port_m in parent_state_m.output_data_ports:
            port = port_m.data_port
            if not isinstance(port_m.meta['gui']['editor']['inner_rel_pos'], tuple):
                # Put output ports by default in the lower right corner
                rel_pos = (parent_meta['size'][0], -parent_meta['size'][1] + num_output_ports * port_height)
                port_m.meta['gui']['editor']['inner_rel_pos'] = rel_pos
            pos = self._get_absolute_position(parent_state_m, port_m.meta['gui']['editor']['inner_rel_pos'])
            port_m.temp['gui']['editor']['inner_pos'] = pos

            selected = port_m in self.model.selection.get_all()
            opengl_id = self.view.editor.draw_inner_output_data_port(port.name, port_m, pos, size, selected,
                                                                     parent_depth + 0.5)
            port_m.temp['gui']['editor']['id'] = opengl_id
            num_output_ports += 1

        # Scoped variables
        num_scoped_variables = 0
        for port_m in parent_state_m.scoped_variables:
            port = port_m.scoped_variable
            if not isinstance(port_m.meta['gui']['editor']['inner_rel_pos'], tuple):
                # Put scoped variables by default row-wise in at the top
                max_cols = parent_meta['size'][0] // max_port_width
                (row, col) = divmod(num_scoped_variables, max_cols)
                rel_pos = (col * max_port_width, -port_height * (2 * row + 1))
                port_m.meta['gui']['editor']['inner_rel_pos'] = rel_pos
            pos = self._get_absolute_position(parent_state_m, port_m.meta['gui']['editor']['inner_rel_pos'])
            port_m.temp['gui']['editor']['inner_pos'] = pos

            selected = port_m in self.model.selection.get_all()
            opengl_id = self.view.editor.draw_scoped_data_port(port.name, port_m, pos, size, selected,
                                                               parent_depth + 0.5)
            port_m.temp['gui']['editor']['id'] = opengl_id
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
                from_pos = parent_state_m.temp['gui']['editor']['income_pos']
            else:
                from_state = parent_state_m.states[from_state_id]

                assert isinstance(from_state, StateModel), "Transition from unknown state with ID {id:s}".format(
                    id=from_state_id)

                try:
                    # Set the from coordinates to the outcome coordinates received earlier
                    from_pos = parent_state_m.states[from_state_id].temp['gui']['editor']['outcome_pos'][
                        transition_m.transition.from_outcome]
                except Exception as e:
                    logger.error("""Outcome position was not found. \
                                maybe the outcome for the transition was not found: {err}""".format(err=e))
                    continue

            to_state_id = transition_m.transition.to_state
            to_state = None if to_state_id is None else parent_state_m.states[to_state_id]

            if to_state is None:  # Transition goes back to parent
                # Set the to coordinates to the outcome coordinates received earlier
                to_pos = parent_state_m.temp['gui']['editor']['outcome_pos'][
                    transition_m.transition.to_outcome]
            else:
                # Set the to coordinates to the center of the next state
                to_pos = to_state.temp['gui']['editor']['income_pos']

            waypoints = []
            for waypoint in transition_m.meta['gui']['editor']['waypoints']:
                waypoint_pos = self._get_absolute_position(parent_state_m, waypoint)
                waypoints.append(waypoint_pos)

            # Let the view draw the transition and store the returned OpenGL object id
            selected = False
            if transition_m in self.model.selection.get_transitions():
                selected = True
            line_width = self.view.editor.transition_stroke_width(parent_state_m)
            opengl_id = self.view.editor.draw_transition(from_pos, to_pos, line_width, waypoints,
                                                         selected, parent_depth + 0.5)
            transition_m.temp['gui']['editor']['id'] = opengl_id
            transition_m.temp['gui']['editor']['from_pos'] = from_pos
            transition_m.temp['gui']['editor']['to_pos'] = to_pos

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
                from_pos = from_port.temp['gui']['editor']['connector_pos']
            elif from_state_id == parent_state_m.state.state_id:  # The data flow is connected to the parents input
                from_pos = from_port.temp['gui']['editor']['inner_connector_pos']
            else:
                from_pos = from_port.temp['gui']['editor']['outer_connector_pos']
            if isinstance(to_port, ScopedVariableModel):
                to_pos = to_port.temp['gui']['editor']['connector_pos']
            elif to_state_id == parent_state_m.state.state_id:  # The data flow is connected to the parents output
                to_pos = to_port.temp['gui']['editor']['inner_connector_pos']
            else:
                to_pos = to_port.temp['gui']['editor']['outer_connector_pos']

            waypoints = []
            for waypoint in data_flow_m.meta['gui']['editor']['waypoints']:
                waypoint_pos = self._get_absolute_position(parent_state_m, waypoint)
                waypoints.append(waypoint_pos)

            selected = False
            if data_flow_m in self.model.selection.get_data_flows():
                selected = True
            line_width = self.view.editor.data_flow_stroke_width(parent_state_m)
            opengl_id = self.view.editor.draw_data_flow(from_pos, to_pos, line_width, waypoints,
                                                        selected, parent_depth + 0.5)
            data_flow_m.temp['gui']['editor']['id'] = opengl_id
            data_flow_m.temp['gui']['editor']['from_pos'] = from_pos
            data_flow_m.temp['gui']['editor']['to_pos'] = to_pos

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
            parent_state_m = self.single_selection.parent if self.selected_port_type != "outer" else \
                self.single_selection.parent.parent
        else:
            return
        restricted_click = self._limit_position_to_state(parent_state_m, self.mouse_move_last_coords)
        # If the user clicked with the parent state of the selected outcome state
        if pos_equal(restricted_click, self.mouse_move_last_coords):
            rel_pos = self._get_position_relative_to_state(parent_state_m, restricted_click)
            self.temporary_waypoints.append(rel_pos)

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
                    responsible_parent_m = parent_state_m
                    origin = parent_state_m.temp['gui']['editor']['income_pos']
                else:
                    outcome = parent_state_m.temp['gui']['editor']['outcome_pos'][self.selected_outcome[1]]
                    responsible_parent_m = parent_state_m if parent_state_m.parent is None else parent_state_m.parent
                    origin = outcome
                cur = self.mouse_move_last_coords
                target = self._limit_position_to_state(responsible_parent_m, cur)
                line_width = self.view.editor.transition_stroke_width(responsible_parent_m)
                waypoints = []
                for waypoint in self.temporary_waypoints:
                    waypoint_pos = self._get_absolute_position(responsible_parent_m, waypoint)
                    waypoints.append(waypoint_pos)
                self.view.editor.draw_transition(origin, target, line_width,
                                                 waypoints, True, parent_depth + 0.6)

    def _handle_new_data_flow(self, parent_state_m, parent_depth):
        """Responsible for drawing new data flows the user creates

        With drag and drop on ports, the user can draw new data flows. Here the data flow is temporary drawn in the
        graphical editor.

        :param parent_state_m: Model of the container state
        :param parent_depth: Depth of the container state
        """
        if self.selected_port_connector:  # and self.last_button_pressed == 1:
            port_m = self.single_selection
            assert isinstance(self.single_selection, (DataPortModel, ScopedVariableModel))
            if (port_m.parent == parent_state_m and self.selected_port_type in ("inner", "scope")) or \
                    (port_m.parent.parent == parent_state_m and self.selected_port_type == "outer"):
                if self.selected_port_type == "inner":
                    connector = port_m.temp['gui']['editor']['inner_connector_pos']
                elif self.selected_port_type == "outer":
                    connector = port_m.temp['gui']['editor']['outer_connector_pos']
                else:  # scoped variable
                    connector = port_m.temp['gui']['editor']['connector_pos']
                cur = self.mouse_move_last_coords
                target = self._limit_position_to_state(parent_state_m, cur)
                line_width = self.view.editor.data_flow_stroke_width(parent_state_m)
                waypoints = []
                for waypoint in self.temporary_waypoints:
                    waypoint_pos = self._get_absolute_position(parent_state_m, waypoint)
                    waypoints.append(waypoint_pos)
                self.view.editor.draw_data_flow(connector, target, line_width,
                                                waypoints, True, parent_depth + 0.6)

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
            if search_state.temp['gui']['editor']['id'] and search_state.temp['gui']['editor']['id'] in ids:
                # if so, add the state to the list of selected states
                selection = update_selection(selection, search_state)
                selection_depth = search_state_depth
                # remove the id from the list to fasten up further searches
                ids.remove(search_state.temp['gui']['editor']['id'])

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
                    if model.temp['gui']['editor']['id'] and model.temp['gui']['editor']['id'] in ids:
                        ids.remove(model.temp['gui']['editor']['id'])
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
        if isinstance(state_m, ContainerStateModel):
            return True
        return False

    @staticmethod
    def get_boundaries(model, include_waypoints=False):
        """Returns the boundaries (in OpenGL) coordinates of the given model

        :param model: Model for which to get the boundaries
        :return: left, right, bottom, top
        """
        meta = model.meta['gui']['editor']
        temp = model.temp['gui']['editor']
        if isinstance(model, StateModel):
            return temp['pos'][0], temp['pos'][0] + meta['size'][0], temp['pos'][1] - meta['size'][1], temp['pos'][1]
        if isinstance(model, (TransitionModel, DataFlowModel)):
            x_coordinates = [temp['from_pos'][0], temp['to_pos'][0]]
            y_coordinates = [temp['from_pos'][1], temp['to_pos'][1]]
            if include_waypoints:
                for waypoint in meta['waypoints']:
                    x_coordinates.append(waypoint[0])
                    y_coordinates.append(waypoint[1])
            return min(x_coordinates), max(x_coordinates), min(y_coordinates), max(y_coordinates)

        if isinstance(model, (DataPortModel, ScopedVariableModel)):
            try:  # Data port position might not be set if data connections are not shown
                left = temp['inner_pos'][0]
                right = temp['inner_pos'][0] + temp['size'][0]
                top = temp['inner_pos'][1]
                bottom = temp['inner_pos'][1] - temp['size'][1]

                if model in model.parent.output_data_ports:
                    left = temp['inner_pos'][0] - temp['size'][0]
                    right = temp['inner_pos'][0]
                if model in model.parent.scoped_variables:
                    pass
                    # bottom = temp['inner_pos'][1] - temp['size'][1] + temp['size_rect'][1]
                    # top = bottom + temp['size'][1]

                return left, right, bottom, top
            except TypeError:
                return None, None, None, None

        return None, None, None, None

    @staticmethod
    def _get_position_relative_to_state(state_m, abs_pos):
        state_pos = state_m.temp['gui']['editor']['pos']
        rel_pos = subtract_pos(abs_pos, state_pos)
        return rel_pos

    @staticmethod
    def _get_absolute_position(state_m, rel_pos):
        state_pos = state_m.temp['gui']['editor']['pos']
        abs_pos = add_pos(rel_pos, state_pos)
        return abs_pos

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
            global_gui_config.set_config_value('show_data_flows',
                                               not global_gui_config.get_config_value("show_data_flows"))
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

            elif self.drag_origin_offset is not None:
                # Multi-selection move
                if isinstance(self.drag_origin_offset, list):
                    for model in self.model.selection:
                        model_meta = model.meta['gui']['editor']
                        model_temp = model.temp['gui']['editor']
                        if isinstance(model, StateModel):
                            model_meta['rel_pos'] = model_temp['original_rel_pos']
                        elif isinstance(model, (DataPortModel, ScopedVariableModel)):
                            model_meta['inner_rel_pos'] = model_temp['original_inner_rel_pos']
                elif isinstance(self.single_selection, StateModel):
                    self.single_selection.meta['gui']['editor']['rel_pos'] = \
                        self.single_selection.temp['gui']['editor']['original_rel_pos']
                elif isinstance(self.single_selection, (DataPortModel, ScopedVariableModel)):
                    self.single_selection.meta['gui']['editor']['inner_rel_pos'] = \
                        self.single_selection.temp['gui']['editor']['original_inner_rel_pos']

                self.last_button_pressed = None  # prevents further movements
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
            logger.debug("Paste")

            current_selection = self.model.selection

            if len(current_selection) != 1 or len(current_selection.get_states()) < 1:
                logger.error("Please select a single state for pasting the clipboard")
                return
            if not isinstance(current_selection.get_states()[0], ContainerStateModel):
                logger.error("Please select a container state (e. g. a Hierarchy state) for pasting the clipboard.")
                return

            # Note: in multi-selection case, a loop over all selected items is necessary instead of the 0 index
            target_state_m = current_selection.get_states()[0]
            state_copy_m, state_orig_m = global_clipboard.paste(target_state_m)

            if state_copy_m is None:  # An error occurred while pasting
                return

            # Adjust size of new state
            old_size = state_orig_m.meta['gui']['editor']['size']
            target_size = target_state_m.meta['gui']['editor']['size']

            # Use the old size, if it is smaller than the target state
            if old_size[0] < target_size[0] and old_size[1] < target_size[1]:
                new_size = old_size
            # Resize to 1/3 of the target state, but keep the size ratio
            else:
                new_size = (target_size[0] / 3., target_size[1] / 3.)
                old_size_ratio = old_size[0] / old_size[1]
                if old_size_ratio < new_size[0] / new_size[1]:
                    new_size = (new_size[1] * old_size_ratio, new_size[1])
                else:
                    new_size = (new_size[0], new_size[0] / old_size_ratio)
            state_copy_m.meta['gui']['editor']['size'] = new_size

            self._redraw()