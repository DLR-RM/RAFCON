# Copyright (C) 2014-2018 DLR
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

"""
.. module:: graphical_editor
   :synopsis: A module that holds the controller to graphically edit a state machine by the opengl library.

"""

import gobject
import itertools
import sys
import time
from copy import copy
from functools import partial
from gtk import DEST_DEFAULT_ALL
from gtk.gdk import ACTION_COPY, ModifierType
from gtk.gdk import SCROLL_DOWN, SCROLL_UP, SHIFT_MASK, CONTROL_MASK, BUTTON1_MASK, BUTTON2_MASK, BUTTON3_MASK
from gtk.gdk import keyval_name
from math import sin, cos, atan2

from rafcon.core.decorators import lock_state_machine
from rafcon.core.states.state import StateExecutionStatus
from rafcon.core.states.state import StateType
from rafcon.gui.clipboard import global_clipboard
from rafcon.gui.config import global_gui_config
from rafcon.gui.controllers.right_click_menu.state import StateRightClickMenuControllerOpenGLEditor
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
import rafcon.gui.helpers.state_machine as gui_helper_state_machine
from rafcon.gui.helpers.label import react_to_event
from rafcon.gui.helpers.meta_data import contains_geometric_info, add_pos, subtract_pos, \
    insert_self_transition_meta_data
from rafcon.gui.models import ContainerStateModel, TransitionModel, DataFlowModel
from rafcon.gui.models.abstract_state import AbstractStateModel
from rafcon.gui.models.data_port import DataPortModel
from rafcon.gui.models.library_state import LibraryStateModel
from rafcon.gui.models.scoped_variable import ScopedVariableModel
from rafcon.gui.models.signals import MetaSignalMsg
from rafcon.gui.models.state_machine import StateMachineModel
from rafcon.gui.runtime_config import global_runtime_config
import rafcon.gui.singleton
from rafcon.gui.views.graphical_editor import Direction, GraphicalEditorView
from rafcon.utils.geometry import point_in_triangle, dist, point_on_line, deg2rad
from rafcon.utils import log

logger = log.get_logger(__name__)


def check_pos(pos):
    if not contains_geometric_info(pos):
        raise ValueError("Position is not valid: {}".format(pos))


def pos_equal(pos1, pos2):
    check_pos(pos1)
    check_pos(pos2)
    return pos1[0] == pos2[0] and pos1[1] == pos2[1]


def calculate_size(current_size, maximum_size):
    # Current size smaller than maximum size, leave as is
    if current_size[0] <= maximum_size[0] and current_size[1] <= maximum_size[1]:
        return copy(current_size)
    # Otherwise calculate new size
    width_ratio = float(current_size[0]) / maximum_size[0]
    height_ratio = float(current_size[1]) / maximum_size[1]
    current_size_ratio = float(current_size[0]) / current_size[1]
    if width_ratio > height_ratio:
        new_width = maximum_size[0]
        new_height = new_width / current_size_ratio
    else:
        new_height = maximum_size[1]
        new_width = new_height * current_size_ratio
    return new_width, new_height


class GraphicalEditorController(ExtendedController):
    """Controller handling the graphical editor

    :param rafcon.gui.models.state_machine.StateMachineModel model: The state machine model, holding the root
        state and the current selection
    :param rafcon.gui.views.graphical_editor.GraphicalEditorView view: The GTK view having an OpenGL rendering
        element
    """

    def __init__(self, model, view):
        """Constructor"""
        assert isinstance(model, StateMachineModel)
        assert isinstance(view, GraphicalEditorView)
        super(GraphicalEditorController, self).__init__(model, view)
        self.add_controller("state_right_click_ctrl", StateRightClickMenuControllerOpenGLEditor(model, view))

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

        self.changed_models = []
        self.changes_affect_children = False
        self._last_meta_data_changed = None

        self._ongoing_complex_actions = []
        self._suspend_drawing = False

        self.last_time = time.time()

    def register_view(self, view):
        """Called when the View was registered"""
        super(GraphicalEditorController, self).register_view(view)
        view.editor.connect('expose_event', self._on_expose_event)
        view.editor.connect('button-press-event', self._on_mouse_press)
        view.editor.connect('button-release-event', self._on_mouse_release)
        view.editor.connect('motion-notify-event', self._on_mouse_motion)
        view.editor.connect('scroll-event', self._on_scroll)
        view.editor.connect('key-press-event', self._on_key_press)
        view.editor.connect('key-release-event', self._on_key_release)

        view.editor.drag_dest_set(DEST_DEFAULT_ALL, [('STRING', 0, 0)], ACTION_COPY)
        view.editor.connect("drag-data-received", self.on_drag_data_received)
        view.editor.connect("drag-motion", self.on_drag_motion)

        self.setup_opengl()

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.gui.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
            between shortcuts and actions.
        """
        shortcut_manager.add_callback_for_action("delete", self._delete_selection)
        shortcut_manager.add_callback_for_action("add", partial(self._add_new_state, state_type=StateType.EXECUTION))
        shortcut_manager.add_callback_for_action("add_execution_state", partial(self._add_new_state,
                                                                                state_type=StateType.EXECUTION))
        shortcut_manager.add_callback_for_action("add_hierarchy_state", partial(self._add_new_state,
                                                                                state_type=StateType.HIERARCHY))
        shortcut_manager.add_callback_for_action("add_barrier_state:", partial(self._add_new_state,
                                                                               state_type=StateType.BARRIER_CONCURRENCY))
        shortcut_manager.add_callback_for_action("add_preemptive_state:", partial(self._add_new_state,
                                                                                  state_type=StateType.PREEMPTION_CONCURRENCY))

        shortcut_manager.add_callback_for_action("abort", self._abort)

        shortcut_manager.add_callback_for_action("add_output", partial(self._add_data_port_to_selected_state,
                                                                       data_port_type='OUTPUT'))
        shortcut_manager.add_callback_for_action("add_input", partial(self._add_data_port_to_selected_state,
                                                                      data_port_type='INPUT'))
        shortcut_manager.add_callback_for_action("add_scoped_variable", self._add_scoped_variable_to_selected_state)
        shortcut_manager.add_callback_for_action("add_outcome", self._add_outcome_to_selected_state)

        shortcut_manager.add_callback_for_action("copy", self._copy_selection)
        shortcut_manager.add_callback_for_action("paste", self._paste_clipboard)
        shortcut_manager.add_callback_for_action("cut", self._cut_selection)

        shortcut_manager.add_callback_for_action("left", partial(self._move_in_direction, Direction.left))
        shortcut_manager.add_callback_for_action("right", partial(self._move_in_direction, Direction.right))
        shortcut_manager.add_callback_for_action("up", partial(self._move_in_direction, Direction.top))
        shortcut_manager.add_callback_for_action("down", partial(self._move_in_direction, Direction.bottom))

    def setup_opengl(self):
        with self.model.state_machine.modification_lock():
            hash_before = self.model.mutable_hash()
            self._redraw()
            hash_after = self.model.mutable_hash()
            if hash_before.digest() != hash_after.digest():
                logger.debug("Hash has changed from {0} to {1}".format(hash_before.hexdigest(), hash_after.hexdigest()))
                self._meta_data_changed(self.root_state_m, 'append_initial_change', True)
                logger.info("Opening the state machine caused some meta data to be generated, which will be stored "
                            " when the state machine is being saved.")

    @property
    def suspend_drawing(self):
        return self._suspend_drawing

    @suspend_drawing.setter
    def suspend_drawing(self, value):
        self._suspend_drawing = value

    @ExtendedController.observe("state_action_signal", signal=True)
    def state_action_signal_before(self, model, prop_name, info):
        if not ('arg' in info and info['arg'].after is False):
            return

        action = info['arg'].action
        # from rafcon.gui.utils.notification_overview import NotificationOverview
        # logger.info("OPENGL action signal {0}".format(NotificationOverview(info, False, self.__class__.__name__)))
        if action in ['change_root_state_type', 'change_state_type', 'substitute_state',
                      'group_states', 'ungroup_state', 'paste', 'cut', 'undo/redo']:
            self._ongoing_complex_actions.append(action)
            self.suspend_drawing = True
            # print self.__class__.__name__, 'add complex action', action
            # logger.info("drawing suspended: {0}".format(self.suspend_drawing))
            if action in ['group_states', 'paste', 'cut']:
                self.observe_model(info['arg'].action_parent_m)
            else:
                self.observe_model(info['arg'].affected_models[0])

    @ExtendedController.observe("action_signal", signal=True)
    def action_signal_after(self, model, prop_name, info):
        if not ('arg' in info and info['arg'].after):
            return

        action = info['arg'].action
        # from rafcon.gui.utils.notification_overview import NotificationOverview
        # logger.info("OPENGL action signal {0}".format(NotificationOverview(info, False, self.__class__.__name__)))
        if action in ['change_state_type', 'change_root_state_type', 'substitute_state', 'group_states',
                      'ungroup_state', 'paste', 'cut', 'undo/redo']:
            self._ongoing_complex_actions.remove(action)
            self.relieve_model(model)
            # print self.__class__.__name__, 'remove complex action', action
            if not self._ongoing_complex_actions:
                self.suspend_drawing = False
                # logger.info("drawing suspended: {0} redraw".format(self.suspend_drawing))
                self._redraw()

    @ExtendedController.observe("state_machine", after=True)
    @ExtendedController.observe("meta_signal", signal=True)  # meta data of state machine changed
    @ExtendedController.observe("state_meta_signal", signal=True)  # meta data of any state within state machine changed
    def state_machine_change(self, model, prop_name, info):
        """Called on any change within the state machine

        This method is called, when any state, transition, data flow, etc. within the state machine changes. This
        then typically requires a redraw of the graphical editor, to display these changes immediately.

        :param rafcon.gui.models.state_machine.StateMachineModel model: The state machine model
        :param str prop_name: The property that was changed
        :param dict info: Information about the change
        """
        if 'method_name' in info:
            if isinstance(info['result'], Exception):
                logger.info("Exception handling in opengl")
                return

            if info['method_name'] == 'root_state_change':
                self._redraw()
            elif info['method_name'] == 'marked_dirty' and info['args'][1]:
                self._redraw()
        elif 'signal' in info:
            msg = info.arg
            if msg.change == 'show_content':
                library_state_m = msg.notification.model
                if library_state_m.meta['gui']['show_content'] is not library_state_m.show_content():
                    logger.warning("The content of the LibraryState won't be shown, "
                                   "because MAX_VISIBLE_LIBRARY_HIERARCHY is 1.")
                else:
                    self._redraw()

    @ExtendedController.observe("root_state", assign=True)
    def root_state_change(self, model, prop_name, info):
        """Called when the root state was exchanged

        Exchanges the local reference to the root state and redraws.

        :param rafcon.gui.models.state_machine.StateMachineModel model: The state machine model
        :param str prop_name: The root state
        :param dict info: Information about the change
        """
        if self.root_state_m is not model.root_state:
            logger.debug("The root state was exchanged")
            self.root_state_m = model.root_state
            self._redraw()

    @ExtendedController.observe("sm_selection_changed_signal", signal=True)
    @lock_state_machine
    def selection_change(self, model, prop_name, info):
        """Called when the selection was changed externally

        Updates the local selection and redraws.

        :param rafcon.gui.selection.Selection model: The state machine model
        :param str prop_name: The selection
        :param dict info: Information about the change
        """
        selection = None
        for selection in self.model.selection:
            pass
        if self.single_selection != selection:
            self.single_selection = selection
            self._redraw()

    @log.log_exceptions(logger)
    def _on_expose_event(self, *args):
        """Redraw the graphical editor

        This method is called typically when the editor window is resized or something triggers are redraw. This
        controller class handles the logic for redrawing, while the corresponding view handles the design.

        :param args: console arguments, not used
        """
        if self.suspend_drawing:
            return
        # Prepare the drawing process
        self.view.editor.expose_init(args)
        # The whole logic of drawing is triggered by calling the root state to be drawn
        with self.model.state_machine.modification_lock():
            redraw = self.draw_state(self.root_state_m)
            self.draw_state_machine()
        # Finish the drawing process (e.g. swap buffers)
        self.view.editor.expose_finish(args)
        if redraw:
            self._redraw()

    def _redraw(self, timer_triggered=False):
        """Force the graphical editor to be redrawn

        First triggers the configure event to cause the perspective to be updated, then trigger the actual expose
        event to redraw.
        """
        if self.suspend_drawing:
            return

        redraw_after = 1 / 50.  # sec
        # Check if initialized
        # and whether the last redraw was more than redraw_after ago
        if hasattr(self.view, "editor") and (time.time() - self.last_time > redraw_after) and \
                        rafcon.gui.singleton.state_machine_manager_model.selected_state_machine_id == \
                        self.model.state_machine.state_machine_id \
                and not self.suspend_drawing:
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

    @lock_state_machine
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

    @lock_state_machine
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

    @lock_state_machine
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

        # Multi-selection is started when the user hold the shift key pressed while clicking the left mouse button,
        # and does this _not_ on a resize handler or waypoint
        if event.button == 1 and event.state & SHIFT_MASK == 1 and \
                        self.selected_resizer is None and self.selected_waypoint is None:
            self.multi_selection_started = True

        # Left mouse button was clicked and no multi selection intended
        if event.button == 1 and event.state & SHIFT_MASK == 0:
            if not self.mouse_move_redraw:
                self.single_selection = new_selection

            # Check, whether an outcome was clicked on
            outcome_state, outcome_key = self._check_for_outcome_selection(new_selection, self.mouse_move_start_coords)

            # Check, whether a port (input, output, scope) was clicked on
            if global_runtime_config.get_config_value('SHOW_DATA_FLOWS', True):
                # Check, whether port (connector) was clicked on
                port_model, port_type, is_connector = self._check_for_port_selection(new_selection,
                                                                                     self.mouse_move_start_coords)
            else:
                port_model = None

            if outcome_state is not None:
                # Abort connection creation, as the user requested a connection from a data port to a logical port
                if self.selected_port_connector:
                    logger.error("Creation of connections between data and logic ports not allowed")
                    self._abort()

                # Store the selected outcome if no outcome was selected before, this is the start of a drag and drop
                # operation to create a new transition
                elif not self.selected_outcome:
                    if outcome_state is not self.root_state_m or outcome_key is None:
                        self.selected_outcome = outcome_state, outcome_key
                        self.mouse_move_redraw = True
                # If there is already a selected outcome, then we create a transition between the previously selected
                # and the new one. This is the end of a drag and drop operation to create a transition.
                else:
                    self._create_new_transition(outcome_state, outcome_key)
            # Another possibility to create a transition is by clicking the state of the transition target when
            # having an outcome selected.
            elif self.selected_outcome and isinstance(new_selection, AbstractStateModel) and \
                    ((new_selection.parent is self.selected_outcome[0].parent and self.selected_outcome[1] is not None)
                     or (new_selection.parent is self.selected_outcome[0] and not self.selected_outcome[1])):
                self._create_new_transition(new_selection)
            # Allow the user to create waypoints while creating a new transition
            elif self.selected_outcome:
                self._handle_new_waypoint()

            # Check, whether a port (input, output, scope) was clicked on
            if global_runtime_config.get_config_value('SHOW_DATA_FLOWS', True):
                if port_model is not None:
                    # Abort connection creation, as the user requested a connection from a logical port to a data port
                    if self.selected_outcome:
                        logger.error("Creation of connections between data and logic ports not allowed")
                        self._abort()

                    # Store the selected port if no port was selected before, this is the start of a drag and drop
                    # operation to create a new data flow
                    elif not self.selected_port_connector and is_connector:
                        self.model.selection.set(port_model)
                        self.selected_port_type = port_type
                        self.selected_port_connector = True
                        self.mouse_move_redraw = True
                    # If there is already a selected port, then we create a data flow between the previously selected
                    # and the new one. This is the end of a drag and drop operation to create a data flow.
                    else:
                        self._create_new_data_flow(port_model)
                # Allow the user to create waypoints while creating a new data flow
                elif self.selected_port_connector and isinstance(new_selection, AbstractStateModel):
                    self._handle_new_waypoint()

            self._redraw()

        # Right mouse button was clicked on
        elif event.button == 3:
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

    @lock_state_machine
    def _on_mouse_release(self, widget, event):
        """Triggered when a mouse button is being released

        :param widget: The widget beneath the mouse when the release was done
        :param event: Information about the event, e.g. x and y coordinate
        """
        self.last_button_pressed = None
        self.drag_origin_offset = None

        # Check if something was selected
        new_selection = self._find_selection(event.x, event.y)

        if event.button == 1:
            # We do not want to change the current selection while creating a new transition or data flow
            if not self.mouse_move_redraw:
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

        if self.changed_models:
            if len(self.changed_models) > 1:
                self.changes_affect_children = True
                self.changed_models = gui_helper_state_machine.reduce_to_parent_states(self.changed_models)
            if len(self.changed_models) > 1:
                parent_m = self.root_state_m
            else:
                parent_m = self.changed_models[0]
            if self._last_meta_data_changed:
                self._meta_data_changed(parent_m, self._last_meta_data_changed, self.changes_affect_children)
                self._last_meta_data_changed = None
            self.changed_models = []
            self.changes_affect_children = False

        self._redraw()

    @lock_state_machine
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

        # States and ports shell only be moved with the left mouse button clicked and the shift key not hold
        if event.state & SHIFT_MASK == 0 and self.last_button_pressed == 1:
            # Move all selected states and data ports of thr multi-selection
            if len(self.model.selection) > 1 and self.selected_outcome is None and self.selected_resizer is None:
                # When starting a move, two information are stored:
                # - the starting position, in case the user aborts a movement
                # - the offset of the mouse to the origin, which is kept throughout the movement to prevent jumps
                if self.drag_origin_offset is None:
                    self._last_meta_data_changed = "position"
                    self.drag_origin_offset = []
                    self.changes_affect_children = True
                    for model in self.model.selection:
                        offset = None
                        model_meta = model.get_meta_data_editor(for_gaphas=False)
                        model_temp = model.temp['gui']['editor']
                        if isinstance(model, AbstractStateModel):
                            self.changed_models.append(model)
                            offset = self._get_position_relative_to_state(model, self.mouse_move_start_coords)
                            model_temp['original_rel_pos'] = copy(model_meta['rel_pos'])
                        elif isinstance(model, (DataPortModel, ScopedVariableModel)):
                            self.changed_models.append(model)
                            offset = subtract_pos(self.mouse_move_start_coords, model_temp['inner_pos'])
                            model_temp['original_inner_rel_pos'] = copy(model_meta['inner_rel_pos'])
                        elif isinstance(model, (DataFlowModel, TransitionModel)):
                            self.changed_models.append(model)
                            waypoints = model_meta['waypoints']
                            if waypoints:
                                offset = {}
                                for waypoint_id, waypoint_pos in enumerate(waypoints):
                                    waypoint_abs_pos = self._get_absolute_position(model.parent, waypoint_pos)
                                    offset[waypoint_id] = subtract_pos(self.mouse_move_start_coords, waypoint_abs_pos)
                                    model_temp['original_waypoint_{0}_rel_pos'.format(waypoint_id)] = waypoint_pos
                        self.drag_origin_offset.append(offset)
                for i, model in enumerate(self.model.selection):
                    if self.drag_origin_offset[i] is not None:
                        if isinstance(model, AbstractStateModel):
                            new_pos = subtract_pos(mouse_current_coord, self.drag_origin_offset[i])
                            self._move_state(model, new_pos, redraw=False, publish_changes=False)
                        elif isinstance(model, (DataPortModel, ScopedVariableModel)):
                            new_pos = subtract_pos(mouse_current_coord, self.drag_origin_offset[i])
                            self._move_data_port(model, new_pos, redraw=False, publish_changes=False)
                        elif isinstance(model, (DataFlowModel, TransitionModel)):
                            waypoints = model.get_meta_data_editor(for_gaphas=False)['waypoints']
                            if waypoints:
                                for waypoint_id, waypoint_pos in enumerate(waypoints):
                                    new_pos = subtract_pos(mouse_current_coord, self.drag_origin_offset[i][waypoint_id])
                                    self._move_waypoint(model, waypoint_id, new_pos, redraw=False)

                self._redraw()

            # Move the current state, if the user didn't click on an outcome (wants to create a transition) or a
            # resize handler (wants to resize the state)
            elif isinstance(self.single_selection, AbstractStateModel) and \
                            self.selected_outcome is None and self.selected_resizer is None:
                selected_state_m = self.single_selection
                # Initially store starting position and offset (see comment above)
                if self.drag_origin_offset is None:
                    self._last_meta_data_changed = "position"
                    self.changes_affect_children = False
                    self.changed_models.append(selected_state_m)
                    offset = self._get_position_relative_to_state(selected_state_m, self.mouse_move_start_coords)
                    self.drag_origin_offset = offset
                    selected_state_m.temp['gui']['editor']['original_rel_pos'] = \
                        copy(selected_state_m.get_meta_data_editor(for_gaphas=False)['rel_pos'])
                new_pos = subtract_pos(mouse_current_coord, self.drag_origin_offset)
                self._move_state(selected_state_m, new_pos, publish_changes=False)

            # Move current data port, if the user didn't click on the connector (i. e. wants to create a data flow)
            elif isinstance(self.single_selection, (DataPortModel, ScopedVariableModel)) and \
                    not self.selected_port_connector:
                selected_port_m = self.single_selection
                # Initially store starting position and offset (see comment above)
                if self.drag_origin_offset is None:
                    self._last_meta_data_changed = "position"
                    self.changed_models.append(selected_port_m.parent)
                    self.drag_origin_offset = subtract_pos(self.mouse_move_start_coords,
                                                           selected_port_m.temp['gui']['editor']['inner_pos'])
                    selected_port_m.temp['gui']['editor']['original_inner_rel_pos'] = \
                        copy(selected_port_m.get_meta_data_editor(for_gaphas=False)['inner_rel_pos'])
                new_pos = subtract_pos(mouse_current_coord, self.drag_origin_offset)
                self._move_data_port(selected_port_m, new_pos, publish_changes=False)

        # Move the selected waypoint (if there is one)
        if self.selected_waypoint is not None:
            # Move selected waypoint within its container state
            connection_m = self.selected_waypoint[0]
            waypoint_id = self.selected_waypoint[1]
            if connection_m not in self.changed_models:
                self._last_meta_data_changed = "waypoint_position"
                self.changes_affect_children = False
                self.changed_models.append(connection_m)
            snap = event.state & SHIFT_MASK != 0
            self._move_waypoint(connection_m, waypoint_id, mouse_current_coord, snap)

        # Redraw to show the new transition/data flow the user is creating with drag and drop
        if self.selected_outcome is not None or self.selected_port_connector:
            self._redraw()

        if self.selected_resizer is not None:
            state_m = self.selected_resizer
            if self.drag_origin_offset is None:
                self._last_meta_data_changed = "size"
                self.changed_models.append(state_m)
                lower_right_corner = (state_m.temp['gui']['editor']['pos'][0] +
                                      state_m.get_meta_data_editor(for_gaphas=False)['size'][0],
                                      state_m.temp['gui']['editor']['pos'][1] -
                                      state_m.get_meta_data_editor(for_gaphas=False)['size'][1])
                self.drag_origin_offset = subtract_pos(self.mouse_move_start_coords, lower_right_corner)
            new_pos = subtract_pos(mouse_current_coord, self.drag_origin_offset)
            modifier_keys = event.state
            keep_ratio = int(modifier_keys & SHIFT_MASK) > 0
            resize_content = int(modifier_keys & CONTROL_MASK) > 0
            if resize_content:
                self.changes_affect_children = True
            self._resize_state(state_m, new_pos, keep_ratio=keep_ratio, resize_content=resize_content)

        self.mouse_move_last_pos = (event.x, event.y)
        self.mouse_move_last_coords = mouse_current_coord

    @lock_state_machine
    def _on_scroll(self, widget, event):
        """Triggered when the mouse wheel is turned

        Calls the zooming method.

        :param widget: The widget beneath the mouse when the event was triggered
        :param event: Information about the event, e. g. x and y coordinate and mouse wheel turning direction
        """
        self._handle_zooming((event.x, event.y), event.direction)

    @lock_state_machine
    def on_drag_data_received(self, widget, context, x, y, data, info, time):
        """Receives state_id from LibraryTree and moves the state to the position of the mouse

        :param widget:
        :param context:
        :param x: Integer: x-position of mouse
        :param y: Integer: y-position of mouse
        :param data: SelectionData: contains state_id
        :param info:
        :param time:
        """
        self.view.editor.emit("expose_event", None)
        rel_pos = self.view.editor.screen_to_opengl_coordinates((x, y))
        if self.model.selection.get_selected_state() is not None:
            self._move_state(self.model.selection.get_selected_state().states[data.get_text()], rel_pos,
                             combined_action=True)
        else:
            logger.warning('Could not select state at drag-position received.')

    @lock_state_machine
    def on_drag_motion(self, widget, context, x, y, time):
        """Changes the selection on mouse over during drag motion

        :param widget:
        :param context:
        :param x: Integer: x-position of mouse
        :param y: Integer: y-position of mouse
        :param time:
        """
        selection = self._find_selection(x, y, find_states=True, find_data_flows=False,
                                         find_data_ports=False, find_transitions=False)
        if not self.model.selection.is_selected(selection) \
                and (isinstance(selection, ContainerStateModel) or selection is None):
            self.single_selection = selection
            self.model.selection.set(selection)

    @staticmethod
    def _limit_position_to_state(state_m, pos, child_size=(0, 0)):
        pos_x, pos_y = pos
        if state_m is not None:
            state_meta = state_m.get_meta_data_editor(for_gaphas=False)
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

    @lock_state_machine
    def _check_for_waypoint_selection(self, selected_model, coords):
        """Check whether a waypoint was clicked on

        Checks whether the current selected_model is a transition or data flow and if so looks for a waypoint at the
        given
        coordinates. If a waypoint is found, it is stored together with its current position.

        :param gtkmvc.Model selected_model: The model that was clicked on
        :param tuple coords: Coordinates of the click to search for waypoints
        """
        if isinstance(selected_model, (TransitionModel, DataFlowModel)):
            connection_m = selected_model
            parent_state_m = connection_m.parent
            close_threshold = min(parent_state_m.get_meta_data_editor(for_gaphas=False)['size']) / 50.
            # Check distance between all waypoints of the selected transition/data flows and the given coordinates
            for i, waypoint in enumerate(connection_m.get_meta_data_editor(for_gaphas=False)['waypoints']):
                waypoint_pos = self._get_absolute_position(parent_state_m, waypoint)
                # Only if coordinates are stored for the waypoints (always should be the case)
                if dist(waypoint_pos, coords) < close_threshold:
                    # As tuples cannot be changed, we have to store the whole list plus the index
                    self.selected_waypoint = (connection_m, i)
                    break

    @staticmethod
    def _check_for_outcome_selection(selected_model, coords):
        """Check whether a port was clicked on

        Checks whether the current selected_model is a state and if so looks for an outcome at the given coordinates.
        If an outcome is found, it is stored.

        :param gtkmvc.Model selected_model: The model that was clicked on
        :param tuple coords: Coordinates to search for outcomes
        """
        if isinstance(selected_model, AbstractStateModel):  # and self.single_selection is not self.root_state_m:
            state_m = selected_model
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
    def _check_for_port_selection(selected_model, coords):
        """Check whether a port was clicked on

        The methods checks whether the user clicked on a connector of a port. If the passed selected_model is a state,
        we have to check the positions of all port connectors of that state. If it is a data port, we only have to
        look at the connector position of that port.

        :param gtkmvc.Model selected_model: The selected_model that was clicked on
        :param tuple coords: Coordinates to search for ports
        """
        if isinstance(selected_model, (DataPortModel, ScopedVariableModel)):
            port_model = selected_model
            prefix = '' if isinstance(port_model, ScopedVariableModel) else 'inner_'
            connector_pos = port_model.temp['gui']['editor'][prefix + 'connector_pos']
            connector_radius = port_model.temp['gui']['editor'][prefix + 'connector_radius']
            if dist(connector_pos, coords) < connector_radius:
                selected_port_type = "inner" if isinstance(port_model, DataPortModel) else "scope"
                return port_model, selected_port_type, True

        elif isinstance(selected_model, AbstractStateModel):
            state_m = selected_model

            for port_m in itertools.chain(state_m.input_data_ports, state_m.output_data_ports):
                connector_pos = port_m.temp['gui']['editor']['outer_connector_pos']
                connector_radius = port_m.temp['gui']['editor']['outer_connector_radius']
                if dist(connector_pos, coords) < connector_radius:
                    return port_m, "outer", True
        return None, None, False

    @lock_state_machine
    def _check_for_resizer_selection(self, selected_model, coords):
        """Check whether a resizer (handle to resize a state) was clicked on

        Checks whether the current selection is a state and if so looks the given coordinates are within the resizer
        of that state. If so, the resizer (or its state model) is stored.

        :param gtkmvc.Model selected_model: The selected_model that was clicked on
        :param tuple coords: Coordinates to check for the resizer
        """
        if isinstance(selected_model, AbstractStateModel):
            state_meta = selected_model.get_meta_data_editor(for_gaphas=False)
            state_temp = selected_model.temp['gui']['editor']
            # Calculate corner points of resizer
            p1 = (state_temp['pos'][0] + state_meta['size'][0], state_temp['pos'][1] - state_meta['size'][1])
            p2 = (p1[0] - state_temp['resize_length'], p1[1])
            p3 = (p1[0], p1[1] + state_temp['resize_length'])

            # The resizer is triangle. Check whether the given coordinates are within that triangle
            if point_in_triangle(coords, p1, p2, p3):
                self.selected_resizer = selected_model

    @lock_state_machine
    def _check_for_multi_selection(self):
        """Select all models beneath the selection frame

        The user can multi-select models with a selection frame. This method searched for all models that are
        completely within the selection frame and adds them to the current selection.
        """
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
                    if frame_left < left <= right < frame_right and frame_bottom < bottom <= top < frame_top:
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
                self.model.set(selected_models)
        # If so, select models beneath frame
        self.multi_selection_started = False
        self.model.temp['gui']['editor']['selection_frame'] = None

    @lock_state_machine
    def _check_for_waypoint_removal(self, coords, connection_m):
        """Checks and removes a waypoint if necessary

        Checks whether the coordinates given are close to a waypoint of the given connection model (transition or
        data flow). If so, the waypoint is removed.

        :param tuple coords: Coordinates to check for a waypoint
        :param rafcon.gui.models.state_element.StateElementModel connection_m: Model of a transition or data flow
        :return: True, if a waypoint was removed, False else
        :rtype: bool
        """
        close_threshold = min(connection_m.parent.get_meta_data_editor(for_gaphas=False)['size']) / 70.
        # Check distance between all waypoints of the connection to the given coordinates
        for waypoint in connection_m.get_meta_data_editor(for_gaphas=False)['waypoints']:
            waypoint_pos = self._get_absolute_position(connection_m.parent, waypoint)
            if dist(waypoint_pos, coords) < close_threshold:
                waypoints = copy(connection_m.get_meta_data_editor(for_gaphas=False)['waypoints'])
                waypoints.remove(waypoint)
                connection_m.set_meta_data_editor('waypoints', waypoints, from_gaphas=False)
                self._meta_data_changed(connection_m, "waypoint_remove", False)
                self._redraw()
                return True
        return False

    @lock_state_machine
    def _draw_multi_selection_frame(self):
        """Shows a selection frame over the state machine
        """
        corner1 = self.mouse_move_start_coords
        corner2 = self.mouse_move_last_coords
        self.model.temp['gui']['editor']['selection_frame'] = [corner1, corner2]
        self._redraw()

    @lock_state_machine
    def _add_waypoint(self, connection_m, coords):
        """Adds a waypoint to the given connection

        The methods adds a waypoint at the given coordinates to the given connection (transition or data flow). If
        the connection also has waypoints, it puts the new one between the correct existing ones.

        :param rafcon.gui.models.state_element.StateElementModel connection_m: The model of the connection to add a
          waypoint to
        :param tuple coords: The coordinates of the new waypoint
        """
        connection_temp = connection_m.temp['gui']['editor']
        parent_state_m = connection_m.parent
        # The waypoints should exist as list. If not (for any reason), we have to convert it to one
        if isinstance(connection_m.get_meta_data_editor(for_gaphas=False)['waypoints'], dict):
            # logger.warn("Connection waypoints was of type dict, expected list")
            # connection_m.meta['gui']['editor']['waypoints'] = connection_m.meta['waypoints'].items()
            connection_m.set_meta_data_editor('waypoints', [], from_gaphas=False)
        waypoint_list = copy(connection_m.get_meta_data_editor(for_gaphas=False)['waypoints'])

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
                break
        logger.debug('Connection waypoint added at rel pos {0:.1f} | {1:.1f} (abs pos {0:.1f} | {1:.1f})'.format(
                rel_coords[0], rel_coords[1], coords[0], coords[1]))
        connection_m.set_meta_data_editor('waypoints', waypoint_list, from_gaphas=False)

        self._meta_data_changed(connection_m, "waypoint_add", False)
        self._redraw()

    @lock_state_machine
    def _create_new_transition(self, to_state_m, to_outcome_id=None):
        """Tries to create a new transition

        The user can create new transition using drag and drop in the graphical editor. The method uses the stored
        selected outcome as starting point and the passed state model and outcome id as target point for the new
        transition.

        :param rafcon.gui.models.abstract_state.AbstractStateModel to_state_m: The to state model of the new transition
        :param int to_outcome_id: The id of the to outcome or None if the transition does not go to the parent state
        """
        from_state_m = self.selected_outcome[0]
        from_state_id = from_state_m.state.state_id
        from_outcome_id = self.selected_outcome[1]
        to_state_id = to_state_m.state.state_id
        # Prevent accidental creation of transitions with double click on one outcome
        if from_state_id == to_state_id and from_outcome_id == to_outcome_id:
            self._abort()
            return

        # Start transition
        if from_outcome_id is None:
            from_state_id = None

        if from_outcome_id is None:  # Origin is income
            responsible_parent_m = from_state_m
        else:
            responsible_parent_m = from_state_m.parent

        try:
            if not isinstance(responsible_parent_m, ContainerStateModel):
                logger.warn("Only container states can have inner transitions.")
            else:
                transition_id = responsible_parent_m.state.add_transition(from_state_id, from_outcome_id, to_state_id,
                                                                          to_outcome_id)
                if self.temporary_waypoints:
                    transition_m = responsible_parent_m.get_transition_m(transition_id)
                    transition_m.set_meta_data_editor('waypoints', self.temporary_waypoints, from_gaphas=False)
                    self._meta_data_changed(model=transition_m, change='append_to_last_change')
                elif from_state_id == to_state_id:
                    insert_self_transition_meta_data(responsible_parent_m.states[from_state_id], transition_id,
                                                     combined_action=True)
        except (AttributeError, ValueError) as e:
            logger.warn("Transition couldn't be added: {0}".format(e))
            # import traceback
            # logger.debug("The graphical editor had an internal error: %s %s" % (str(e), str(traceback.format_exc())))
        except Exception as e:
            logger.error("Unexpected exception while creating transition: {0}".format(e))
            # import traceback
            # logger.debug("The graphical editor had an internal error: %s %s" % (str(e), str(traceback.format_exc())))

        self._abort()

    @lock_state_machine
    def _create_new_data_flow(self, target_port_m):
        """Tries to create a new data flow

        The user can create new data flow using drag and drop in the graphical editor. The method uses the stored
        selected port as starting point and the passed target port model as target point for the new data flow.

        :param rafcon.gui.models.data_port.DataPortModel target_port_m: The target port of the data flow
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
                if self.temporary_waypoints:
                    data_flow_m = responsible_parent.get_data_flow_m(data_flow_id)
                    data_flow_m.set_meta_data_editor('waypoints', self.temporary_waypoints, from_gaphas=False)
                    self._meta_data_changed(model=data_flow_m, change='append_to_last_change')
            except AttributeError as e:
                logger.warn("Data flow couldn't be added: {0}".format(e))
            except Exception as e:
                logger.error("Unexpected exception while creating data flow: {0}".format(e))

        self._abort()

    @lock_state_machine
    def _move_state(self, state_m, new_pos, redraw=True, publish_changes=True, combined_action=False):
        """Move the state to the given position

        The method moves the state and all its child states with their transitions, data flows and waypoints. The
        state is kept within its parent, thus restricting the movement.

        :param rafcon.gui.models.abstract_state.AbstractStateModel state_m: The model of the state to be moved
        :param tuple new_pos: The desired new absolute position (x, y)
        :param bool redraw: Flag whether to redraw state-machine after moving
        :param bool redraw: Flag whether to publish the changes after moving
        """
        if state_m.state.is_root_state:
            return

        cur_size = state_m.get_meta_data_editor(for_gaphas=False)['size']

        # Keep the state within its container state
        new_pos = self._limit_position_to_state(state_m.parent, new_pos, cur_size)

        parent_pos = state_m.parent.temp['gui']['editor']['pos']
        new_rel_pos = subtract_pos(new_pos, parent_pos)

        state_m.set_meta_data_editor('rel_pos', new_rel_pos, from_gaphas=False)

        if publish_changes:
            if combined_action:
                self._meta_data_changed(state_m, "append_to_last_change", affects_children=False)
            else:
                self._meta_data_changed(state_m, "position", affects_children=False)
        if redraw:
            self._redraw()

    @lock_state_machine
    def _move_data_port(self, port_m, new_pos, redraw=True, publish_changes=True):
        """Move the port to the given position

        This method moves the given port to the given coordinates, with respect to the mouse offset to the origin od
        the port and with respect to the size of the container state.

        :param rafcon.gui.models.data_port.DataPortModel port_m: The port model to be moved
        :param tuple new_pos: The desired new position (x, y)
        :param bool redraw: Flag whether to redraw state-machine after moving
        :param bool redraw: Flag whether to publish the changes after moving
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

        port_m.set_meta_data_editor('inner_rel_pos', new_rel_pos, from_gaphas=False)

        if publish_changes:
            self._meta_data_changed(port_m, "position", affects_children=False)
        if redraw:
            self._redraw()

    @lock_state_machine
    def _move_in_direction(self, direction, key, modifier):
        """Move the current selection into the given direction

        The method is the callback handler for arrow key presses. It moves the current selection into the direction
        of the pressed arrow key.

        :param rafcon.gui.views.graphical_editor.Direction direction: direction into which to move
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

        def move_state(state_m, redraw=True, publish_changes=True):
            if state_m.state.is_root_state:
                return
            cur_pos = state_m.temp['gui']['editor']['pos']
            new_pos = move_pos(cur_pos, state_m.parent.get_meta_data_editor(for_gaphas=False)['size'])
            self._move_state(state_m, new_pos, redraw, publish_changes)

        def move_port(port_m, redraw=True, publish_changes=True):
            cur_pos = port_m.temp['gui']['editor']['inner_pos']
            new_pos = move_pos(cur_pos, port_m.get_meta_data_editor(for_gaphas=False)['size'])
            self._move_data_port(port_m, new_pos, redraw, publish_changes)

        event = (key, modifier)
        if react_to_event(self.view, self.view.editor, event) or (len(event) == 2 and not isinstance(event[1], ModifierType)):
            if self.model.selection:
                for model in self.model.selection:
                    if isinstance(model, AbstractStateModel):
                        move_state(model, redraw=False, publish_changes=False)
                    elif isinstance(model, (DataPortModel, ScopedVariableModel)):
                        move_port(model, redraw=False, publish_changes=False)

                affects_children = len(self.model.selection) > 1
                if affects_children:
                    reduced_list = gui_helper_state_machine.reduce_to_parent_states(self.model.selection)
                    if len(reduced_list) > 1:
                        parent_m = self.root_state_m
                    else:
                        parent_m = reduced_list[0]
                else:
                    parent_m = self.model.selection[0]
                self._meta_data_changed(parent_m, "position", affects_children)
                self._redraw()
            elif isinstance(self.single_selection, AbstractStateModel):
                move_state(self.single_selection)
            elif isinstance(self.single_selection, (DataPortModel, ScopedVariableModel)):
                move_port(self.single_selection)
            return True  # Prevent shortcut from being passed to GTK
        return False  # Allow passing of shortcut

    @lock_state_machine
    def _move_waypoint(self, connection_m, waypoint_id, new_pos, snap=False, redraw=True, publish_changes=False):
        """Moves the currently selected waypoint to the given position

        :param connection_m: The model of the connection (transition or data flow)
        :param int waypoint_id: The id of the waypoint within the connection
        :param tuple new_pos: The new absolute position of the waypoint
        :param bool snap: Whether to snap the viapoints at certain angles
        :param bool redraw: Whether to redraw after updating the position
        :param bool publish_changes: Whether to publish the changes of the meta data
        """
        waypoints = copy(connection_m.get_meta_data_editor(for_gaphas=False)['waypoints'])
        connection_temp = connection_m.temp['gui']['editor']
        parent_state_m = connection_m.parent
        new_pos = self._limit_position_to_state(parent_state_m, new_pos)
        new_rel_pos = self._get_position_relative_to_state(parent_state_m, new_pos)

        # With the shift key pressed, try to snap the waypoint such that the connection has a multiple of 45 deg
        if snap:
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
        connection_m.set_meta_data_editor('waypoints', waypoints, from_gaphas=False)

        if publish_changes:
            self._meta_data_changed(connection_m, "waypoint_position", affects_children=False)
        if redraw:
            self._redraw()

    @lock_state_machine
    def _resize_state(self, state_m, new_corner_pos, keep_ratio=False, resize_content=False, publish_changes=False,
                      redraw=True):
        """Resize the state by the given delta width and height

        The resize function checks the child states and keeps the state around the children, thus limiting the minimum
        size. Two modifier keys can be used to alter the resize options:
         - Ctrl also causes the child states to be resized
         - Shift caused the resized states to keep their width to height ratio

        :param rafcon.gui.models.abstract_state.AbstractStateModel state_m: The model of the state to be resized
        :param tuple new_corner_pos: The absolute coordinates of the new desired lower right corner
        :param keep_ratio: Flag, if set, the size ratio is kept constant
        :param resize_content: Flag, if set, the content of the state is also resized
        :param redraw: Whether to call redraw() at the end or not
        """
        state_temp = state_m.temp['gui']['editor']
        state_meta = copy(state_m.get_meta_data_editor(for_gaphas=False))

        new_width = new_corner_pos[0] - state_temp['pos'][0]
        new_height = abs(new_corner_pos[1] - state_temp['pos'][1])

        # Keep size ratio?
        if keep_ratio:
            state_size_ratio = state_meta['size'][0] / state_meta['size'][1]
            new_state_size_ratio = new_width / new_height

            if new_state_size_ratio < state_size_ratio:
                new_height = new_width / state_size_ratio
            else:
                new_width = new_height * state_size_ratio

        if isinstance(state_m, LibraryStateModel):
            resize_content = True

        min_right_edge = state_temp['pos'][0]
        max_bottom_edge = state_temp['pos'][1]

        # If the content is not supposed to be resized, we have to calculate the inner edges, which define the
        # minimum size of our state
        if not resize_content and isinstance(state_m, ContainerStateModel):
            # Check lower right corner of all child states
            for child_state_m in state_m.states.itervalues():
                _, child_right_edge, child_bottom_edge, _ = self.get_boundaries(child_state_m)
                if child_right_edge is not None and child_bottom_edge is not None:
                    min_right_edge = child_right_edge if min_right_edge < child_right_edge else min_right_edge
                    max_bottom_edge = child_bottom_edge if max_bottom_edge > child_bottom_edge else max_bottom_edge
            # Check position of all waypoints of all transitions
            for transition_m in state_m.transitions:
                for waypoint in transition_m.get_meta_data_editor(for_gaphas=False)['waypoints']:
                    min_right_edge = waypoint[0] if min_right_edge < waypoint[0] else min_right_edge
                    max_bottom_edge = waypoint[1] if max_bottom_edge > waypoint[1] else max_bottom_edge
            # Check position of all waypoints of all data flows
            for data_flow_m in state_m.data_flows:
                for waypoint in data_flow_m.get_meta_data_editor(for_gaphas=False)['waypoints']:
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
        if not state_m.state.is_root_state:
            if state_m.state.is_root_state_of_library:
                parent_state_m = state_m.parent.parent
            else:
                parent_state_m = state_m.parent
            max_right_edge = parent_state_m.temp['gui']['editor']['pos'][0] + \
                             parent_state_m.get_meta_data_editor(for_gaphas=False)['size'][0]
            min_bottom_edge = parent_state_m.temp['gui']['editor']['pos'][1] - \
                              parent_state_m.get_meta_data_editor(for_gaphas=False)['size'][1]

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

        state_meta = state_m.set_meta_data_editor('size', (new_width, new_height), from_gaphas=False)

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

                if isinstance(state_m, LibraryStateModel):
                    root_state_m = state_m.state_copy
                    root_state_m.set_meta_data_editor('size', copy(new_size), from_gaphas=False)
                    if isinstance(root_state_m, ContainerStateModel):
                        resize_children(root_state_m, old_size, new_size)

                elif isinstance(state_m, ContainerStateModel):
                    # Resize all transitions
                    for transition_m in state_m.transitions:
                        # By repositioning all waypoints
                        for i, waypoint in enumerate(transition_m.get_meta_data_editor(for_gaphas=False)['waypoints']):
                            new_rel_pos = calc_new_rel_pos(waypoint, old_size, new_size)
                            transition_m.set_meta_data_editor('waypoints.{}'.format(i), new_rel_pos, from_gaphas=False)
                    # Resize all data flows
                    for data_flow_m in state_m.data_flows:
                        # By repositioning all waypoints
                        for i, waypoint in enumerate(data_flow_m.get_meta_data_editor(for_gaphas=False)['waypoints']):
                            new_rel_pos = calc_new_rel_pos(waypoint, old_size, new_size)
                            data_flow_m.set_meta_data_editor('waypoints.{}'.format(i), new_rel_pos, from_gaphas=False)

                    for port_m in itertools.chain(state_m.input_data_ports, state_m.output_data_ports,
                                                  state_m.scoped_variables):
                        old_rel_pos = port_m.get_meta_data_editor(for_gaphas=False)['inner_rel_pos']
                        new_rel_pos = calc_new_rel_pos(old_rel_pos, old_size, new_size)
                        port_m.set_meta_data_editor('inner_rel_pos', new_rel_pos, from_gaphas=False)

                    # Resize all child states
                    for child_state_m in state_m.states.itervalues():
                        old_rel_pos = child_state_m.get_meta_data_editor(for_gaphas=False)['rel_pos']
                        new_rel_pos = calc_new_rel_pos(old_rel_pos, old_size, new_size)
                        child_state_m.set_meta_data_editor('rel_pos', new_rel_pos, from_gaphas=False)

                        old_size = child_state_m.get_meta_data_editor(for_gaphas=False)['size']
                        new_size = (old_size[0] * width_factor, old_size[1] * height_factor)
                        child_state_m.set_meta_data_editor('size', new_size, from_gaphas=False)

                        if isinstance(child_state_m, (ContainerStateModel, LibraryStateModel)):
                            resize_children(child_state_m, old_size, new_size)

            # Start recursive call of the content resize
            resize_children(state_m, old_size, state_meta['size'])

        if publish_changes:
            self._meta_data_changed(state_m, "size", affects_children=resize_content)
        if redraw:
            self._redraw()

    @lock_state_machine
    def _move_view(self, rel_motion, opengl_coords=False):
        """Move the view according to the relative coordinates

        The whole view/scene is moved, causing the state machine to move within the viewport.

        :param tuple rel_motion: Distance to move (x, y)
        :param bool opengl_coords: Whether to specified relative coordinates are in OpenGl coordinate system
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

    @lock_state_machine
    def _handle_zooming(self, pos, direction):
        """Zooms in or out at a given position

        The method zooms increases or decreases the viewport, resulting in a zoom effect. The zoom keeps the current
        position of the cursor within the state machine, allowing to zoom in/out in specific directions.

        :param tuple pos: The mouse position at which the zoom occured
        :param direction: The scroll direction
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

    @lock_state_machine
    def draw_state_machine(self):
        """Draws remaining components of the state machine

        This method draws all other components, not directly belonging to a certain state. For a starter, this is the
        selection frame the user draws for a multi selection.
        """
        # Draw the multi selection frame
        frame = self.model.temp['gui']['editor']['selection_frame']
        if isinstance(frame, list):
            self.view.editor.draw_frame(frame[0], frame[1], 10)

    @ExtendedController.observe("destruction_signal", signal=True)
    def state_machine_destruction(self, model, prop_name, info):
        """ Close state editor when state machine is being destroyed """
        if self.model is model:
            self.suspend_drawing = True
            # self.relieve_all_models()

    @lock_state_machine
    def draw_state(self, state_m, rel_pos=(0, 0), size=(100, 100), depth=1):
        """Draws a (container) state with all its content

        Mainly contains the logic for drawing (e. g. reading and calculating values). The actual drawing process is
        done in the view, which is called from this method with the appropriate arguments.

        :param rafcon.gui.models.abstract_state.AbstractStateModel state_m: The state to be drawn
        :param tuple rel_pos: The default relative position (x, y) if there is no relative position stored
        :param tuple size: The default size (width, height) if there is no size stored
        :param float depth: The hierarchy level of the state
        """
        assert isinstance(state_m, AbstractStateModel)
        state_meta = state_m.get_meta_data_editor(for_gaphas=False)
        state_temp = state_m.temp['gui']['editor']
        is_first_draw_of_state = not contains_geometric_info(state_temp['pos'])
        redraw = False

        # Use default values if no size information is stored
        if not contains_geometric_info(state_meta['size']):

            if not isinstance(state_m, LibraryStateModel):
                state_meta = state_m.set_meta_data_editor('size', size, from_gaphas=False)
            else:
                # Try to get size from library (will later be resized to fit in state
                lib_size = state_m.state_copy.get_meta_data_editor(for_gaphas=False)['size']
                if contains_geometric_info(lib_size):
                    parent_size = state_m.parent.get_meta_data_editor(for_gaphas=False)['size']
                    if lib_size[0] >= parent_size[0] or lib_size[1] >= parent_size[1]:
                        target_size = parent_size[0] / 5., parent_size[1] / 5.
                        lib_size = calculate_size(lib_size, target_size)
                        lib_size = lib_size[0], lib_size[1]  # inverted y axis
                    state_meta = state_m.set_meta_data_editor('size', copy(lib_size), from_gaphas=False)
                else:
                    state_meta = state_m.set_meta_data_editor('size', size, from_gaphas=False)

        size = state_meta['size']
        # Root state is always in the origin
        if state_m.state.is_root_state:
            pos = (0, 0)
        else:
            # Use default values if no size information is stored
            if not contains_geometric_info(state_meta['rel_pos']):
                state_meta = state_m.set_meta_data_editor('rel_pos', rel_pos, from_gaphas=False)

                state_abs_pos = self._get_absolute_position(state_m.parent, rel_pos)
                self._move_state(state_m, state_abs_pos, redraw=False, publish_changes=False)
                redraw = True

            rel_pos = state_meta['rel_pos']
            pos = self._get_absolute_position(state_m.parent, rel_pos)

        state_temp['pos'] = pos

        # Was the state selected?
        selected_states = self.model.selection.states
        selected = False if state_m not in selected_states else True
        selected = selected if not (state_m.state.is_root_state_of_library and state_m.parent in selected_states) \
            else True

        # Is the state active (executing)?
        active = StateExecutionStatus.INACTIVE
        self_execution_status = state_m.state.state_execution_status
        if state_m.parent:
            parent_execution_status = state_m.parent.state.state_execution_status

        if self_execution_status is not StateExecutionStatus.INACTIVE or \
                (state_m.state.is_root_state_of_library and parent_execution_status is not StateExecutionStatus.INACTIVE):
            if isinstance(state_m, ContainerStateModel) and state_m.state.child_execution:
                active = StateExecutionStatus.EXECUTE_CHILDREN
            elif isinstance(state_m, LibraryStateModel) and not state_m.show_content():
                active = StateExecutionStatus.ACTIVE
            elif isinstance(state_m, LibraryStateModel) and isinstance(state_m.state_copy, ContainerStateModel) and \
                    state_m.state_copy.state.child_execution:
                active = StateExecutionStatus.EXECUTE_CHILDREN
            else:
                if self_execution_status is StateExecutionStatus.WAIT_FOR_NEXT_STATE:
                    active = StateExecutionStatus.WAIT_FOR_NEXT_STATE
                else:
                    active = StateExecutionStatus.ACTIVE

        is_child_of_library = False
        # Increase hierarchy level if child of a library state
        if not state_m.state.is_root_state and isinstance(state_m.parent.temp['gui']['editor']['library_level'], int):
            is_child_of_library = True
            if state_m.state.is_root_state_of_library:
                state_temp['library_level'] = state_m.parent.temp['gui']['editor']['library_level']
            else:
                state_temp['library_level'] = state_m.parent.temp['gui']['editor']['library_level'] + 1

        # Call the drawing method of the view
        # The view returns the id of the state in OpenGL and the positions of the outcomes, input and output ports
        (opengl_id, income_pos, outcome_pos, outcome_radius, resize_length) = self.view.editor.draw_state(
                state_m.state.name if not state_m.state.is_root_state_of_library else "", pos, size,
                state_m.state.outcomes,
                state_m.input_data_ports if global_runtime_config.get_config_value('SHOW_DATA_FLOWS', True) else [],
                state_m.output_data_ports if global_runtime_config.get_config_value('SHOW_DATA_FLOWS', True) else [],
                selected, active, depth)
        state_temp['id'] = opengl_id
        state_temp['income_pos'] = income_pos
        state_temp['outcome_pos'] = outcome_pos
        state_temp['outcome_radius'] = outcome_radius
        state_temp['resize_length'] = resize_length

        # If the state is a container state, we also have to draw its transitions and data flows as well as
        # recursively its child states
        if isinstance(state_m, ContainerStateModel):

            # Draw library states only up to a certain hierarchy level
            if is_child_of_library and state_temp['library_level'] >= global_gui_config.get_config_value(
                    'MAX_VISIBLE_LIBRARY_HIERARCHY', 2):
                # All states must be drawn once to create meta data
                if not is_first_draw_of_state:
                    return False
                redraw = True

            ratio = self.view.editor.pixel_to_size_ratio()
            minimum_size = global_gui_config.get_config_value('MINIMUM_SIZE_FOR_CONTENT', 30) / ratio
            if (size[0] < minimum_size or size[1] < minimum_size) and not is_first_draw_of_state:
                return False

            num_child_state = 0
            width = size[0]
            height = size[1]

            for child_state in state_m.states.itervalues():
                # Calculate default positions for the child states
                # Make the inset from the top left corner

                child_width = min(width, height) / 5.
                child_height = min(width, height) / 5.
                child_size = (child_width, child_height)
                child_spacing = max(child_size) * 1.2

                max_cols = width // child_spacing
                (row, col) = divmod(num_child_state, max_cols)
                child_rel_pos_x = col * child_spacing + child_spacing - child_width
                child_rel_pos_y = -child_spacing * (1.5 * row + 1)
                child_rel_pos = (child_rel_pos_x, child_rel_pos_y)
                num_child_state += 1

                redraw |= self.draw_state(child_state, child_rel_pos, child_size, depth + 1)

            if global_runtime_config.get_config_value('SHOW_DATA_FLOWS', True):
                self.draw_inner_data_ports(state_m, depth)

            self.draw_transitions(state_m, depth)

            if global_runtime_config.get_config_value('SHOW_DATA_FLOWS', True):
                self.draw_data_flows(state_m, depth)

            if isinstance(state_temp['template'], bool) and state_temp['template'] is True:
                parent_size = state_m.parent.get_meta_data_editor(for_gaphas=False)['size']
                new_size = calculate_size(state_meta['size'], (parent_size[0] / 5., parent_size[1] / 5.))
                new_size = new_size[0], -new_size[1]  # inverted y axis
                new_corner_pos = add_pos(state_m.temp['gui']['editor']['pos'], new_size)
                self._resize_state(state_m, new_corner_pos, keep_ratio=True, resize_content=True, redraw=False)
                state_temp['template'] = False
                redraw = True

        elif isinstance(state_m, LibraryStateModel):

            if state_m.show_content() or is_child_of_library:
                # Start calculation hierarchy level within a library
                if not is_child_of_library:
                    state_temp['library_level'] = 1

                # First draw inner states to generate meta data
                if state_m.state_copy:
                    import rafcon.gui.helpers.meta_data as gui_helper_meta_data
                    # logger.verbose("Scale meta data {0} {1}".format(not state_m.meta_data_was_scaled, state_m))
                    if not state_m.meta_data_was_scaled:
                        gui_helper_meta_data.scale_library_content(state_m, gaphas_editor=False)
                        state_m.meta_data_was_scaled = True
                    if isinstance(state_m.state_copy, ContainerStateModel):
                        self.draw_state(state_m.state_copy, (0, 0), size, depth)
                else:
                    redraw = False

        self._handle_new_transition(state_m, depth)

        if global_runtime_config.get_config_value('SHOW_DATA_FLOWS', True):
            self._handle_new_data_flow(state_m, depth)

        return redraw

    @lock_state_machine
    def draw_inner_data_ports(self, parent_state_m, parent_depth):
        """Draw the inner ports of a state

        This method draws the ports that are displayed within a container state. The inner ports are the input data
        ports, output data ports and scoped variables.

        :param rafcon.gui.models.container_state.ContainerStateModel parent_state_m: The parent state model of
            the ports
        :param float parent_depth: The depth of the parent state
        """
        parent_meta = parent_state_m.get_meta_data_editor(for_gaphas=False)
        max_rows = max(20, len(parent_state_m.input_data_ports), len(parent_state_m.output_data_ports))
        port_height = min(parent_meta['size']) / float(max_rows)
        max_port_width = min(parent_meta['size']) / 5.
        size = (max_port_width, port_height)

        # Input data ports
        num_input_ports = 1
        for port_m in parent_state_m.input_data_ports:
            port_meta = port_m.get_meta_data_editor(for_gaphas=False)
            port_temp = port_m.temp['gui']['editor']
            port = port_m.data_port
            if not contains_geometric_info(port_meta['inner_rel_pos']):
                # Put input ports by default in the lower left corner
                rel_pos = (0, -parent_meta['size'][1] + num_input_ports * port_height)
                port_meta = port_m.set_meta_data_editor('inner_rel_pos', rel_pos, from_gaphas=False)
            pos = self._get_absolute_position(parent_state_m, port_meta['inner_rel_pos'])
            port_temp['inner_pos'] = pos

            selected = port_m in self.model.selection.get_all()
            opengl_id = self.view.editor.draw_inner_input_data_port(port.name, port_m, pos, size, selected,
                                                                    parent_depth + 0.5)
            port_temp['id'] = opengl_id
            num_input_ports += 1

        # Output data ports
        num_output_ports = 1
        for port_m in parent_state_m.output_data_ports:
            port_meta = port_m.get_meta_data_editor(for_gaphas=False)
            port_temp = port_m.temp['gui']['editor']
            port = port_m.data_port
            if not contains_geometric_info(port_meta['inner_rel_pos']):
                # Put output ports by default in the lower right corner
                rel_pos = (parent_meta['size'][0], -parent_meta['size'][1] + num_output_ports * port_height)
                port_meta = port_m.set_meta_data_editor('inner_rel_pos', rel_pos, from_gaphas=False)
            pos = self._get_absolute_position(parent_state_m, port_meta['inner_rel_pos'])
            port_temp['inner_pos'] = pos

            selected = port_m in self.model.selection.get_all()
            opengl_id = self.view.editor.draw_inner_output_data_port(port.name, port_m, pos, size, selected,
                                                                     parent_depth + 0.5)
            port_temp['id'] = opengl_id
            num_output_ports += 1

        # Scoped variables
        num_scoped_variables = 0
        for port_m in parent_state_m.scoped_variables:
            port_meta = port_m.get_meta_data_editor(for_gaphas=False)
            port_temp = port_m.temp['gui']['editor']
            port = port_m.scoped_variable
            if not contains_geometric_info(port_meta['inner_rel_pos']):
                # Put scoped variables by default row-wise in at the top
                max_cols = parent_meta['size'][0] // max_port_width
                (row, col) = divmod(num_scoped_variables, max_cols)
                rel_pos = (col * max_port_width, -port_height * (2 * row + 1))
                port_meta = port_m.set_meta_data_editor('inner_rel_pos', rel_pos, from_gaphas=False)
            pos = self._get_absolute_position(parent_state_m, port_meta['inner_rel_pos'])
            port_temp['inner_pos'] = pos

            selected = port_m in self.model.selection.get_all()
            opengl_id = self.view.editor.draw_scoped_data_port(port.name, port_m, pos, size, selected,
                                                               parent_depth + 0.5)
            port_temp['id'] = opengl_id
            num_scoped_variables += 1

    @lock_state_machine
    def draw_transitions(self, parent_state_m, parent_depth):
        """Draws the transitions belonging to a state

        The method takes all transitions from the given state and calculates their start and end point positions.
        Those are passed together with the waypoints to the view of the graphical editor.

        :param rafcon.gui.models.container_state.ContainerStateModel parent_state_m: The model of the container
            state, of which the transitions shall be drawn
        :param float parent_depth: The depth of the container state
        """
        for transition_m in parent_state_m.transitions:
            # Get id and references to the from and to state
            from_state_id = transition_m.transition.from_state
            if from_state_id is None:
                from_pos = parent_state_m.temp['gui']['editor']['income_pos']
            else:
                from_state = parent_state_m.states[from_state_id]

                assert isinstance(from_state,
                                  AbstractStateModel), "Transition from unknown state with ID {id:s}".format(
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

            try:
                if to_state_id == parent_state_m.state.state_id:  # Transition goes back to parent
                    # Set the to coordinates to the outcome coordinates received earlier
                    to_pos = parent_state_m.temp['gui']['editor']['outcome_pos'][
                        transition_m.transition.to_outcome]
                else:
                    # Set the to coordinates to the center of the next state
                    to_state = parent_state_m.states[to_state_id]
                    to_pos = to_state.temp['gui']['editor']['income_pos']
            except KeyError:
                logger.warning("Transition could not be drawn because outcome-meta-data could not been reached "
                               "{0}".format(transition_m.transition))
                continue

            waypoints = []
            for waypoint in transition_m.get_meta_data_editor(for_gaphas=False)['waypoints']:
                waypoint_pos = self._get_absolute_position(parent_state_m, waypoint)
                waypoints.append(waypoint_pos)

            # Let the view draw the transition and store the returned OpenGL object id
            selected = False
            if transition_m in self.model.selection.transitions:
                selected = True
            line_width = self.view.editor.transition_stroke_width(parent_state_m)
            opengl_id = self.view.editor.draw_transition(from_pos, to_pos, line_width, waypoints,
                                                         selected, parent_depth + 0.5)
            transition_m.temp['gui']['editor']['id'] = opengl_id
            transition_m.temp['gui']['editor']['from_pos'] = from_pos
            transition_m.temp['gui']['editor']['to_pos'] = to_pos

    @lock_state_machine
    def draw_data_flows(self, parent_state_m, parent_depth):
        """Draw all data flows contained in the given container state

        The method takes all data flows from the given state and calculates their start and end point positions.
        Those are passed together with the waypoints to the view of the graphical editor.

        :param rafcon.gui.models.container_state.ContainerStateModel parent_state_m: The model of the container
            state, of which the data flows shall be drawn
        :param float parent_depth: The depth pf the container state
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

            from_port = from_state.get_data_port_m(from_key)
            to_port = to_state.get_data_port_m(to_key)

            if from_port is None:
                logger.warn('Cannot find model of the from data port {0}, ({1})'.format(from_key,
                                                                                        data_flow_m.data_flow))
                continue
            if to_port is None:
                logger.warn('Cannot find model of the to data port {0}, ({1})'.format(to_key, data_flow_m.data_flow))
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
            for waypoint in data_flow_m.get_meta_data_editor(for_gaphas=False)['waypoints']:
                waypoint_pos = self._get_absolute_position(parent_state_m, waypoint)
                waypoints.append(waypoint_pos)

            selected = False
            if data_flow_m in self.model.selection.data_flows:
                selected = True
            line_width = self.view.editor.data_flow_stroke_width(parent_state_m)
            opengl_id = self.view.editor.draw_data_flow(from_pos, to_pos, line_width, waypoints,
                                                        selected, parent_depth + 0.5)
            data_flow_m.temp['gui']['editor']['id'] = opengl_id
            data_flow_m.temp['gui']['editor']['from_pos'] = from_pos
            data_flow_m.temp['gui']['editor']['to_pos'] = to_pos

    @lock_state_machine
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

    @lock_state_machine
    def _handle_new_transition(self, parent_state_m, parent_depth):
        """Responsible for drawing new transition the user creates

        With drag and drop on outcomes, the user can draw new transitions. Here the transition is temporary drawn in
        the graphical editor.

        :param rafcon.gui.models.container_state.ContainerStateModel parent_state_m: Model of the container state
        :param float parent_depth: Depth of the container state
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
                    responsible_parent_m = parent_state_m if parent_state_m.state.is_root_state else \
                        parent_state_m.parent
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

    @lock_state_machine
    def _handle_new_data_flow(self, parent_state_m, parent_depth):
        """Responsible for drawing new data flows the user creates

        With drag and drop on ports, the user can draw new data flows. Here the data flow is temporary drawn in the
        graphical editor.

        :param rafcon.gui.models.container_state.ContainerStateModel parent_state_m: Model of the container state
        :param float parent_depth: Depth of the container state
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

    @lock_state_machine
    def _find_selection(self, pos_x, pos_y, width=6, height=6, all=False,
                        find_states=True, find_transitions=True, find_data_flows=True, find_data_ports=True):
        """Returns the model at the given position

        This method is used when the model (state/transition/data flow) the user clicked on is to be found. The
        position is told to OpenGl and the whole scene is redrawn. From the stack ob objects beneath the position,
        the uppermost one is returned.

        :param float pos_x: The x coordinate of the position
        :param float pos_y: The y coordinate of the position
        :param bool find_states: Flag whether to find states
        :param bool find_transitions: Flag whether to find transitions
        :param bool find_data_flows: Flag whether to find data flows
        :param bool find_data_ports: Flag whether to find data ports
        :return gtkmvc.Model: The uppermost model beneath the given position, None if nothing was found
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

    @lock_state_machine
    def _selection_ids_to_model(self, ids, search_state_m, search_state_depth, selection, selection_depth, all=False,
                                find_states=True, find_transitions=True, find_data_flows=True, find_data_ports=True):
        """Searches recursively for objects with the given ids

        The method searches recursively and compares all stored ids with the given ones. It finally returns the
        object with the biggest depth (furthest nested).

        :param ids: The ids to search for
        :param rafcon.gui.models.abstract_state.AbstractStateModel search_state_m: The state to search in
        :param float search_state_depth: The depth the search state is in
        :param gtkmvc.Model selection: The currently found object
        :param float selection_depth: The depth of the currently found object
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
            if search_state_m.temp['gui']['editor']['id'] and search_state_m.temp['gui']['editor']['id'] in ids:
                # if so, add the state to the list of selected states
                selection = update_selection(selection, search_state_m)
                selection_depth = search_state_depth
                # remove the id from the list to fasten up further searches
                ids.remove(search_state_m.temp['gui']['editor']['id'])

        # Return if there is nothing more to find
        if len(ids) == 0:
            return selection, selection_depth

        # If it is a container state, check its transitions, data flows and child states
        if isinstance(search_state_m, ContainerStateModel):

            for state in search_state_m.states.itervalues():
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
                selection = search_selection_in_model_list(search_state_m.transitions, selection)
                if len(ids) == 0:
                    return selection, selection_depth

            if find_data_flows:
                selection = search_selection_in_model_list(search_state_m.data_flows, selection)
                if len(ids) == 0:
                    return selection, selection_depth

            if find_data_ports:
                selection = search_selection_in_model_list(search_state_m.input_data_ports, selection)
                selection = search_selection_in_model_list(search_state_m.output_data_ports, selection)
                selection = search_selection_in_model_list(search_state_m.scoped_variables, selection)
        return selection, selection_depth

    @staticmethod
    def get_boundaries(model, include_waypoints=False):
        """Returns the boundaries (in OpenGL) coordinates of the given model

        :param rafcon.gui.models.meta.MetaModel model: Model for which to get the boundaries
        :return list: left, right, bottom, top
        """
        meta = model.get_meta_data_editor(for_gaphas=False)
        temp = model.temp['gui']['editor']
        if isinstance(model, AbstractStateModel):
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

    def _meta_data_changed(self, model, change, affects_children=False):
        # self.model.history.meta_changed_notify_after(model.parent, model, affects_children)
        msg = MetaSignalMsg('graphical_editor', change, affects_children)
        model.meta_signal.emit(msg)
        # logger.debug("publish changes to history")

    @lock_state_machine
    def _delete_selection(self, *event):
        if react_to_event(self.view, self.view.editor, event):
            return gui_helper_state_machine.delete_selected_elements(self.model)

    @lock_state_machine
    def _add_new_state(self, *event, **kwargs):
        """Triggered when shortcut keys for adding a new state are pressed, or Menu Bar "Edit, Add State" is clicked.

        Adds a new state only if the graphical editor is in focus.
        """
        if react_to_event(self.view, self.view.editor, event):
            state_type = StateType.EXECUTION if 'state_type' not in kwargs else kwargs['state_type']
            return gui_helper_state_machine.add_new_state(self.model, state_type)

    @lock_state_machine
    def _abort(self, *event):
        if react_to_event(self.view, self.view.editor, event):
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
                        model_meta = model.get_meta_data_editor(for_gaphas=False)
                        model_temp = model.temp['gui']['editor']
                        if isinstance(model, AbstractStateModel):
                            model.set_meta_data_editor('rel_pos', model_temp['original_rel_pos'], from_gaphas=False)
                        elif isinstance(model, (DataPortModel, ScopedVariableModel)):
                            model.set_meta_data_editor('inner_rel_pos', model_temp['original_inner_rel_pos'],
                                                       from_gaphas=False)
                        elif isinstance(model, (DataFlowModel, TransitionModel)):
                            waypoints = model_meta['waypoints']
                            if waypoints:
                                for waypoint_id, waypoint_pos in enumerate(waypoints):
                                    model.set_meta_data_editor('waypoints.{}'.format(waypoint_id), model_temp[
                                        'original_waypoint_{0}_rel_pos'.format(waypoint_id)], from_gaphas=False)
                elif isinstance(self.single_selection, AbstractStateModel):
                    self.single_selection.set_meta_data_editor('rel_pos', self.single_selection.temp['gui'][
                        'editor']['original_rel_pos'], from_gaphas=False)
                elif isinstance(self.single_selection, (DataPortModel, ScopedVariableModel)):
                    self.single_selection.set_meta_data_editor('inner_rel_pos', self.single_selection.temp['gui'][
                        'editor']['original_inner_rel_pos'], from_gaphas=False)

                self.last_button_pressed = None  # prevents further movements
                self._redraw()

    @lock_state_machine
    def _copy_selection(self, *event):
        """Copies the current selection to the clipboard.
        """
        if react_to_event(self.view, self.view.editor, event):
            logger.debug("copy selection")
            global_clipboard.copy(self.model.selection)
            return True

    @lock_state_machine
    def _cut_selection(self, *event):
        """Cuts the current selection and copys it to the clipboard.
        """
        if react_to_event(self.view, self.view.editor, event):
            logger.debug("cut selection")
            global_clipboard.cut(self.model.selection)
            return True

    @lock_state_machine
    def _paste_clipboard(self, *event):
        """Paste the current clipboard into the current selection if the current selection is a container state.
        """
        if react_to_event(self.view, self.view.editor, event):
            logger.debug("Paste")
            gui_helper_state_machine.paste_into_selected_state(self.model)
            return True

    def check_focus_and_sm_selection_according_event(self, event):
        if not react_to_event(self.view, self.view.editor, event):
            return False
        if not rafcon.gui.singleton.state_machine_manager_model.selected_state_machine_id == \
                self.model.state_machine.state_machine_id:
            return False
        return True

    @lock_state_machine
    def _add_data_port_to_selected_state(self, *event, **kwargs):
        if self.check_focus_and_sm_selection_according_event(event):
            data_port_type = None if 'data_port_type' not in kwargs else kwargs['data_port_type']
            gui_helper_state_machine.add_data_port_to_selected_states(data_port_type)

    @lock_state_machine
    def _add_scoped_variable_to_selected_state(self, *event):
        if self.check_focus_and_sm_selection_according_event(event):
            gui_helper_state_machine.add_scoped_variable_to_selected_states()

    @lock_state_machine
    def _add_outcome_to_selected_state(self, *event):
        if self.check_focus_and_sm_selection_according_event(event):
            gui_helper_state_machine.add_outcome_to_selected_states()
