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

"""
.. module:: graphical_editor_gaphas
   :synopsis: A module that holds the controller to graphically edit a state machine by the gaphas library.

"""

from gi.repository import Gtk
from gi.repository import Gdk
from gi.repository import GLib
from future.utils import string_types
import time
from functools import partial
from gaphas.aspect import InMotion, ItemFinder
from gaphas.item import Item
import math

from rafcon.core.decorators import lock_state_machine
from rafcon.core.states.state import StateType
from rafcon.gui.clipboard import global_clipboard
from rafcon.gui.controllers.utils.extended_controller import ExtendedController

from rafcon.gui.helpers.label import react_to_event
from rafcon.gui.helpers.meta_data import generate_default_state_meta_data
import rafcon.gui.utils.constants as gui_constants
from rafcon.gui.models import ContainerStateModel, AbstractStateModel, TransitionModel, DataFlowModel
from rafcon.gui.models.scoped_variable import ScopedVariableModel
from rafcon.gui.models.library_state import LibraryStateModel
from rafcon.gui.models.signals import MetaSignalMsg
from rafcon.gui.models.state_machine import StateMachineModel
from rafcon.gui.mygaphas.canvas import MyCanvas
# noinspection PyUnresolvedReferences
from rafcon.gui.mygaphas import guide
from rafcon.gui.mygaphas.items.connection import DataFlowView, TransitionView
from rafcon.gui.mygaphas.items.ports import OutcomeView, DataPortView, ScopedVariablePortView
from rafcon.gui.mygaphas.items.state import StateView, NameView
import rafcon.gui.singleton
from rafcon.gui.views.graphical_editor_gaphas import GraphicalEditorView
import rafcon.gui.helpers.meta_data as gui_helper_meta_data
import rafcon.gui.helpers.state_machine as gui_helper_state_machine

from rafcon.utils import log

logger = log.get_logger(__name__)


class GraphicalEditorController(ExtendedController):
    """Controller handling the graphical editor

    :param rafcon.gui.models.state_machine.StateMachineModel model: The state machine model, holding the root
        state and the current selection
    :param rafcon.gui.views.graphical_editor.GraphicalEditorView view: The GTK view having an OpenGL rendering
        element
    """

    drag_motion_handler_id = None
    focus_changed_handler_id = None

    def __init__(self, model, view):
        """Constructor"""
        start_time = time.time()
        ExtendedController.__init__(self, model, view)
        assert type(view) == GraphicalEditorView
        assert isinstance(self.model, StateMachineModel)
        self.observe_model(rafcon.gui.singleton.gui_config_model)
        self.observe_model(rafcon.gui.singleton.runtime_config_model)
        self.root_state_m = model.root_state

        self.canvas = MyCanvas()
        self.zoom = 3.
        self.perform_drag_and_drop = False

        view.setup_canvas(self.canvas, self.zoom)

        view.editor.drag_dest_set(Gtk.DestDefaults.ALL, None, Gdk.DragAction.COPY)
        view.editor.drag_dest_add_text_targets()
        logger.verbose("Time spent in init {0} seconds for state machine {1}"
                       "".format(time.time() - start_time, self.model.state_machine_id))

    def destroy(self):
        if self.view:
            self.view.editor.prepare_destruction()
        super(GraphicalEditorController, self).destroy()

    def register_view(self, view):
        """Called when the View was registered"""
        super(GraphicalEditorController, self).register_view(view)

        self.view.connect('meta_data_changed', self._meta_data_changed)
        self.focus_changed_handler_id = self.view.editor.connect('focus-changed', self._move_focused_item_into_viewport)
        self.view.editor.connect("drag-data-received", self.on_drag_data_received)
        self.drag_motion_handler_id = self.view.editor.connect("drag-motion", self.on_drag_motion)

        try:
            self.setup_canvas()
        except:
            logger.exception("Error while setting up canvas")

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.gui.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
            between shortcuts and actions.
        """
        shortcut_manager.add_callback_for_action("add", partial(self._add_new_state, state_type=StateType.EXECUTION))
        shortcut_manager.add_callback_for_action("add_execution_state", partial(self._add_new_state,
                                                                                state_type=StateType.EXECUTION))
        shortcut_manager.add_callback_for_action("add_hierarchy_state", partial(self._add_new_state,
                                                                                state_type=StateType.HIERARCHY))
        shortcut_manager.add_callback_for_action("add_barrier_state", partial(self._add_new_state,
                                                                              state_type=StateType.BARRIER_CONCURRENCY))
        shortcut_manager.add_callback_for_action("add_preemptive_state", partial(self._add_new_state,
                                                                                 state_type=StateType.PREEMPTION_CONCURRENCY))

        shortcut_manager.add_callback_for_action("add_output", partial(self._add_data_port_to_selected_state,
                                                                       data_port_type='OUTPUT'))
        shortcut_manager.add_callback_for_action("add_input", partial(self._add_data_port_to_selected_state,
                                                                      data_port_type='INPUT'))
        shortcut_manager.add_callback_for_action("add_scoped_variable", self._add_scoped_variable_to_selected_state)
        shortcut_manager.add_callback_for_action("add_outcome", self._add_outcome_to_selected_state)

        shortcut_manager.add_callback_for_action("delete", self._remove_selected_elements)

        shortcut_manager.add_callback_for_action("copy", self._copy_selection)
        shortcut_manager.add_callback_for_action("paste", self._paste_clipboard)
        shortcut_manager.add_callback_for_action("cut", self._cut_selection)

        shortcut_manager.add_callback_for_action('show_data_flows', self.update_view)
        shortcut_manager.add_callback_for_action('show_data_values', self.update_view)
        shortcut_manager.add_callback_for_action('data_flow_mode', self.data_flow_mode)
        shortcut_manager.add_callback_for_action('show_aborted_preempted', self.update_view)

    @ExtendedController.observe('config', after=True)
    def on_config_value_changed(self, config_m, prop_name, info):
        """Callback when a config value has been changed

        :param ConfigModel config_m: The config model that has been changed
        :param str prop_name: Should always be 'config'
        :param dict info: Information e.g. about the changed config key
        """
        config_key = info['args'][1]
        # config_value = info['args'][2]

        if config_key in ["ENABLE_CACHING", "SHOW_ABORTED_PREEMPTED", "SHOW_DATA_FLOWS",
                          "SHOW_DATA_FLOW_VALUE_LABELS", "SHOW_NAMES_ON_DATA_FLOWS", "ROTATE_NAMES_ON_CONNECTION"]:
            self.update_view()

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
        state_id_insert = data.get_text()
        parent_m = self.model.selection.get_selected_state()
        if not isinstance(parent_m, ContainerStateModel):
            return
        state_v = self.canvas.get_view_for_model(parent_m.states[state_id_insert])
        pos_start = state_v.model.get_meta_data_editor()['rel_pos']
        motion = InMotion(state_v, self.view.editor)
        motion.start_move(self.view.editor.get_matrix_i2v(state_v).transform_point(pos_start[0], pos_start[1]))
        motion.move((x, y))
        motion.stop_move()
        state_v.model.set_meta_data_editor('rel_pos', motion.item.position)
        self.canvas.wait_for_update(trigger_update=True)
        self._meta_data_changed(None, state_v.model, 'append_to_last_change', True)

    @lock_state_machine
    def on_drag_motion(self, widget, context, x, y, time):
        """Changes the selection on mouse over during drag motion

        :param widget:
        :param context:
        :param x: Integer: x-position of mouse
        :param y: Integer: y-position of mouse
        :param time:
        """
        hovered_item = ItemFinder(self.view.editor).get_item_at_point((x, y))
        if isinstance(hovered_item, NameView):
            hovered_item = hovered_item.parent
        if hovered_item is None:
            self.view.editor.unselect_all()
        elif isinstance(hovered_item.model, ContainerStateModel):
            if len(self.view.editor.selected_items) == 1 and hovered_item in self.view.editor.selected_items:
                return
            if len(self.view.editor.selected_items) > 0:
                self.view.editor.unselect_all()

            if not rafcon.gui.singleton.global_gui_config.get_config_value('DRAG_N_DROP_WITH_FOCUS'):
                self.view.editor.handler_block(self.focus_changed_handler_id)
            self.view.editor.focused_item = hovered_item
            if not rafcon.gui.singleton.global_gui_config.get_config_value('DRAG_N_DROP_WITH_FOCUS'):
                self.view.editor.handler_unblock(self.focus_changed_handler_id)

    def update_view(self, *args, **kwargs):
        self.canvas.update_root_items()

    @lock_state_machine
    def data_flow_mode(self, *args, **kwargs):
        pass

    @lock_state_machine
    def _add_new_state(self, *event, **kwargs):
        """Triggered when shortcut keys for adding a new state are pressed, or Menu Bar "Edit, Add State" is clicked.

        Adds a new state only if the graphical editor is in focus.
        """
        if react_to_event(self.view, self.view.editor, event):
            state_type = StateType.EXECUTION if 'state_type' not in kwargs else kwargs['state_type']
            cursor_position = kwargs.get("cursor_position", None)
            if cursor_position:
                from rafcon.gui.helpers.coordinates import main_window2graphical_editor
                cursor_position = main_window2graphical_editor(cursor_position)
            return gui_helper_state_machine.add_new_state(self.model, state_type, cursor_position)

    @lock_state_machine
    def _copy_selection(self, *event, **kwargs):
        """Copies the current selection to the clipboard.
        """
        if react_to_event(self.view, self.view.editor, event):
            logger.verbose("copy selection")
            global_clipboard.copy(self.model.selection)
            return True

    @lock_state_machine
    def _cut_selection(self, *event, **kwargs):
        """Cuts the current selection and copies it to the clipboard.
        """
        if react_to_event(self.view, self.view.editor, event):
            logger.verbose("cut selection")
            global_clipboard.cut(self.model.selection)
            return True

    @lock_state_machine
    def _paste_clipboard(self, *event, **kwargs):
        """Paste the current clipboard into the current selection if the current selection is a container state.
        """
        if react_to_event(self.view, self.view.editor, event):
            logger.verbose("Paste")
            cursor_position = kwargs.get("cursor_position", None)

            gui_helper_state_machine.paste_into_selected_state(self.model, cursor_position)
            return True

    def _move_focused_item_into_viewport(self, view, focused_item):
        """Called when an item is focused, moves the item into the viewport

        :param view:
        :param StateView | ConnectionView | PortView focused_item: The focused item
        """
        self.view.editor.handler_block(self.drag_motion_handler_id)
        self.move_item_into_viewport(focused_item)
        self.view.editor.handler_unblock(self.drag_motion_handler_id)

    def move_item_into_viewport(self, item):
        """Causes the `item` to be moved into the viewport

        The zoom factor and the position of the viewport are updated to move the `item` into the viewport. If `item`
        is not a `StateView`, the parental `StateView` is moved into the viewport.

        :param StateView | ConnectionView | PortView item: The item to be moved into the viewport
        """
        if not item:
            return
        HORIZONTAL = 0
        VERTICAL = 1
        if not isinstance(item, Item):
            state_v = item.parent
        elif not isinstance(item, StateView):
            state_v = self.canvas.get_parent(item)
        else:
            state_v = item
        viewport_size = self.view.editor.get_allocation().width, self.view.editor.get_allocation().height
        state_size = self.view.editor.get_matrix_i2v(state_v).transform_distance(state_v.width, state_v.height)
        min_relative_size = min(viewport_size[i] / state_size[i] for i in [HORIZONTAL, VERTICAL])

        if min_relative_size != 1:
            # Allow margin around state
            margin_relative = 1. / gui_constants.BORDER_WIDTH_STATE_SIZE_FACTOR
            zoom_factor = min_relative_size * (1 - margin_relative)
            if zoom_factor > 1:
                zoom_base = 4
                zoom_factor = max(1, math.log(zoom_factor*zoom_base, zoom_base))
            self.view.editor.zoom(zoom_factor)
            # The zoom operation must be performed before the pan operation to work on updated GtkAdjustments (scroll
            # bars)
            self.canvas.wait_for_update()

        state_pos = self.view.editor.get_matrix_i2v(state_v).transform_point(0, 0)
        state_size = self.view.editor.get_matrix_i2v(state_v).transform_distance(state_v.width, state_v.height)
        viewport_size = self.view.editor.get_allocation().width, self.view.editor.get_allocation().height

        # Calculate offset around state so that the state is centered in the viewport
        padding_offset_horizontal = (viewport_size[HORIZONTAL] - state_size[HORIZONTAL]) / 2.
        padding_offset_vertical = (viewport_size[VERTICAL] - state_size[VERTICAL]) / 2.
        self.view.editor.hadjustment.set_value(state_pos[HORIZONTAL] - padding_offset_horizontal)
        self.view.editor.vadjustment.set_value(state_pos[VERTICAL] - padding_offset_vertical)

    def _meta_data_changed(self, view, model, name, affects_children):
        msg = MetaSignalMsg('graphical_editor_gaphas', name, affects_children)
        model.meta_signal.emit(msg)

    @ExtendedController.observe("destruction_signal", signal=True)
    def state_machine_destruction(self, model, prop_name, info):
        """ Clean up when state machine is being destructed """
        if self.model is model:  # only used for the state machine destruction case
            self.canvas.get_view_for_model(self.root_state_m).remove()

    @ExtendedController.observe("state_meta_signal", signal=True)
    def meta_changed_notify_after(self, state_machine_m, _, info):
        """Handle notification about the change of a state's meta data

        The meta data of the affected state(s) are read and the view updated accordingly.
        :param StateMachineModel state_machine_m: Always the state machine model belonging to this editor
        :param str _: Always "state_meta_signal"
        :param dict info: Information about the change, contains the MetaSignalMessage in the 'arg' key value
        """
        meta_signal_message = info['arg']
        if meta_signal_message.origin == "graphical_editor_gaphas":  # Ignore changes caused by ourself
            return
        if meta_signal_message.origin == "load_meta_data":  # Meta data can't be applied, as the view has not yet
            return                                          # been created
        notification = meta_signal_message.notification
        if not notification:    # For changes applied to the root state, there are always two notifications
            return              # Ignore the one with less information
        if self.model.ongoing_complex_actions:
            return

        model = notification.model
        view = self.canvas.get_view_for_model(model)

        if meta_signal_message.change == 'show_content':
            library_state_m = model
            library_state_v = view
            if library_state_m.meta['gui']['show_content'] is not library_state_m.show_content():
                logger.warning("The content of the LibraryState won't be shown, because "
                               "MAX_VISIBLE_LIBRARY_HIERARCHY is 1.")
            if library_state_m.show_content():
                if not library_state_m.state_copy_initialized:
                    logger.warning("Show library content without initialized state copy does not work {0}"
                                   "".format(library_state_m))
                logger.debug("Show content of {}".format(library_state_m.state))
                gui_helper_meta_data.scale_library_content(library_state_m)
                self.add_state_view_for_model(library_state_m.state_copy, view,
                                              hierarchy_level=library_state_v.hierarchy_level + 1)
            else:
                logger.debug("Hide content of {}".format(library_state_m.state))
                state_copy_v = self.canvas.get_view_for_model(library_state_m.state_copy)
                if state_copy_v:
                    state_copy_v.remove()
        else:
            if isinstance(view, StateView):
                view.apply_meta_data(recursive=meta_signal_message.affects_children)
            else:
                view.apply_meta_data()

        self.canvas.request_update(view, matrix=True)
        self.canvas.wait_for_update()

    @ExtendedController.observe("ongoing_complex_actions", after=True)
    def update_of_ongoing_complex_actions(self, model, prop_name, info):
        # only once at the end of an complex action the ongoing complex actions dictionary is empty
        if not model.ongoing_complex_actions:
            action_name, action_dict = self.model.complex_action_observer.nested_action_already_in[-1]
            self.adapt_complex_action(action_dict['target'], action_dict['new'])

    @ExtendedController.observe("state_machine", after=True)
    def state_machine_change_after(self, model, prop_name, info):
        """Called on any change within th state machine

        This method is called, when any state, transition, data flow, etc. within the state machine changes. This
        then typically requires a redraw of the graphical editor, to display these changes immediately.

        :param rafcon.gui.models.state_machine.StateMachineModel model: The state machine model
        :param str prop_name: The property that was changed
        :param dict info: Information about the change
        """

        if 'method_name' in info and info['method_name'] == 'root_state_change':
            method_name, model, result, arguments, instance = self._extract_info_data(info['kwargs'])

            if self.model.ongoing_complex_actions:
                return

            # The method causing the change raised an exception, thus nothing was changed
            if (isinstance(result, string_types) and "CRASH" in result) or isinstance(result, Exception):
                return

            # avoid to remove views of elements of states which parent state is destroyed recursively
            if 'remove' in method_name:
                # for remove the model is always a state and in case of remove_state it is the container_state
                # that performs the operation therefore if is_about_to_be_destroyed_recursively is False
                # the child state can be removed and for True ignored because its parent will create a notification
                if model.is_about_to_be_destroyed_recursively:
                    return

            # only react to the notification if the model is a model, which has to be drawn
            # if it is a model inside a library state, this is eventually not the case
            if isinstance(model, AbstractStateModel):
                library_root_state = model.state.get_next_upper_library_root_state()
                if library_root_state:
                    parent_library_root_state_m = self.model.get_state_model_by_path(library_root_state.get_path())
                    if not parent_library_root_state_m.parent.show_content():
                        return

            if method_name == 'state_execution_status':
                state_v = self.canvas.get_view_for_model(model)
                if state_v:  # Children of LibraryStates are not modeled, yet
                    self.canvas.request_update(state_v, matrix=False)
            elif method_name == 'add_state':
                new_state = arguments[1]
                new_state_m = model.states[new_state.state_id]
                self.add_state_view_with_meta_data_for_model(new_state_m, model)
                if not self.perform_drag_and_drop:
                    self.canvas.wait_for_update()
            elif method_name == 'remove_state':
                state_v = self.canvas.get_view_for_core_element(result)
                if state_v:
                    parent_v = self.canvas.get_parent(state_v)
                    state_v.remove()
                    if parent_v:
                        self.canvas.request_update(parent_v)
                    self.canvas.wait_for_update()

            # ----------------------------------
            #           TRANSITIONS
            # ----------------------------------
            elif method_name == 'add_transition':
                transitions_models = model.transitions
                transition_id = result
                for transition_m in transitions_models:
                    if transition_m.transition.transition_id == transition_id:
                        self.add_transition_view_for_model(transition_m, model)
                        self.canvas.wait_for_update()
                        break
            elif method_name == 'remove_transition':
                transition_v = self.canvas.get_view_for_core_element(result)
                if transition_v:
                    state_m = model
                    state_v = self.canvas.get_view_for_model(state_m)
                    transition_v.remove()
                    self.canvas.request_update(state_v, matrix=False)
                    self.canvas.wait_for_update()
            elif method_name == 'transition_change':
                transition_m = model
                transition_v = self.canvas.get_view_for_model(transition_m)
                self._reconnect_transition(transition_v, transition_m, transition_m.parent)
                self.canvas.wait_for_update()

            # ----------------------------------
            #           DATA FLOW
            # ----------------------------------
            elif method_name == 'add_data_flow':
                data_flow_models = model.data_flows
                data_flow_id = result
                for data_flow_m in data_flow_models:
                    if data_flow_m.data_flow.data_flow_id == data_flow_id:
                        self.add_data_flow_view_for_model(data_flow_m, model)
                        self.canvas.wait_for_update()
                        break
            elif method_name == 'remove_data_flow':
                data_flow_v = self.canvas.get_view_for_core_element(result)
                if data_flow_v:
                    state_m = model
                    state_v = self.canvas.get_view_for_model(state_m)
                    self.canvas.request_update(state_v, matrix=False)
                    data_flow_v.remove()
                    self.canvas.wait_for_update()
            elif method_name == 'data_flow_change':
                data_flow_m = model
                data_flow_v = self.canvas.get_view_for_model(data_flow_m)
                self._reconnect_data_flow(data_flow_v, data_flow_m, data_flow_m.parent)
                self.canvas.wait_for_update()

            # ----------------------------------
            #           OUTCOMES
            # ----------------------------------
            elif method_name == 'add_outcome':
                state_m = model
                state_v = self.canvas.get_view_for_model(state_m)
                for outcome_m in state_m.outcomes:
                    if outcome_m.outcome.outcome_id == result:
                        state_v.add_outcome(outcome_m)
                        self.canvas.request_update(state_v, matrix=False)
                        self.canvas.wait_for_update()
                        break
            elif method_name == 'remove_outcome':
                state_m = model
                state_v = self.canvas.get_view_for_model(state_m)
                if state_v is None:
                    logger.debug("no state_v found for method_name '{}'".format(method_name))
                else:
                    outcome_v = self.canvas.get_view_for_core_element(result)
                    if outcome_v:
                        state_v.remove_outcome(outcome_v)
                        self.canvas.request_update(state_v, matrix=False)
                        self.canvas.wait_for_update()

            # ----------------------------------
            #           DATA PORTS
            # ----------------------------------
            elif method_name == 'add_input_data_port':
                state_m = model
                state_v = self.canvas.get_view_for_model(state_m)
                for input_data_port_m in state_m.input_data_ports:
                    if input_data_port_m.data_port.data_port_id == result:
                        state_v.add_input_port(input_data_port_m)
                        self.canvas.request_update(state_v, matrix=False)
                        self.canvas.wait_for_update()
                        break
            elif method_name == 'add_output_data_port':
                state_m = model
                state_v = self.canvas.get_view_for_model(state_m)
                for output_data_port_m in state_m.output_data_ports:
                    if output_data_port_m.data_port.data_port_id == result:
                        state_v.add_output_port(output_data_port_m)
                        self.canvas.request_update(state_v, matrix=False)
                        self.canvas.wait_for_update()
                        break
            elif method_name == 'remove_input_data_port':
                state_m = model
                state_v = self.canvas.get_view_for_model(state_m)
                if state_v is None:
                    logger.debug("no state_v found for method_name '{}'".format(method_name))
                else:
                    input_port_v = self.canvas.get_view_for_core_element(result)
                    if input_port_v:
                        state_v.remove_input_port(input_port_v)
                        self.canvas.request_update(state_v, matrix=False)
                        self.canvas.wait_for_update()
            elif method_name == 'remove_output_data_port':
                state_m = model
                state_v = self.canvas.get_view_for_model(state_m)
                if state_v is None:
                    logger.debug("no state_v found for method_name '{}'".format(method_name))
                else:
                    output_port_v = self.canvas.get_view_for_core_element(result)
                    if output_port_v:
                        state_v.remove_output_port(output_port_v)
                        self.canvas.request_update(state_v, matrix=False)
                        self.canvas.wait_for_update()
            elif method_name in ['data_type', 'change_data_type']:
                pass
            elif method_name == 'default_value':
                pass

            # ----------------------------------
            #         SCOPED VARIABLES
            # ----------------------------------
            elif method_name == 'add_scoped_variable':
                state_m = model
                state_v = self.canvas.get_view_for_model(state_m)
                for scoped_variable_m in state_m.scoped_variables:
                    if scoped_variable_m.scoped_variable.data_port_id == result:
                        state_v.add_scoped_variable(scoped_variable_m)
                        self.canvas.request_update(state_v, matrix=False)
                        self.canvas.wait_for_update()
                        break
            elif method_name == 'remove_scoped_variable':
                state_m = model
                state_v = self.canvas.get_view_for_model(state_m)
                if state_v is None:
                    logger.debug("no state_v found for method_name '{}'".format(method_name))
                else:
                    scoped_variable_v = self.canvas.get_view_for_core_element(result)
                    if scoped_variable_v:
                        state_v.remove_scoped_variable(scoped_variable_v)
                        self.canvas.request_update(state_v, matrix=False)
                        self.canvas.wait_for_update()

            # ----------------------------------
            #        STATE MISCELLANEOUS
            # ----------------------------------
            elif method_name == 'name':
                # The name of a state was changed
                if not isinstance(model, AbstractStateModel):
                    parent_model = model.parent
                # The name of a port (input, output, scoped var, outcome) was changed
                else:
                    parent_model = model
                state_v = self.canvas.get_view_for_model(parent_model)
                if parent_model is model:
                    state_v.name_view.name = arguments[1]
                    self.canvas.request_update(state_v.name_view, matrix=False)
                else:
                    self.canvas.request_update(state_v, matrix=False)
                self.canvas.wait_for_update()
            elif method_name == 'parent':
                pass
            elif method_name == 'description':
                pass
            elif method_name == 'script_text':
                pass
            # TODO handle the following method calls -> for now those are explicit (in the past implicit) ignored
            # TODO -> correct the complex actions which are used in some test (by test calls or by adapting the model)
            elif method_name in ['input_data_ports', 'output_data_ports', 'outcomes',
                                 'change_root_state_type', 'change_state_type',
                                 'group_states', 'ungroup_state', 'substitute_state']:
                pass
            else:
                known_ignore_list = ['set_input_runtime_value', 'set_use_input_runtime_value',  # from library State
                                     'set_output_runtime_value', 'set_use_output_runtime_value',
                                     'input_data_port_runtime_values', 'use_runtime_value_input_data_ports',
                                     'output_data_port_runtime_values', 'use_runtime_value_output_data_ports',
                                     'semantic_data', 'add_semantic_data', 'remove_semantic_data',
                                     'remove_income']
                if method_name not in known_ignore_list:
                    logger.warning("Method {0} not caught in GraphicalViewer, details: {1}".format(method_name, info))

            if method_name in ['add_state', 'add_transition', 'add_data_flow', 'add_outcome', 'add_input_data_port',
                               'add_output_data_port', 'add_scoped_variable', 'data_flow_change', 'transition_change']:
                try:
                    self._meta_data_changed(None, model, 'append_to_last_change', True)
                except Exception as e:
                    logger.exception('Error while trying to emit meta data signal {0} {1}'.format(e, model))

    @lock_state_machine
    def adapt_complex_action(self, old_state_m, new_state_m):
        old_state_v = self.canvas.get_view_for_model(old_state_m)
        parent_state_v = self.canvas.get_view_for_model(new_state_m.parent)
        old_state_v.remove()

        # If the root state has been changed, we recreate the whole state machine view
        if old_state_m is self.root_state_m:
            # Create and and new root state view from new root state model
            self.root_state_m = new_state_m
            root_state_v = self.add_state_view_for_model(self.root_state_m)
            self.canvas.request_update(root_state_v)

        # Otherwise we only look at the modified state and its children
        else:
            # TODO make the redraw again not recursive for all elements because that is expansive (longer drawing waits)
            # TODO use the handed affected_models list

            # 1st Recreate StateView by removing the old one and adding the new one
            self.add_state_view_with_meta_data_for_model(new_state_m, new_state_m.parent)

            # 2nd Recreate connections to the replaced StateView to ensure correct connectivity
            parent_state = parent_state_v.model.state
            connected_transitions, connected_data_flows = parent_state.get_connections_for_state(new_state_m.state.state_id)
            external_connections = set(connected_transitions['external']['ingoing'] +
                                       connected_transitions['external']['outgoing'] +
                                       connected_data_flows['external']['ingoing'] +
                                       connected_data_flows['external']['outgoing'] +
                                       connected_transitions['external']['self'] +
                                       connected_data_flows['external']['self'])
            for connection_m in external_connections:
                connection_v = self.canvas.get_view_for_core_element(connection_m)
                if connection_v:
                    connection_m = connection_v.model
                    connection_v.remove()
                else:
                    logger.info("The connection element was not existing before and therefore not removed, now."
                                "{0}".format(connection_m))
                if isinstance(connection_m, TransitionModel):
                    self.add_transition_view_for_model(connection_m, parent_state_v.model)
                else:
                    self.add_data_flow_view_for_model(connection_m, parent_state_v.model)

            self.canvas.request_update(parent_state_v)

        self.canvas.wait_for_update()

        try:
            self._meta_data_changed(None, new_state_m, 'append_to_last_change', True)
        except Exception as e:
            logger.exception('Error while trying to emit meta data signal {0} {1}'.format(e, new_state_m))

    @staticmethod
    def _extract_info_data(info):
        if info['method_name'] in ['state_change', 'input_data_port_change', 'output_data_port_change',
                                   'scoped_variable_change', 'outcome_change', 'transition_change', 'data_flow_change']:
            if 'info' in info:
                info = info['info']
            elif 'kwargs' in info:
                info = info['kwargs']
        method_name = info['method_name']
        model = info['model']
        args = info['args']
        instance = info['instance']
        if 'result' in info:
            result = info['result']
        else:
            result = None
        if method_name in ('transition_change', 'data_flow_change'):
            model = args[0]
        elif method_name in ('modify_origin', 'modify_target', 'from_state', 'to_state', 'from_key', 'to_key',
                             'from_outcome', 'to_outcome'):
            if isinstance(model, TransitionModel):
                method_name = 'transition_change'
            elif isinstance(model, DataFlowModel):
                method_name = 'data_flow_change'
        return method_name, model, result, args, instance

    def set_focus_to_state_model(self, state_m, ratio_requested=0.8):
        """ Focus a state view of respective state model
        :param rafcon.gui.model.state state_m: Respective state model of state view to be focused
        :param ratio_requested: Minimum ratio of the screen which is requested, so can be more
        :return:
        """
        state_machine_m = self.model
        state_v = self.canvas.get_view_for_model(state_m)
        if state_v is None:
            logger.warning('There is no view for state model {0}'.format(state_m))
        self.move_item_into_viewport(state_v)
        # check_relative size in view and call it again if the state is still very small
        state_v = self.canvas.get_view_for_model(state_machine_m.root_state)
        state_size = self.view.editor.get_matrix_i2v(state_v).transform_distance(state_v.width, state_v.height)
        viewport_size = self.view.editor.get_allocation().width, self.view.editor.get_allocation().height
        if state_size[0] < ratio_requested*viewport_size[0] and state_size[1] < ratio_requested*viewport_size[1]:
            self.set_focus_to_state_model(state_m, ratio_requested)

    def setup_canvas(self):
        logger.verbose("start setup canvas")
        start_time_view_generation = time.time()
        with self.model.state_machine.modification_lock():
            hash_before = self.model.meta_data_hash()
            # there is no more root state position it will be set always to the root state default relative position
            if not self.root_state_m.get_meta_data_editor()['rel_pos'] == gui_helper_meta_data.ROOT_STATE_DEFAULT_REL_POS:
                self.root_state_m.set_meta_data_editor('rel_pos', gui_helper_meta_data.ROOT_STATE_DEFAULT_REL_POS)
            self.add_state_view_for_model(self.root_state_m, rel_pos=gui_helper_meta_data.STATE_DEFAULT_REL_POS)
            hash_after = self.model.meta_data_hash()
            if hash_before.digest() != hash_after.digest():
                logger.debug("Hash has changed from {0} to {1}".format(hash_before.hexdigest(), hash_after.hexdigest()))
                self._meta_data_changed(None, self.root_state_m, 'append_initial_change', True)
                logger.info("Opening the state machine caused some meta data to be generated, which will be stored "
                            " when the state machine is being saved.")
        logger.verbose("Time spent in setup canvas {0} state machine {1}".format(time.time() - start_time_view_generation,
                                                                                 self.model.state_machine_id))

        # finally set the focus to the root state (needs to be idle add to be executed after gaphas drawing is finished)
        if rafcon.gui.singleton.global_gui_config.get_config_value('GAPHAS_EDITOR_AUTO_FOCUS_OF_ROOT_STATE', True):
            GLib.idle_add(self.set_focus_to_state_model, self.root_state_m)

    @lock_state_machine
    def add_state_view_for_model(self, state_m, parent_v=None, rel_pos=(0, 0), size=(100, 100), hierarchy_level=1):
        """Creates a `StateView` (recursively) and adds it to the canvas

        The method uses the `StateModel` `state_m` to create the according `StateView`. For all content within 
        `state_m`, such as connections, states and ports, the views are also created. All views are added to the canvas.

        :param rafcon.gui.models.state.StateModel state_m: The state to be drawn
        :param rafcon.gui.mygaphas.items.state.StateView parent_v: The parent state view of new state view `state_m`
        :param tuple rel_pos: The default relative position (x, y) if there is no relative position stored
        :param tuple size: The default size (width, height) if there is no size stored
        :param float hierarchy_level: The hierarchy level of the state
        :return: The created `StateView`
        :rtype: StateView
        """
        assert isinstance(state_m, AbstractStateModel)
        state_meta = state_m.get_meta_data_editor()

        # Use default values if no size information is stored
        if not gui_helper_meta_data.contains_geometric_info(state_meta['size']):
            state_meta = state_m.set_meta_data_editor('size', size)

        size = state_meta['size']

        # Use default values if no position information is stored
        if not gui_helper_meta_data.contains_geometric_info(state_meta['rel_pos']):
            state_meta = state_m.set_meta_data_editor('rel_pos', rel_pos)

        rel_pos = state_meta['rel_pos']

        if isinstance(state_m, LibraryStateModel):
            if not state_m.meta_data_was_scaled:
                gui_helper_meta_data.scale_library_ports_meta_data(state_m, gaphas_editor=True)

        state_v = StateView(state_m, size, hierarchy_level)

        # Draw state above data flows and NameView but beneath transitions
        num_data_flows = len(state_m.state.parent.data_flows) if isinstance(state_m.parent, ContainerStateModel) else 0
        index = 1 if not parent_v else num_data_flows + 1
        # if self.model.root_state is state_m:
        #     print("init root_state", state_m, state_v)
        # else:
        #     print("init state", state_m, state_v)
        # print([hash(elem) for elem in state_m.state.outcomes.values()])
        self.canvas.add(state_v, parent_v, index=index)
        state_v.matrix.translate(*rel_pos)

        state_v.add_income(state_m.income)

        for outcome_m in state_m.outcomes:
            state_v.add_outcome(outcome_m)

        for input_port_m in state_m.input_data_ports:
            state_v.add_input_port(input_port_m)

        for output_port_m in state_m.output_data_ports:
            state_v.add_output_port(output_port_m)

        if parent_v is not None:
            # Keep state within parent
            pass

        if isinstance(state_m, LibraryStateModel) and state_m.show_content() and state_m.state_copy_initialized:
            gui_helper_meta_data.scale_library_content(state_m)
            self.add_state_view_for_model(state_m.state_copy, state_v, hierarchy_level=hierarchy_level + 1)

        elif isinstance(state_m, ContainerStateModel):
            num_child_state = 0

            for scoped_variable_m in state_m.scoped_variables:
                state_v.add_scoped_variable(scoped_variable_m)

            for child_state_m in state_m.states.values():
                # generate optional meta data for child state - not used if valid meta data already in child state model
                child_rel_pos, child_size = gui_helper_meta_data.generate_default_state_meta_data(state_m, self.canvas,
                                                                                                  num_child_state)
                num_child_state += 1

                self.add_state_view_for_model(child_state_m, state_v, child_rel_pos, child_size, hierarchy_level + 1)

            for transition_m in state_m.transitions:
                self.add_transition_view_for_model(transition_m, state_m)

            for data_flow_m in state_m.data_flows:
                self.add_data_flow_view_for_model(data_flow_m, state_m)

        return state_v

    @lock_state_machine
    def add_transition_view_for_model(self, transition_m, parent_state_m):
        """Creates a `TransitionView` and adds it to the canvas

        The method creates a`TransitionView` from the given `TransitionModel `transition_m` and adds it to the canvas.

        :param TransitionModel transition_m: The transition for which a view is to be created 
        :param ContainerStateModel parent_state_m: The parental `StateModel` of the transition
        """
        parent_state_v = self.canvas.get_view_for_model(parent_state_m)

        hierarchy_level = parent_state_v.hierarchy_level
        transition_v = TransitionView(transition_m, hierarchy_level)

        # Draw transition above all other state elements
        self.canvas.add(transition_v, parent_state_v, index=None)

        self._connect_transition_to_ports(transition_m, transition_v, parent_state_m, parent_state_v)

        return transition_v

    @lock_state_machine
    def add_data_flow_view_for_model(self, data_flow_m, parent_state_m):
        """Creates a `DataFlowView` and adds it to the canvas

        The method creates a`DataFlowView` from the given `DataFlowModel `data_flow_m` and adds it to the canvas.

        :param DataFlowModel data_flow_m: The data flow for which a view is to be created 
        :param ContainerStateModel parent_state_m: The parental `StateModel` of the data flow
        """
        parent_state_v = self.canvas.get_view_for_model(parent_state_m)

        hierarchy_level = parent_state_v.hierarchy_level
        data_flow_v = DataFlowView(data_flow_m, hierarchy_level)

        # Draw data flow above NameView but beneath all other state elements
        self.canvas.add(data_flow_v, parent_state_v, index=1)
        self._connect_data_flow_to_ports(data_flow_m, data_flow_v, parent_state_m)

    @lock_state_machine
    def add_state_view_with_meta_data_for_model(self, state_m, parent_state_m):
        parent_state_v = self.canvas.get_view_for_model(parent_state_m)

        # generate default meta data for state only if necessary
        state_meta = state_m.get_meta_data_editor()
        if not gui_helper_meta_data.contains_geometric_info(state_meta['size']) or \
                not gui_helper_meta_data.contains_geometric_info(state_meta['rel_pos']):
            child_rel_pos, new_state_size = generate_default_state_meta_data(parent_state_m, self.canvas)
            return self.add_state_view_for_model(state_m, parent_state_v, size=new_state_size, rel_pos=child_rel_pos,
                                                 hierarchy_level=parent_state_m.hierarchy_level + 1)
        else:
            return self.add_state_view_for_model(state_m, parent_state_v,
                                                 hierarchy_level=parent_state_m.hierarchy_level + 1)

    @lock_state_machine
    def _connect_transition_to_ports(self, transition_m, transition_v, parent_state_m, parent_state_v,
                                     use_waypoints=True):

        transition_meta = transition_m.get_meta_data_editor()
        # The state_copy (root_state_of_library) is not shown, therefore transitions to the state_copy are connected
        # to the LibraryState belonging to the state copy
        grandparent_state_v = self.canvas.get_parent(parent_state_v)
        connect_to_grandparent = False
        if parent_state_m.state.is_root_state_of_library:
            connect_to_grandparent = True

        try:
            if use_waypoints:
                waypoint_list = transition_meta['waypoints']

                for waypoint in waypoint_list:
                    transition_v.add_waypoint(waypoint)

            # Get id and references to the from and to state
            from_state_id = transition_m.transition.from_state
            if from_state_id is None:
                if connect_to_grandparent:
                    grandparent_state_v.connect_to_income(transition_v, transition_v.from_handle())
                else:
                    parent_state_v.connect_to_income(transition_v, transition_v.from_handle())
            else:
                from_state_m = parent_state_m.states[from_state_id]
                from_state_v = self.canvas.get_view_for_model(from_state_m)
                from_outcome_id = transition_m.transition.from_outcome
                from_state_v.connect_to_outcome(from_outcome_id, transition_v, transition_v.from_handle())

            to_state_id = transition_m.transition.to_state

            if to_state_id == parent_state_m.state.state_id:  # Transition goes back to parent
                # Set the to coordinates to the outcome coordinates received earlier
                to_outcome_id = transition_m.transition.to_outcome
                if connect_to_grandparent:
                    grandparent_state_v.connect_to_outcome(to_outcome_id, transition_v, transition_v.to_handle())
                else:
                    parent_state_v.connect_to_outcome(to_outcome_id, transition_v, transition_v.to_handle())
            else:
                # Set the to coordinates to the center of the next state
                to_state_m = parent_state_m.states[to_state_id]
                to_state_v = self.canvas.get_view_for_model(to_state_m)
                to_state_v.connect_to_income(transition_v, transition_v.to_handle())

        except AttributeError as e:
            logger.error("Cannot connect transition: {0}".format(e))
            try:
                self.canvas.remove(transition_v)
            except KeyError:
                pass

    @lock_state_machine
    def _connect_data_flow_to_ports(self, data_flow_m, data_flow_v, parent_state_m):
        # Get id and references to the from and to state
        from_state_id = data_flow_m.data_flow.from_state
        from_state_m = parent_state_m if from_state_id == parent_state_m.state.state_id else parent_state_m.states[
            from_state_id]
        # The state_copy (root_state_of_library) is not shown, therefore data flows to the state_copy are connected
        # to the LibraryState belonging to the state copy
        if from_state_m.state.is_root_state_of_library:
            from_state_m = from_state_m.parent
        from_state_v = self.canvas.get_view_for_model(from_state_m)

        to_state_id = data_flow_m.data_flow.to_state
        to_state_m = parent_state_m if to_state_id == parent_state_m.state.state_id else parent_state_m.states[
            to_state_id]
        if to_state_m.state.is_root_state_of_library:  # see comment above
            to_state_m = to_state_m.parent
        to_state_v = self.canvas.get_view_for_model(to_state_m)

        from_key = data_flow_m.data_flow.from_key
        to_key = data_flow_m.data_flow.to_key

        from_port_m = from_state_m.get_data_port_m(from_key)
        to_port_m = to_state_m.get_data_port_m(to_key)

        if from_port_m is None:
            # One case, for which there is no from_port_m is when the the from-port is a ScopedVariable of a
            # LibraryState
            if not isinstance(from_state_m, LibraryStateModel):
                logger.warning('Cannot find model of the from data port {0}, ({1})'.format(from_key,
                                                                                        data_flow_m.data_flow))
            return
        if to_port_m is None:
            # One case, for which there is no to_port_m is when the the to-port is a ScopedVariable of a LibraryState
            if not isinstance(to_state_m, LibraryStateModel):
                logger.warning('Cannot find model of the to data port {0}, ({1})'.format(to_key, data_flow_m.data_flow))
            return

        # For scoped variables, there is no inner and outer connector
        if isinstance(from_port_m, ScopedVariableModel):
            from_state_v.connect_to_scoped_variable_port(from_key, data_flow_v, data_flow_v.from_handle())
        elif from_port_m in from_state_m.input_data_ports:
            from_state_v.connect_to_input_port(from_key, data_flow_v, data_flow_v.from_handle())
        elif from_port_m in from_state_m.output_data_ports:
            from_state_v.connect_to_output_port(from_key, data_flow_v, data_flow_v.from_handle())

        if isinstance(to_port_m, ScopedVariableModel):
            to_state_v.connect_to_scoped_variable_port(to_key, data_flow_v, data_flow_v.to_handle())
        elif to_port_m in to_state_m.output_data_ports:
            to_state_v.connect_to_output_port(to_key, data_flow_v, data_flow_v.to_handle())
        elif to_port_m in to_state_m.input_data_ports:
            to_state_v.connect_to_input_port(to_key, data_flow_v, data_flow_v.to_handle())

    def _reconnect_transition(self, transition_v, transition_m, parent_state_m):
        parent_state_v = self.canvas.get_view_for_model(parent_state_m)

        self.canvas.disconnect_item(transition_v)
        transition_v.remove_connection_from_ports()
        self._connect_transition_to_ports(transition_m, transition_v, parent_state_m, parent_state_v, False)
        self.canvas.update()

    def _reconnect_data_flow(self, data_flow_v, data_flow_m, parent_state_m):
        self.canvas.disconnect_item(data_flow_v)
        data_flow_v.remove_connection_from_ports()
        self._connect_data_flow_to_ports(data_flow_m, data_flow_v, parent_state_m)
        self.canvas.update()

    def react_to_event(self, event):
        """Check whether the given event should be handled

        Checks, whether the editor widget has the focus and whether the selected state machine corresponds to the
        state machine of this editor.

        :param event: GTK event object
        :return: True if the event should be handled, else False
        :rtype: bool
        """
        if not react_to_event(self.view, self.view.editor, event):
            return False
        if not rafcon.gui.singleton.state_machine_manager_model.selected_state_machine_id == \
                self.model.state_machine.state_machine_id:
            return False
        return True

    @lock_state_machine
    def _add_data_port_to_selected_state(self, *event, **kwargs):
        if self.react_to_event(event):
            data_port_type = None if 'data_port_type' not in kwargs else kwargs['data_port_type']
            gui_helper_state_machine.add_data_port_to_selected_states(data_port_type)

    @lock_state_machine
    def _add_scoped_variable_to_selected_state(self, *event, **kwargs):
        if self.react_to_event(event):
            gui_helper_state_machine.add_scoped_variable_to_selected_states()

    @lock_state_machine
    def _add_outcome_to_selected_state(self, *event, **kwargs):
        if self.react_to_event(event):
            gui_helper_state_machine.add_outcome_to_selected_states()

    @lock_state_machine
    def _remove_selected_elements(self, *event, **kwargs):
        if self.react_to_event(event):
            gui_helper_state_machine.delete_selected_elements(self.model)
