"""
.. module:: graphical_editor_gaphas
   :platform: Unix, Windows
   :synopsis: A module that holds the controller to graphically edit a state machine by the gaphas library.

.. moduleauthor:: Franz Steinmetz


"""

from functools import partial

import gtk
from gtk.gdk import ACTION_COPY
from gaphas.aspect import InMotion, ItemFinder

from rafcon.statemachine.enums import StateType

from rafcon.mvc.clipboard import global_clipboard
from rafcon.mvc import state_machine_helper

from rafcon.mvc.models.signals import MetaSignalMsg
from rafcon.mvc.models.state_machine import StateMachineModel
from rafcon.mvc.models import ContainerStateModel, AbstractStateModel, TransitionModel, DataFlowModel
from rafcon.mvc.models.scoped_variable import ScopedVariableModel

from rafcon.mvc.controllers.utils.extended_controller import ExtendedController
from rafcon.mvc.views.graphical_editor_gaphas import GraphicalEditorView

from rafcon.mvc.mygaphas.items.state import StateView, NameView
from rafcon.mvc.mygaphas.items.connection import DataFlowView, TransitionView
from rafcon.mvc.mygaphas.items.ports import OutcomeView, DataPortView, ScopedVariablePortView
from rafcon.mvc.mygaphas.canvas import MyCanvas
from rafcon.mvc.mygaphas import guide

from rafcon.utils import log
logger = log.get_logger(__name__)


class GraphicalEditorController(ExtendedController):
    """Controller handling the graphical editor

    :param rafcon.mvc.models.state_machine.StateMachineModel model: The state machine model, holding the root
        state and the current selection
    :param rafcon.mvc.views.graphical_editor.GraphicalEditorView view: The GTK view having an OpenGL rendering
        element
    """

    _change_state_type = False

    def __init__(self, model, view):
        """Constructor"""
        ExtendedController.__init__(self, model, view)
        assert type(view) == GraphicalEditorView
        assert isinstance(self.model, StateMachineModel)
        # assert isinstance(self.view, GraphicalEditorView)
        # assert isinstance(self.view.editor, GraphicalEditor)
        self.root_state_m = model.root_state

        self.canvas = MyCanvas()
        self.zoom = 3.

        view.setup_canvas(self.canvas, self.zoom)

        view.editor.drag_dest_set(gtk.DEST_DEFAULT_ALL, [('STRING', 0, 0)], ACTION_COPY)

    def register_view(self, view):
        """Called when the View was registered"""
        assert self.view == view
        self.setup_canvas()

        self.view.editor.connect('selection-changed', self._update_selection_from_gaphas)
        self.view.connect('remove_state_from_state_machine', self._remove_state_view)
        self.view.connect('meta_data_changed', self._meta_data_changed)
        self.view.editor.connect("drag-data-received", self.on_drag_data_received)
        self.view.editor.connect("drag-motion", self.on_drag_motion)

    def register_adapters(self):
        """Adapters should be registered in this method call"""
        pass

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.mvc.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
            between shortcuts and actions.
        """
        shortcut_manager.add_callback_for_action("add", partial(self._add_new_state, state_type=StateType.EXECUTION))
        shortcut_manager.add_callback_for_action("add2", partial(self._add_new_state, state_type=StateType.HIERARCHY))

        shortcut_manager.add_callback_for_action("copy", self._copy_selection)
        shortcut_manager.add_callback_for_action("paste", self._paste_clipboard)
        shortcut_manager.add_callback_for_action("cut", self._cut_selection)

        shortcut_manager.add_callback_for_action('show_data_flows', self.update_view)
        shortcut_manager.add_callback_for_action('show_data_values', self.update_view)
        shortcut_manager.add_callback_for_action('data_flow_mode', self.data_flow_mode)
        shortcut_manager.add_callback_for_action('show_aborted_preempted', self.update_view)

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
        item = self.canvas.get_view_for_model(self.model.selection.get_selected_state().states[data.get_text()])
        pos_start = item.model.meta['gui']['editor_gaphas']['rel_pos']
        motion = InMotion(item, self.view.editor)
        motion.start_move(self.view.editor.get_matrix_i2v(item).transform_point(pos_start[0], pos_start[1]))
        motion.move((x, y))
        motion.stop_move()

    def on_drag_motion(self, widget, context, x, y, time):
        """Changes the selection on mouse over during drag motion

        :param widget:
        :param context:
        :param x: Integer: x-position of mouse
        :param y: Integer: y-position of mouse
        :param time:
        """
        hovered = ItemFinder(self.view.editor).get_item_at_point((x, y))
        if isinstance(hovered, NameView):
            hovered = hovered.parent
        if hovered is None:
            self.view.editor.unselect_all()
        elif isinstance(hovered.model, ContainerStateModel):
            if len(self.view.editor.selected_items) == 1 and hovered in self.view.editor.selected_items:
                return
            if len(self.view.editor.selected_items) > 0:
                self.view.editor.unselect_all()
            self.view.editor.focused_item = hovered

    def update_view(self, *args):
        self.canvas.update_root_items()

    def data_flow_mode(self, *args):
        self.handle_selected_states(self.model.selection.get_states())
        self.canvas.update_root_items()

    def _add_new_state(self, *args, **kwargs):
        """Triggered when shortcut keys for adding a new state are pressed, or Menu Bar "Edit, Add State" is clicked.

        Adds a new state only if the graphical editor is in focus.
        """
        if self.view and self.view.editor.is_focus():
            state_type = StateType.EXECUTION if 'state_type' not in kwargs else kwargs['state_type']
            return state_machine_helper.add_new_state(self.model, state_type)

    def _copy_selection(self, *args):
        """Copies the current selection to the clipboard.
        """
        if self.view and self.view.editor.is_focus():
            logger.debug("copy selection")
            global_clipboard.copy(self.model.selection)

    def _cut_selection(self, *args):
        """Cuts the current selection and copys it to the clipboard.
        """
        if self.view and self.view.editor.is_focus():
            logger.debug("cut selection")
            global_clipboard.cut(self.model.selection)

    def _paste_clipboard(self, *args):
        """Paste the current clipboard into the current selection if the current selection is a container state.
        """
        if self.view and self.view.editor.is_focus():
            logger.debug("Paste")

            # Always update canvas and handle all events in the gtk queue before performing any changes
            self.canvas.update_now()
            while gtk.events_pending():
                gtk.main_iteration(False)

            current_selection = self.model.selection
            if len(current_selection) != 1 or len(current_selection.get_states()) < 1:
                logger.error("Please select a single state for pasting the clipboard")
                return
            if not isinstance(current_selection.get_states()[0], ContainerStateModel):
                # the default behaviour of the copy paste is that the state is copied into the parent state
                parent_of_old_state = current_selection.get_states()[0].parent
                current_selection.clear()
                current_selection.add(parent_of_old_state)

            # Note: in multi-selection case, a loop over all selected items is necessary instead of the 0 index
            target_state_m = current_selection.get_states()[0]
            state_copy_m, state_orig_m = global_clipboard.paste(target_state_m)

            if state_copy_m is None:  # An error occurred while pasting
                return

            # Adjust size of new state
            old_size = state_orig_m.meta['gui']['editor_gaphas']['size']
            target_size = target_state_m.meta['gui']['editor_gaphas']['size']

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

            new_state_v = self.canvas.get_view_for_model(state_copy_m)
            new_state_v.width = new_size[0]
            new_state_v.height = new_size[1]
            state_copy_m.meta['gui']['editor_gaphas']['size'] = (new_state_v.width, new_state_v.height)

            new_state_v.resize_all_children(old_size, True)
            self._meta_data_changed(new_state_v, state_copy_m, 'all', True)

    def _update_selection_from_gaphas(self, view, selected_items):
        selected_items = self.view.editor.selected_items
        selected_models = []
        for item in selected_items:
            if isinstance(item, (StateView, TransitionView, DataFlowView, OutcomeView, DataPortView,
                                 ScopedVariablePortView)):
                selected_models.append(item.model)
            elif isinstance(item, NameView):
                selected_models.append(item.parent.model)
            else:
                logger.debug("Cannot select item {}".format(item))
        new_selected_models = any([model not in self.model.selection for model in selected_models])
        if new_selected_models or len(self.model.selection) != len(selected_models):
            self.model.selection.set(selected_models)

    def _update_selection_from_external(self):
        selected_items = [self.canvas.get_view_for_model(model) for model in self.model.selection]
        select_items = filter(lambda item: item not in self.view.editor.selected_items, selected_items)
        deselect_items = filter(lambda item: item not in selected_items, self.view.editor.selected_items)
        for item in deselect_items:
            self.view.editor.selected_items.discard(item)
            self.view.editor.queue_draw_item(item)
        for item in select_items:
            self.view.editor.selected_items.add(item)
            self.view.editor.queue_draw_item(item)
        if select_items or deselect_items:
            self.view.editor.emit('selection-changed', self.view.editor.selected_items)
        # TODO: Jump to the selected state in the view and adjust the zoom
        # state_v = None

        # if global_runtime_config.get_config_value("DATA_FLOW_MODE"):
        #     for data_flow in self.get_connected_data_flows(state_v):
        #         data_flow.show()
        #     self.set_non_active_states_transparent(True, state_v)
        # else:
        #     for data_flow in self.get_connected_data_flows(state_v):
        #         data_flow.hide()
        #     self.set_non_active_states_transparent(False, state_v)
        #
        # self.view.editor.focused_item = state_v

    def _meta_data_changed(self, view, model, name, affects_children):
        msg = MetaSignalMsg('graphical_editor_gaphas', name, affects_children)
        model.meta_signal.emit(msg)

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
        state_m = notification.model
        state_v = self.canvas.get_view_for_model(state_m)

        # Always update canvas and handle all events in the gtk queue before performing any changes
        self.canvas.update_now()
        while gtk.events_pending():
            gtk.main_iteration(False)
        state_v.apply_meta_data(recursive=meta_signal_message.affects_children)
        self.canvas.request_update(state_v, matrix=True)

    def manual_notify_after(self, state_m):
        state_v = self.canvas.get_view_for_model(state_m)
        if state_v:
            state_v.apply_meta_data(recursive=True)
            self.canvas.request_update(state_v, matrix=True)
        else:
            logger.info("Meta data operation on state model without view: {}".format(state_m))

    @ExtendedController.observe("state_machine", before=True)
    def state_machine_change_before(self, model, prop_name, info):
        if 'method_name' in info and info['method_name'] == 'root_state_change':
            method_name, model, result, arguments, instance = self._extract_info_data(info['kwargs'])

            if method_name in ['change_state_type', 'change_root_state_type']:
                self._change_state_type = True

    @ExtendedController.observe("state_machine", after=True)
    def state_machine_change_after(self, model, prop_name, info):
        """Called on any change within th state machine

        This method is called, when any state, transition, data flow, etc. within the state machine changes. This
        then typically requires a redraw of the graphical editor, to display these changes immediately.

        :param rafcon.mvc.models.state_machine.StateMachineModel model: The state machine model
        :param str prop_name: The property that was changed
        :param dict info: Information about the change
        """

        if 'method_name' in info and info['method_name'] == 'root_state_change':
            method_name, model, result, arguments, instance = self._extract_info_data(info['kwargs'])

            if self._change_state_type and method_name not in ['change_state_type', 'change_root_state_type']:
                return

            # Always update canvas and handle all events in the gtk queue before performing any changes
            self.canvas.update_now()
            while gtk.events_pending():
                gtk.main_iteration(False)

            # The method causing the change raised an exception, thus nothing was changed
            if (isinstance(result, str) and "CRASH" in result) or isinstance(result, Exception):
                return

            if method_name == 'state_execution_status':
                state_v = self.canvas.get_view_for_model(model)
                if state_v:  # Children of LibraryStates are not modeled, yet
                    self.canvas.request_update(state_v, matrix=False)
            elif method_name == 'add_state':
                if self._change_state_type:
                    return
                new_state = arguments[1]
                new_state_m = model.states[new_state.state_id]
                self.add_state_view_to_parent(new_state_m, model)
            elif method_name == 'remove_state':
                if self._change_state_type:
                    return
                parent_state = arguments[0]
                state_id = arguments[1]
                parent_v = self.canvas.get_view_for_core_element(parent_state)
                state_v = self.canvas.get_view_for_id(StateView, state_id, parent_v)
                if state_v:
                    parent_v = self.canvas.get_parent(state_v)
                    state_v.remove()
                    if parent_v:
                        self.canvas.request_update(parent_v)

            # ----------------------------------
            #           TRANSITIONS
            # ----------------------------------
            elif method_name == 'add_transition':
                if self._change_state_type:
                    return
                transitions_models = model.transitions
                transition_id = result
                for transition_m in transitions_models:
                    if transition_m.transition.transition_id == transition_id:
                        self.add_transition_view_for_model(transition_m, model)
            elif method_name == 'remove_transition':
                if self._change_state_type:
                    return
                self.remove_transition_view_from_parent_view(model)
            elif method_name == 'transition_change':
                transition_m = model
                transition_v = self.canvas.get_view_for_model(transition_m)
                self.connect_transition_handle_to_state(transition_v, transition_m, transition_m.parent)

            # ----------------------------------
            #           DATA FLOW
            # ----------------------------------
            elif method_name == 'add_data_flow':
                data_flow_models = model.data_flows
                data_flow_id = result
                for data_flow_m in data_flow_models:
                    if data_flow_m.data_flow.data_flow_id == data_flow_id:
                        self.add_data_flow_view_for_model(data_flow_m, model)
            elif method_name == 'remove_data_flow':
                self.remove_data_flow_view_from_parent_view(model)
            elif method_name == 'data_flow_change':
                data_flow_m = model
                data_flow_v = self.canvas.get_view_for_model(data_flow_m)
                self.connect_data_flow_handle_to_state(data_flow_v, data_flow_m, data_flow_m.parent)

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
            elif method_name == 'remove_outcome':
                state_m = model
                state_v = self.canvas.get_view_for_model(state_m)
                if state_v is None:
                    logger.debug("no state_v found for method_name '{}'".format(method_name))
                else:
                    for outcome_v in state_v.outcomes:
                        if outcome_v.outcome_id == arguments[1]:
                            state_v.remove_outcome(outcome_v)
                            self.canvas.request_update(state_v, matrix=False)

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
            elif method_name == 'add_output_data_port':
                state_m = model
                state_v = self.canvas.get_view_for_model(state_m)
                for output_data_port_m in state_m.output_data_ports:
                    if output_data_port_m.data_port.data_port_id == result:
                        state_v.add_output_port(output_data_port_m)
                        self.canvas.request_update(state_v, matrix=False)
            elif method_name == 'remove_input_data_port':
                state_m = model
                state_v = self.canvas.get_view_for_model(state_m)
                if state_v is None:
                    logger.debug("no state_v found for method_name '{}'".format(method_name))
                else:
                    for input_port_v in state_v.inputs:
                        if input_port_v.port_id == arguments[1]:
                            state_v.remove_input_port(input_port_v)
                            self.canvas.request_update(state_v, matrix=False)
            elif method_name == 'remove_output_data_port':
                state_m = model
                state_v = self.canvas.get_view_for_model(state_m)
                if state_v is None:
                    logger.debug("no state_v found for method_name '{}'".format(method_name))
                else:
                    for output_port_v in state_v.outputs:
                        if output_port_v.port_id == arguments[1]:
                            state_v.remove_output_port(output_port_v)
                            self.canvas.request_update(state_v, matrix=False)
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
            elif method_name == 'remove_scoped_variable':
                state_m = model
                state_v = self.canvas.get_view_for_model(state_m)
                if state_v is None:
                    logger.debug("no state_v found for method_name '{}'".format(method_name))
                else:
                    for scoped_variable_v in state_v.scoped_variables:
                        if scoped_variable_v.port_id == arguments[1]:
                            state_v.remove_scoped_variable(scoped_variable_v)
                            self.canvas.request_update(state_v, matrix=False)

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
            elif method_name in ['change_state_type', 'change_root_state_type']:
                self._change_state_type = False
                if method_name == 'change_state_type':
                    old_state = arguments[1]
                    state_v = self.canvas.get_view_for_core_element(old_state)
                    parent_state_m = model
                    new_state = result
                    new_state_m = parent_state_m.states[new_state.state_id]
                else:
                    state_machine_m = model
                    new_state_m = state_machine_m.root_state
                    state_v = self.canvas.get_root_items()[0]
                state_v.model = new_state_m
                if isinstance(new_state_m, ContainerStateModel):
                    # Check for new states, which do not have a StateView (typically DeciderState)
                    for child_state_m in new_state_m.states.itervalues():
                        if not self.canvas.get_view_for_model(child_state_m):
                            self.add_state_view_to_parent(child_state_m, new_state_m)
                    # Check for new transitions, which do not have a TransitionView (typically related to DeciderState)
                    for transition_m in new_state_m.transitions:
                        if not self.canvas.get_view_for_model(transition_m):
                            self.add_transition_view_for_model(transition_m, new_state_m)
                    # Check for old StateViews (typically DeciderState) and TransitionViews, no longer existing
                    for child_v in self.canvas.get_children(state_v):
                        if isinstance(child_v, StateView):
                            if child_v.model.state.state_id not in new_state_m.states:
                                child_v.remove()
                        elif isinstance(child_v, TransitionView):
                            if child_v.model not in new_state_m.transitions:
                                self.canvas.remove(child_v)
                else:
                    # Remove all child states, as StateModels cannot have children
                    children = self.canvas.get_children(state_v)[:]
                    for child_v in children:
                        if isinstance(child_v, StateView):
                            child_v.remove()
                        elif not isinstance(child_v, NameView):  # Don't remove the name view
                            self.canvas.remove(child_v)
                parent_v = self.canvas.get_parent(state_v)
                if parent_v:
                    self.canvas.request_update(parent_v)
                else:
                    self.canvas.request_update(state_v)
            elif method_name == 'parent':
                pass
            elif method_name == 'description':
                pass
            else:
                logger.debug("Method '%s' not caught in GraphicalViewer" % method_name)

            if method_name in ['add_state', 'add_transition', 'add_data_flow', 'add_outcome', 'add_input_data_port',
                               'add_output_data_port', 'add_scoped_variable', 'data_flow_change', 'transition_change',
                               'change_state_type', 'change_root_state_type']:
                try:
                    self._meta_data_changed(None, model, 'append_to_last_change', True)
                except Exception as e:
                    logger.error('Error while trying to emit meta data signal {}'.format(e))

    @ExtendedController.observe("root_state", assign=True)
    def root_state_change(self, state_machine_m, prop_name, info):
        """Called when the root state was exchanged

        Exchanges the local reference to the root state and redraws.

        :param rafcon.mvc.models.state_machine.StateMachineModel state_machine_m: The state machine model
        :param str prop_name: The root state
        :param dict info: Information about the change
        """
        if self.root_state_m is not state_machine_m.root_state:
            if self._change_state_type:
                return
            logger.debug("The root state was exchanged")

            # Always update canvas and handle all events in the gtk queue before performing any changes
            self.canvas.update_now()
            while gtk.events_pending():
                gtk.main_iteration(False)

            # Remove old root state view
            root_state_v = self.canvas.get_root_items()[0]
            try:
                root_state_v.remove()
            except KeyError:
                pass

            # Create and and new root state view from new root state model
            self.root_state_m = state_machine_m.root_state
            self.setup_state(self.root_state_m)

    @ExtendedController.observe("selection", after=True)
    def selection_change(self, model, prop_name, info):
        """Called when the selection was changed externally

        Updates the local selection and redraws.

        :param rafcon.mvc.models.state_machine.StateMachineModel model: The state machine model
        :param str prop_name: The selection
        :param dict info: Information about the change
        """
        self._update_selection_from_external()

    def set_non_active_states_transparent(self, transparent, state_v):
        if transparent:
            for root_item in self.canvas.get_root_items():
                if isinstance(root_item, StateView):
                    if root_item is not state_v:
                        root_item.background()
                for child in self.canvas.get_all_children(root_item):
                    if isinstance(child, StateView):
                        if child is not state_v:
                            child.background()
        else:
            for root_item in self.canvas.get_root_items():
                if isinstance(root_item, StateView):
                    root_item.foreground()
                for child in self.canvas.get_all_children(root_item):
                    if isinstance(child, StateView):
                        child.foreground()

    def get_connected_data_flows(self, state_v):
        parent_v = self.canvas.get_parent(state_v)
        connected_data_flows = []
        for child in self.canvas.get_children(parent_v):
            if isinstance(child, DataFlowView):
                if child.from_port in state_v.get_data_ports() or child.to_port in state_v.get_data_ports():
                    connected_data_flows.append(child)
        return connected_data_flows

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

    def connect_transition_handle_to_state(self, transition_v, transition_m, parent_state_m):
        parent_state_v = self.canvas.get_view_for_model(parent_state_m)

        self.canvas.disconnect_item(transition_v)
        transition_v.remove_connection_from_ports()
        self.add_transition(transition_m, transition_v, parent_state_m, parent_state_v, False)
        self.canvas.update()

    def connect_data_flow_handle_to_state(self, data_flow_v, data_flow_m, parent_state_m):
        self.canvas.disconnect_item(data_flow_v)
        data_flow_v.remove_connection_from_ports()
        self.add_data_flow(data_flow_m, data_flow_v, parent_state_m)
        self.canvas.update()

    @staticmethod
    def get_state_model(container_m, state_id):
        if state_id is None:
            return container_m
        return container_m.states[state_id]

    def add_transition_view_for_model(self, transition_m, parent_state_m):
        parent_state_v = self.canvas.get_view_for_model(parent_state_m)

        new_transition_hierarchy_level = parent_state_v.hierarchy_level
        new_transition_v = TransitionView(transition_m, new_transition_hierarchy_level)

        self.canvas.add(new_transition_v, parent_state_v, index=0)

        self.add_transition(transition_m, new_transition_v, parent_state_m, parent_state_v)

    def add_data_flow_view_for_model(self, data_flow_m, parent_state_m):
        parent_state_v = self.canvas.get_view_for_model(parent_state_m)

        new_data_flow_hierarchy_level = parent_state_v.hierarchy_level
        new_data_flow_v = DataFlowView(data_flow_m, new_data_flow_hierarchy_level)

        self.canvas.add(new_data_flow_v, parent_state_v, index=0)
        self.add_data_flow(data_flow_m, new_data_flow_v, parent_state_m)

    def _remove_connection_view(self, parent_state_m, transitions=True):
        parent_state_v = self.canvas.get_view_for_model(parent_state_m)

        if transitions:
            available_connections = parent_state_m.transitions
        else:
            available_connections = parent_state_m.data_flows

        children = self.canvas.get_children(parent_state_v)
        for child in list(children):
            if transitions and isinstance(child, TransitionView) and child.model not in available_connections:
                child.prepare_destruction()
                self.canvas.remove(child)
            elif not transitions and isinstance(child, DataFlowView) and child.model not in available_connections:
                child.prepare_destruction()
                self.canvas.remove(child)

    def remove_data_flow_view_from_parent_view(self, parent_state_m):
        self._remove_connection_view(parent_state_m, False)

    def remove_transition_view_from_parent_view(self, parent_state_m):
        self._remove_connection_view(parent_state_m)

    def add_state_view_to_parent(self, state_m, parent_state_m):
        parent_state_v = self.canvas.get_view_for_model(parent_state_m)

        new_state_side_size = min(parent_state_v.width * 0.2, parent_state_v.height * 0.2)
        new_state_hierarchy_level = parent_state_v.hierarchy_level + 1
        new_state_size = (new_state_side_size, new_state_side_size)

        child_width = new_state_side_size
        child_height = new_state_side_size
        child_size = (child_width, child_height)
        child_spacing = max(child_size) * 1.2

        max_cols = parent_state_v.width // child_spacing
        (row, col) = divmod(len(parent_state_m.states) - 1, max_cols)
        child_rel_pos_x = col * child_spacing + child_spacing - child_width
        child_rel_pos_y = child_spacing * (1.5 * row + 1)
        child_rel_pos = (child_rel_pos_x, child_rel_pos_y)

        return self.setup_state(state_m, parent_state_v, size=new_state_size, rel_pos=child_rel_pos,
                                hierarchy_level=new_state_hierarchy_level)

    def _remove_state_view(self, view):
        return state_machine_helper.delete_selected_elements(self.model)

    def setup_canvas(self):

        self.setup_state(self.root_state_m, rel_pos=(10, 10))
        self._meta_data_changed(None, self.root_state_m, 'append_to_last_change', True)

    def setup_state(self, state_m, parent=None, rel_pos=(0, 0), size=(100, 100), hierarchy_level=1):

        """Draws a (container) state with all its content

        Mainly contains the logic for drawing (e. g. reading and calculating values). The actual drawing process is
        done in the view, which is called from this method with the appropriate arguments.

        :param rafcon.mvc.models.state.StateModel state_m: The state to be drawn
        :param tuple rel_pos: The default relative position (x, y) if there is no relative position stored
        :param tuple size: The default size (width, height) if there is no size stored
        :param float depth: The hierarchy level of the state
        """
        assert isinstance(state_m, AbstractStateModel)
        state_meta_gaphas = state_m.meta['gui']['editor_gaphas']
        state_meta_opengl = state_m.meta['gui']['editor_opengl']

        # Use default values if no size information is stored
        if isinstance(state_meta_opengl['size'], tuple) and not isinstance(state_meta_gaphas['size'], tuple):
            state_meta_gaphas['size'] = state_meta_opengl['size']
            self.model.state_machine.marked_dirty = True
        if not isinstance(state_meta_gaphas['size'], tuple):
            state_meta_gaphas['size'] = size

        size = state_meta_gaphas['size']

        if isinstance(state_meta_opengl['rel_pos'], tuple) and not isinstance(state_meta_gaphas['rel_pos'], tuple):
            rel_pos = state_meta_opengl['rel_pos']
            state_meta_gaphas['rel_pos'] = (rel_pos[0], -rel_pos[1])
            self.model.state_machine.marked_dirty = True
        elif not isinstance(state_meta_gaphas['rel_pos'], tuple):
            state_meta_gaphas['rel_pos'] = rel_pos

        rel_pos = state_meta_gaphas['rel_pos']

        state_v = StateView(state_m, size, hierarchy_level)
        self.canvas.add(state_v, parent)
        state_v.matrix.translate(*rel_pos)

        for outcome_m in state_m.outcomes:
            state_v.add_outcome(outcome_m)
            # state_v.add_double_port_outcome(outcome_m)

        for input_port_m in state_m.input_data_ports:
            state_v.add_input_port(input_port_m)

        for output_port_m in state_m.output_data_ports:
            state_v.add_output_port(output_port_m)

        if parent is not None:
            # Keep state within parent
            pass

        if isinstance(state_m, ContainerStateModel):
            num_child_state = 0
            state_width = size[0]
            state_height = size[1]

            for scoped_variable_m in state_m.scoped_variables:
                state_v.add_scoped_variable(scoped_variable_m)

            for child_state in state_m.states.itervalues():
                # Calculate default positions for the child states
                # Make the inset from the top left corner

                child_width = state_width / 5.
                child_height = state_height / 5.
                child_size = (child_width, child_height)
                child_spacing = max(child_size) * 1.2

                max_cols = state_width // child_spacing
                (row, col) = divmod(num_child_state, max_cols)
                child_rel_pos_x = col * child_spacing + child_spacing - child_width
                child_rel_pos_y = child_spacing * (1.5 * row + 1)
                child_rel_pos = (child_rel_pos_x, child_rel_pos_y)
                num_child_state += 1

                self.setup_state(child_state, state_v, child_rel_pos, child_size, hierarchy_level + 1)

            self.add_transitions(state_m, hierarchy_level)

            self.add_data_flows(state_m, hierarchy_level)

        return state_v

    def add_transitions(self, parent_state_m, hierarchy_level):
        """Draws the transitions belonging to a state

        The method takes all transitions from the given state and calculates their start and end point positions.
        Those are passed together with the waypoints to the view of the graphical editor.

        :param rafcon.mvc.models.container_state.ContainerStateModel parent_state_m: The model of the container
            state, of which the transitions shall be drawn
        """
        parent_state_v = self.canvas.get_view_for_model(parent_state_m)
        assert isinstance(parent_state_v, StateView)
        for transition_m in parent_state_m.transitions:
            transition_v = TransitionView(transition_m, hierarchy_level)
            self.canvas.add(transition_v, parent_state_v, index=0)

            self.add_transition(transition_m, transition_v, parent_state_m, parent_state_v)

    def add_transition(self, transition_m, transition_v, parent_state_m, parent_state_v, use_waypoints=True):

        transition_meta_gaphas = transition_m.meta['gui']['editor_gaphas']
        transition_meta_opengl = transition_m.meta['gui']['editor_opengl']

        try:
            if use_waypoints:
                if (isinstance(transition_meta_opengl['waypoints'], list) and
                        isinstance(transition_meta_gaphas['waypoints'], dict)):
                    old_waypoint_list = transition_meta_opengl['waypoints']
                    new_waypoint_list = []
                    for wp in old_waypoint_list:
                        wp = (wp[0], -wp[1])
                        new_waypoint_list.append(wp)
                    transition_meta_gaphas['waypoints'] = new_waypoint_list
                    self.model.state_machine.marked_dirty = True

                waypoint_list = transition_meta_gaphas['waypoints']

                for waypoint in waypoint_list:
                    transition_v.add_waypoint(waypoint)

            # Get id and references to the from and to state
            from_state_id = transition_m.transition.from_state
            if from_state_id is None:
                parent_state_v.connect_to_income(transition_v, transition_v.from_handle())
            else:
                from_state_m = parent_state_m.states[from_state_id]
                from_state_v = self.canvas.get_view_for_model(from_state_m)
                from_outcome_id = transition_m.transition.from_outcome
                from_state_v.connect_to_outcome(from_outcome_id, transition_v, transition_v.from_handle())
                # from_state_v.connect_to_double_port_outcome(from_outcome_id, transition_v,
                # transition_v.from_handle(), False)

            to_state_id = transition_m.transition.to_state

            if to_state_id == parent_state_m.state.state_id:  # Transition goes back to parent
                # Set the to coordinates to the outcome coordinates received earlier
                to_outcome_id = transition_m.transition.to_outcome
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

    def add_data_flows(self, parent_state_m, hierarchy_level):
        """Draw all data flows contained in the given container state

        The method takes all data flows from the given state and calculates their start and end point positions.
        Those are passed together with the waypoints to the view of the graphical editor.

        :param rafcon.mvc.models.container_state.ContainerStateModel parent_state_m: The model of the container
            state, of which the data flows shall be drawn
        """
        parent_state_v = self.canvas.get_view_for_model(parent_state_m)
        assert isinstance(parent_state_v, StateView)
        for data_flow_m in parent_state_m.data_flows:
            data_flow_v = DataFlowView(data_flow_m, hierarchy_level)

            self.canvas.add(data_flow_v, parent_state_v, index=0)
            self.add_data_flow(data_flow_m, data_flow_v, parent_state_m)

    def add_data_flow(self, data_flow_m, data_flow_v, parent_state_m):
        # Get id and references to the from and to state
        from_state_id = data_flow_m.data_flow.from_state
        from_state_m = parent_state_m if from_state_id == parent_state_m.state.state_id else parent_state_m.states[
            from_state_id]
        from_state_v = self.canvas.get_view_for_model(from_state_m)

        to_state_id = data_flow_m.data_flow.to_state
        to_state_m = parent_state_m if to_state_id == parent_state_m.state.state_id else parent_state_m.states[
            to_state_id]
        to_state_v = self.canvas.get_view_for_model(to_state_m)

        from_key = data_flow_m.data_flow.from_key
        to_key = data_flow_m.data_flow.to_key

        from_port_m = from_state_m.get_data_port_m(from_key)
        to_port_m = to_state_m.get_data_port_m(to_key)

        if from_port_m is None:
            logger.warn('Cannot find model of the from data port {0}, ({1})'.format(from_key,
                                                                                    data_flow_m.data_flow))
            return
        if to_port_m is None:
            logger.warn('Cannot find model of the to data port {0}, ({1})'.format(to_key, data_flow_m.data_flow))
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
