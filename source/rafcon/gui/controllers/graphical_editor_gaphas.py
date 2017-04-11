# Copyright (C) 2015-2017 DLR
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

import gtk
from functools import partial
from gaphas.aspect import InMotion, ItemFinder
from gtk.gdk import ACTION_COPY

import rafcon.core.id_generator as idgen
from rafcon.core.decorators import lock_state_machine
from rafcon.core.states.state import StateType
from rafcon.gui.clipboard import global_clipboard
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
import rafcon.gui.helpers.state_machine as gui_helper_state_machine
from rafcon.gui.helpers.label import react_to_event
from rafcon.gui.models import ContainerStateModel, AbstractStateModel, TransitionModel, DataFlowModel
from rafcon.gui.models.scoped_variable import ScopedVariableModel
from rafcon.gui.models.signals import MetaSignalMsg
from rafcon.gui.models.state_machine import StateMachineModel
from rafcon.gui.mygaphas.canvas import MyCanvas
# noinspection PyUnresolvedReferences
from rafcon.gui.mygaphas import guide
from rafcon.gui.mygaphas.items.connection import DataFlowView, TransitionView
from rafcon.gui.mygaphas.items.ports import OutcomeView, DataPortView, ScopedVariablePortView
from rafcon.gui.mygaphas.items.state import StateView, NameView
from rafcon.gui.singleton import gui_config_model, runtime_config_model
from rafcon.gui.views.graphical_editor_gaphas import GraphicalEditorView
import rafcon.gui.helpers.state as gui_helper_state
from rafcon.utils import log
logger = log.get_logger(__name__)


class GraphicalEditorController(ExtendedController):
    """Controller handling the graphical editor

    :param rafcon.gui.models.state_machine.StateMachineModel model: The state machine model, holding the root
        state and the current selection
    :param rafcon.gui.views.graphical_editor.GraphicalEditorView view: The GTK view having an OpenGL rendering
        element
    """

    _complex_action = False

    def __init__(self, model, view):
        """Constructor"""
        ExtendedController.__init__(self, model, view)
        assert type(view) == GraphicalEditorView
        assert isinstance(self.model, StateMachineModel)
        self.observe_model(gui_config_model)
        self.observe_model(runtime_config_model)
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
        item = self.canvas.get_view_for_model(self.model.selection.get_selected_state().states[data.get_text()])
        pos_start = item.model.get_meta_data_editor()['rel_pos']
        motion = InMotion(item, self.view.editor)
        motion.start_move(self.view.editor.get_matrix_i2v(item).transform_point(pos_start[0], pos_start[1]))
        motion.move((x, y))
        motion.stop_move()

    @lock_state_machine
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

    @lock_state_machine
    def data_flow_mode(self, *args):
        self.handle_selected_states(self.model.selection.get_states())
        self.canvas.update_root_items()

    @lock_state_machine
    def _add_new_state(self, *event, **kwargs):
        """Triggered when shortcut keys for adding a new state are pressed, or Menu Bar "Edit, Add State" is clicked.

        Adds a new state only if the graphical editor is in focus.
        """
        if react_to_event(self.view, self.view.editor, event):
            state_type = StateType.EXECUTION if 'state_type' not in kwargs else kwargs['state_type']
            return gui_helper_state_machine.add_new_state(self.model, state_type)

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
        if self._complex_action:
            return
        model = notification.model
        view = self.canvas.get_view_for_model(model)
        if isinstance(view, StateView):
            view.apply_meta_data(recursive=meta_signal_message.affects_children)
        else:
            view.apply_meta_data()

        self.canvas.request_update(view, matrix=True)
        self.canvas.perform_update()

    def manual_notify_after(self, state_m):
        state_v = self.canvas.get_view_for_model(state_m)
        if state_v:
            state_v.apply_meta_data(recursive=True)
            self.canvas.request_update(state_v, matrix=True)
        else:
            logger.info("Meta data operation on state model without view: {}".format(state_m))

    @ExtendedController.observe("state_action_signal", signal=True)
    def state_action_signal(self, model, prop_name, info):
        # print "GSME state_action_signal: ", info['arg'] if 'arg' in info else "XXX" + str(info)
        if 'arg' in info and info['arg'].action in ['change_root_state_type', 'change_state_type', 'substitute_state'
                                                    'group_states', 'ungroup_state', 'paste']:
            if info['arg'].after is False:
                self._complex_action = True
                if info['arg'].action in ['group_states', 'paste']:
                    self.observe_model(info['arg'].action_parent_m)
                    # print "GSME observe: ", info['arg'].action_parent_m
                else:
                    self.observe_model(info['arg'].affected_models[0])
                    # print "GSME observe: ", info['arg'].affected_models[0]

                # assert not hasattr(self.state_action_signal.__func__, "affected_models")
                # assert not hasattr(self.state_action_signal.__func__, "target")
                self.state_action_signal.__func__.affected_models = info['arg'].affected_models
                self.state_action_signal.__func__.target = info['arg'].action_parent_m

    @ExtendedController.observe("action_signal", signal=True)
    def action_signal(self, model, prop_name, info):
        if isinstance(model, AbstractStateModel) and 'arg' in info and \
                info['arg'].action in ['change_root_state_type', 'change_state_type', 'substitute_state'
                                       'group_states', 'ungroup_state', 'paste']:
            self._complex_action = False
            self.relieve_model(model)
            self.adapt_complex_action(self.state_action_signal.__func__.target, info['arg'].action_parent_m)
            # print "GSME ACTION adapt to change"

    @ExtendedController.observe("state_machine", before=True)
    def state_machine_change_before(self, model, prop_name, info):
        if 'method_name' in info and info['method_name'] == 'root_state_change':
            method_name, model, result, arguments, instance = self._extract_info_data(info['kwargs'])

            if method_name in ['change_state_type', 'change_root_state_type']:
                self._complex_action = True
                if method_name == 'change_root_state_type':
                    state_model_to_be_changed = model.root_state
                else:
                    state_to_be_changed = arguments[1]
                    state_model_to_be_changed = gui_helper_state_machine.get_state_model_for_state(state_to_be_changed)
                self.observe_model(state_model_to_be_changed)

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

            if self._complex_action:
                return

            # The method causing the change raised an exception, thus nothing was changed
            if (isinstance(result, str) and "CRASH" in result) or isinstance(result, Exception):
                return

            if method_name == 'state_execution_status':
                state_v = self.canvas.get_view_for_model(model)
                if state_v:  # Children of LibraryStates are not modeled, yet
                    self.canvas.request_update(state_v, matrix=False)
                    self.canvas.perform_update()
            elif method_name == 'add_state':
                new_state = arguments[1]
                new_state_m = model.states[new_state.state_id]
                self.add_state_view_to_parent(new_state_m, model)
                self.canvas.perform_update()
            elif method_name == 'remove_state':
                parent_state = arguments[0]
                state_id = arguments[1]
                parent_v = self.canvas.get_view_for_core_element(parent_state)
                state_v = self.canvas.get_view_for_id(StateView, state_id, parent_v)
                if state_v:
                    parent_v = self.canvas.get_parent(state_v)
                    state_v.remove()
                    if parent_v:
                        self.canvas.request_update(parent_v)
                    self.canvas.perform_update()

            # ----------------------------------
            #           TRANSITIONS
            # ----------------------------------
            elif method_name == 'add_transition':
                transitions_models = model.transitions
                transition_id = result
                for transition_m in transitions_models:
                    if transition_m.transition.transition_id == transition_id:
                        self.add_transition_view_for_model(transition_m, model)
                        self.canvas.perform_update()
                        break
            elif method_name == 'remove_transition':
                self.remove_transition_view_from_parent_view(model)
                self.canvas.perform_update()
            elif method_name == 'transition_change':
                transition_m = model
                transition_v = self.canvas.get_view_for_model(transition_m)
                self.connect_transition_handle_to_state(transition_v, transition_m, transition_m.parent)
                self.canvas.perform_update()

            # ----------------------------------
            #           DATA FLOW
            # ----------------------------------
            elif method_name == 'add_data_flow':
                data_flow_models = model.data_flows
                data_flow_id = result
                for data_flow_m in data_flow_models:
                    if data_flow_m.data_flow.data_flow_id == data_flow_id:
                        self.add_data_flow_view_for_model(data_flow_m, model)
                        self.canvas.perform_update()
                        break
            elif method_name == 'remove_data_flow':
                self.remove_data_flow_view_from_parent_view(model)
                self.canvas.perform_update()
            elif method_name == 'data_flow_change':
                data_flow_m = model
                data_flow_v = self.canvas.get_view_for_model(data_flow_m)
                self.connect_data_flow_handle_to_state(data_flow_v, data_flow_m, data_flow_m.parent)
                self.canvas.perform_update()

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
                        self.canvas.perform_update()
                        break
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
                            self.canvas.perform_update()
                            break

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
                        self.canvas.perform_update()
                        break
            elif method_name == 'add_output_data_port':
                state_m = model
                state_v = self.canvas.get_view_for_model(state_m)
                for output_data_port_m in state_m.output_data_ports:
                    if output_data_port_m.data_port.data_port_id == result:
                        state_v.add_output_port(output_data_port_m)
                        self.canvas.request_update(state_v, matrix=False)
                        self.canvas.perform_update()
                        break
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
                            self.canvas.perform_update()
                            break
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
                            self.canvas.perform_update()
                            break
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
                        self.canvas.perform_update()
                        break
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
                            self.canvas.perform_update()
                            break

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
                self.canvas.perform_update()
            elif method_name in ['change_state_type', 'change_root_state_type']:
                pass
            elif method_name == 'parent':
                pass
            elif method_name == 'description':
                pass
            else:
                logger.debug("Method '%s' not caught in GraphicalViewer" % method_name)

            if method_name in ['add_state', 'add_transition', 'add_data_flow', 'add_outcome', 'add_input_data_port',
                               'add_output_data_port', 'add_scoped_variable', 'data_flow_change', 'transition_change']:
                try:
                    self._meta_data_changed(None, model, 'append_to_last_change', True)
                except Exception as e:
                    logger.error('Error while trying to emit meta data signal {}'.format(e))
                    raise

    @ExtendedController.observe("state_type_changed_signal", signal=True)
    def state_type_changed(self, old_state_m, prop_name, info):
        self._complex_action = False
        self.relieve_model(old_state_m)
        signal_msg = info['arg']
        new_state_m = signal_msg.new_state_m
        # print "state_type_changed relieve observer"
        self.adapt_complex_action(old_state_m, new_state_m)

    @lock_state_machine
    def adapt_complex_action(self, old_state_m, new_state_m):
        state_v = self.canvas.get_view_for_model(old_state_m)

        # If the root state has been changed, we recreate the whole state machine view
        if old_state_m is self.root_state_m:
            state_v.remove()

            # Create and and new root state view from new root state model
            self.root_state_m = new_state_m
            root_state_v = self.setup_state(self.root_state_m)
            self.canvas.request_update(root_state_v)

        # Otherwise we only look at the modified state and its children
        else:
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
                    elif not isinstance(child_v, NameView):  # Remove transitions and data flows but keep the NameView
                        self.canvas.remove(child_v)
            parent_v = self.canvas.get_parent(state_v)
            self.canvas.request_update(parent_v)

        self.canvas.perform_update()

        try:
            self._meta_data_changed(None, new_state_m, 'append_to_last_change', True)
        except Exception as e:
            logger.error('Error while trying to emit meta data signal {}'.format(e))

    @ExtendedController.observe("sm_selection_changed_signal", signal=True)
    def selection_change(self, model, prop_name, info):
        """Called when the selection was changed externally

        Updates the local selection and redraws.

        :param rafcon.gui.models.state_machine.StateMachineModel model: The state machine model
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

    @lock_state_machine
    def add_transition_view_for_model(self, transition_m, parent_state_m):
        parent_state_v = self.canvas.get_view_for_model(parent_state_m)

        new_transition_hierarchy_level = parent_state_v.hierarchy_level
        new_transition_v = TransitionView(transition_m, new_transition_hierarchy_level)

        # Draw transition above all other state elements
        self.canvas.add(new_transition_v, parent_state_v, index=None)

        self.add_transition(transition_m, new_transition_v, parent_state_m, parent_state_v)

    @lock_state_machine
    def add_data_flow_view_for_model(self, data_flow_m, parent_state_m):
        parent_state_v = self.canvas.get_view_for_model(parent_state_m)

        new_data_flow_hierarchy_level = parent_state_v.hierarchy_level
        new_data_flow_v = DataFlowView(data_flow_m, new_data_flow_hierarchy_level)

        # Draw data flow above NameView but beneath all other state elements
        self.canvas.add(new_data_flow_v, parent_state_v, index=1)
        self.add_data_flow(data_flow_m, new_data_flow_v, parent_state_m)

    @lock_state_machine
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

    @lock_state_machine
    def remove_data_flow_view_from_parent_view(self, parent_state_m):
        self._remove_connection_view(parent_state_m, False)

    @lock_state_machine
    def remove_transition_view_from_parent_view(self, parent_state_m):
        self._remove_connection_view(parent_state_m)

    @lock_state_machine
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
        return gui_helper_state_machine.delete_selected_elements(self.model)

    def setup_canvas(self):
        with self.model.state_machine.modification_lock():
            hash_before = self.model.mutable_hash().digest()
            self.setup_state(self.root_state_m, rel_pos=(10, 10))
            hash_after = self.model.mutable_hash().digest()
            if hash_before != hash_after:
                self._meta_data_changed(None, self.root_state_m, 'append_initial_change', True)
                logger.info("Opening the state machine caused some meta data to be generated, which will be stored "
                            " when the state machine is being saved.")

    @lock_state_machine
    def setup_state(self, state_m, parent=None, rel_pos=(0, 0), size=(100, 100), hierarchy_level=1):
        """Draws a (container) state with all its content

        Mainly contains the logic for drawing (e. g. reading and calculating values). The actual drawing process is
        done in the view, which is called from this method with the appropriate arguments.

        :param rafcon.gui.models.state.StateModel state_m: The state to be drawn
        :param rafcon.gui.models.state.StateModel parent: The parent state of `state_m`
        :param tuple rel_pos: The default relative position (x, y) if there is no relative position stored
        :param tuple size: The default size (width, height) if there is no size stored
        :param float hierarchy_level: The hierarchy level of the state
        """
        assert isinstance(state_m, AbstractStateModel)
        state_meta = state_m.get_meta_data_editor()

        # Use default values if no size information is stored
        if not isinstance(state_meta['size'], tuple):
            state_meta = state_m.set_meta_data_editor('size', size)

        size = state_meta['size']

        # Use default values if no position information is stored
        if not isinstance(state_meta['rel_pos'], tuple):
            state_meta = state_m.set_meta_data_editor('rel_pos', rel_pos)

        rel_pos = state_meta['rel_pos']

        state_v = StateView(state_m, size, hierarchy_level)

        # Draw state above data flows and NameView but beneath transitions
        index = 1 if not parent else len(state_m.state.parent.data_flows) + 1
        self.canvas.add(state_v, parent, index=index)
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

    @lock_state_machine
    def add_transitions(self, parent_state_m, hierarchy_level):
        """Draws the transitions belonging to a state

        The method takes all transitions from the given state and calculates their start and end point positions.
        Those are passed together with the waypoints to the view of the graphical editor.

        :param rafcon.gui.models.container_state.ContainerStateModel parent_state_m: The model of the container
            state, of which the transitions shall be drawn
        """
        parent_state_v = self.canvas.get_view_for_model(parent_state_m)
        assert isinstance(parent_state_v, StateView)
        for transition_m in parent_state_m.transitions:
            transition_v = TransitionView(transition_m, hierarchy_level)
            # Draw transition above all other state elements
            self.canvas.add(transition_v, parent_state_v, index=None)

            self.add_transition(transition_m, transition_v, parent_state_m, parent_state_v)

    @lock_state_machine
    def add_transition(self, transition_m, transition_v, parent_state_m, parent_state_v, use_waypoints=True):

        transition_meta = transition_m.get_meta_data_editor()

        try:
            if use_waypoints:
                waypoint_list = transition_meta['waypoints']

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

    @lock_state_machine
    def add_data_flows(self, parent_state_m, hierarchy_level):
        """Draw all data flows contained in the given container state

        The method takes all data flows from the given state and calculates their start and end point positions.
        Those are passed together with the waypoints to the view of the graphical editor.

        :param rafcon.gui.models.container_state.ContainerStateModel parent_state_m: The model of the container
            state, of which the data flows shall be drawn
        """
        parent_state_v = self.canvas.get_view_for_model(parent_state_m)
        assert isinstance(parent_state_v, StateView)
        for data_flow_m in parent_state_m.data_flows:
            data_flow_v = DataFlowView(data_flow_m, hierarchy_level)

            # Draw data flow above NameView but beneath all other state elements
            self.canvas.add(data_flow_v, parent_state_v, index=1)
            self.add_data_flow(data_flow_m, data_flow_v, parent_state_m)

    @lock_state_machine
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

    def check_focus_and_sm_selection_according_event(self, event):
        if not react_to_event(self.view, self.view.editor, event):
            return False
        if not gui_helper_state.gui_singletons.state_machine_manager_model.selected_state_machine_id == \
                self.model.state_machine.state_machine_id:
            return False
        return True

    @lock_state_machine
    def _add_data_port_to_selected_state(self, *event, **kwargs):
        if self.check_focus_and_sm_selection_according_event(event):
            data_port_type = None if 'data_port_type' not in kwargs else kwargs['data_port_type']
            gui_helper_state.add_data_port_to_selected_states(data_port_type)

    @lock_state_machine
    def _add_scoped_variable_to_selected_state(self, *event):
        if self.check_focus_and_sm_selection_according_event(event):
            gui_helper_state.add_scoped_variable_to_selected_states()

    @lock_state_machine
    def _add_outcome_to_selected_state(self, *event):
        if self.check_focus_and_sm_selection_according_event(event):
            gui_helper_state.add_outcome_to_selected_states()
