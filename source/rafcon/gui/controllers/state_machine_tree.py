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
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: state_machine_tree
   :synopsis: A module that holds the controller to access a state machine overview by a TreeView.

"""

from gi.repository import GObject
from gi.repository import Gtk
from gi.repository import Gdk
from builtins import range
from functools import partial

from rafcon.core.states.state import State
from rafcon.core.states.library_state import LibraryState
from rafcon.core.states.state import StateType
from rafcon.gui.clipboard import global_clipboard
from rafcon.gui.controllers.right_click_menu.state import StateMachineTreeRightClickMenuController
from rafcon.gui.controllers.utils.tree_view_controller import TreeViewController
import rafcon.gui.helpers.state_machine as gui_helper_state_machine
from rafcon.gui.helpers.label import react_to_event
from rafcon.gui.models import ContainerStateModel, LibraryStateModel, AbstractStateModel
from rafcon.gui.models.state_machine_manager import StateMachineManagerModel
from rafcon.gui.utils.notification_overview import NotificationOverview
from rafcon.utils import log

logger = log.get_logger(__name__)


class StateMachineTreeController(TreeViewController):
    """Controller handling the state machine tree.

    :param rafcon.gui.models.state_machine_manager.StateMachineManagerModel model: The state machine manager model,
        holding data regarding state machines. Should be exchangeable.
    :param rafcon.gui.views.state_machine_tree.StateMachineTreeView view: The GTK view showing the state machine tree.
    """

    # TODO hold expansion if refresh all is performed -> minor feature (use storage_path instate of state_machine_id)
    # TODO hold expansion type changes are re- and undone -> minor feature which also depends on modification-history
    NAME_STORAGE_ID = 0
    ID_STORAGE_ID = 1
    TYPE_NAME_STORAGE_ID = 2
    MODEL_STORAGE_ID = 3
    STATE_PATH_STORAGE_ID = 4
    CORE_ELEMENT_CLASS = State

    def __init__(self, model, view):
        assert isinstance(model, StateMachineManagerModel)
        tree_store = Gtk.TreeStore(GObject.TYPE_STRING, GObject.TYPE_STRING, GObject.TYPE_STRING, GObject.TYPE_PYOBJECT, GObject.TYPE_STRING)
        super(StateMachineTreeController, self).__init__(model, view, view, tree_store)

        self.add_controller("state_right_click_ctrl", StateMachineTreeRightClickMenuController(model, view))

        self.view_is_registered = False

        # view.set_hover_expand(True)
        self.state_row_iter_dict_by_state_path = {}
        self.__my_selected_sm_id = None
        self._selected_sm_model = None

        self.__expansion_state = {}

        self._ongoing_complex_actions = []

        self._state_which_is_updated = None

        self.register()

    def register_view(self, view):
        """Called when the view was registered"""
        super(StateMachineTreeController, self).register_view(view)
        self.view.connect('button_press_event', self.mouse_click)
        self.view_is_registered = True
        self.update(with_expand=True)

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.gui.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
            between shortcuts and actions.
        """
        shortcut_manager.add_callback_for_action("delete", self._delete_selection)
        shortcut_manager.add_callback_for_action("add", partial(self._add_new_state, state_type=StateType.EXECUTION))
        shortcut_manager.add_callback_for_action("add2", partial(self._add_new_state, state_type=StateType.HIERARCHY))
        shortcut_manager.add_callback_for_action("copy", self.copy_action_callback)
        shortcut_manager.add_callback_for_action("cut", self.cut_action_callback)
        shortcut_manager.add_callback_for_action("paste", self.paste_action_callback)

    def register(self):
        """Change the state machine that is observed for new selected states to the selected state machine."""
        self._do_selection_update = True
        # relieve old model
        if self.__my_selected_sm_id is not None:  # no old models available
            self.relieve_model(self._selected_sm_model)
        # set own selected state machine id
        self.__my_selected_sm_id = self.model.selected_state_machine_id
        if self.__my_selected_sm_id is not None:
            # observe new model
            self._selected_sm_model = self.model.state_machines[self.__my_selected_sm_id]
            self.observe_model(self._selected_sm_model)  # for selection
            self.update()
        else:
            self._selected_sm_model = None
            self.state_row_iter_dict_by_state_path.clear()
            self.tree_store.clear()
        self._do_selection_update = False

    def paste_action_callback(self, *event, **kwargs):
        """Callback method for paste action"""
        if react_to_event(self.view, self.tree_view, event):
            sm_selection, _ = self.get_state_machine_selection()
            # only list specific elements are cut by widget
            if len(sm_selection.states) == 1:
                global_clipboard.paste(sm_selection.get_selected_state(), limited=['states', 'transitions', 'data_flows'])
            else:
                logger.warning("Please select only one state to paste.")
            return True

    def _add_new_state(self, *event, **kwargs):
        """Triggered when shortcut keys for adding a new state are pressed, or Menu Bar "Edit, Add State" is clicked.

        Adds a new state only if the the state machine tree is in focus.
        """
        if react_to_event(self.view, self.view['state_machine_tree_view'], event):
            state_type = StateType.EXECUTION if 'state_type' not in kwargs else kwargs['state_type']
            gui_helper_state_machine.add_new_state(self._selected_sm_model, state_type)
            return True

    def _delete_selection(self, *event, **kwargs):
        if react_to_event(self.view, self.view['state_machine_tree_view'], event):
            return gui_helper_state_machine.delete_selected_elements(self._selected_sm_model)

    def selection_changed(self, widget, event=None):
        """Notify state machine about tree view selection"""
        # do not forward cursor selection updates if state update is running
        # TODO maybe make this generic for respective abstract controller
        if self._state_which_is_updated:
            return
        super(StateMachineTreeController, self).selection_changed(widget, event)

    @TreeViewController.observe("state_machine", after=True)
    def states_update(self, model, prop_name, info):
        overview = NotificationOverview(info)
        if not overview.caused_modification():
            return

        if overview.get_affected_property() == 'state' and \
                overview.get_cause() in ["name"]:  # , "add_state", "remove_state"]:
            self.update_tree_store_row(overview.get_affected_model())
        # TODO check the work around for get_library_root_state -> maybe the notifications can be avoided if upper lib
        elif overview.get_affected_property() == 'state' and not overview.get_affected_model().state.get_next_upper_library_root_state() and \
                overview.get_cause() in ["add_state", "remove_state"]:
            self.update(overview.get_affected_model())

    @TreeViewController.observe("state_meta_signal", signal=True)
    def state_meta_update(self, model, prop_name, info):
        meta_signal_message = info['arg']
        model = meta_signal_message.notification.model
        if meta_signal_message.change == 'show_content':
            # store selection and expansion state of tree
            self.store_expansion_state()
            selected_states = self._selected_sm_model.selection.states

            # update library state starting in its parent
            self.update(model.parent if model.parent is not None else model)

            # recover selection and expansion state of tree
            self.redo_expansion_state(ignore_not_existing_rows=True)
            for state_m in selected_states:
                self._selected_sm_model.selection.add(state_m)
            self.update_selection_sm_prior()

    @TreeViewController.observe("state_machine", before=True)
    def states_update_before(self, model, prop_name, info):
        overview = NotificationOverview(info)
        if not overview.caused_modification():
            return

        if overview.get_affected_property() == 'state' and \
                overview.get_cause() in ["change_state_type"]:
            changed_model = self._selected_sm_model.get_state_model_by_path(overview.get_method_args()[1].get_path())
            self.observe_model(changed_model)

    @TreeViewController.observe("state_action_signal", signal=True)
    def state_action_signal(self, model, prop_name, info):
        # TODO check if this is the right way or only some of them
        if 'arg' in info and info['arg'].action in ['change_root_state_type', 'change_state_type', 'substitute_state',
                                                    'group_states', 'ungroup_state', 'paste', 'undo/redo']:
            if info['arg'].after is False:
                self._ongoing_complex_actions.append(info['arg'].action)
                if info['arg'].action in ['group_states', 'paste']:
                    self.observe_model(info['arg'].action_parent_m)
                else:
                    self.observe_model(info['arg'].affected_models[0])

    @TreeViewController.observe("action_signal", signal=True)
    def action_signal(self, model, prop_name, info):
        # TODO check why the expansion of tree is not recovered
        if not (isinstance(model, AbstractStateModel) and 'arg' in info and info['arg'].after):
            return

        action = info['arg'].action
        if action in ['substitute_state', 'group_states', 'ungroup_state', 'paste', 'undo/redo']:
            target_state_m = info['arg'].action_parent_m
        elif action in ['change_state_type', 'change_root_state_type']:
            target_state_m = info['arg'].affected_models[-1]
        else:
            return

        self._ongoing_complex_actions.remove(action)
        if not self._ongoing_complex_actions:
            self.relieve_model(model)

            # TODO check selection warnings if not all a the tree is recreated
            self.update()  # if target_state_m.state.is_root_state else self.update(target_state_m.parent)

    @TreeViewController.observe("root_state", assign=True)
    def state_machine_notification(self, model, property, info):
        self.update(model.root_state)

    @TreeViewController.observe("selected_state_machine_id", assign=True)
    def state_machine_manager_notification(self, model, property, info):
        # store expansion state
        self.store_expansion_state()
        # register new state machine
        self.register()
        self.assign_notification_selection(None, None, None)
        # redo expansion state
        self.redo_expansion_state()

    def store_expansion_state(self):
        if self.__my_selected_sm_id is None:
            return

        try:
            act_expansion_state = {}
            for state_path, state_row_iter in self.state_row_iter_dict_by_state_path.items():
                state_row_path = self.tree_store.get_path(state_row_iter)
                if state_row_path is not None:
                    act_expansion_state[state_path] = self.view.row_expanded(state_row_path)
                else:
                    if self._selected_sm_model and \
                            self._selected_sm_model.state_machine.get_state_by_path(state_path, as_check=True):
                        # happens if refresh all is performed -> otherwise it is a error
                        logger.debug("State not in StateMachineTree but in StateMachine, {0}. {1}, {2}"
                                     "".format(state_path, state_row_path, state_row_iter))
            self.__expansion_state[self.__my_selected_sm_id] = act_expansion_state
        except TypeError:
            logger.error("Expansion state of state machine {0} could not be stored".format(self.__my_selected_sm_id))

    def redo_expansion_state(self, ignore_not_existing_rows=False):
        """ Considers the tree to be collapsed and expand into all tree item with the flag set True """

        def set_expansion_state(state_path):
            state_row_iter = self.state_row_iter_dict_by_state_path[state_path]
            if state_row_iter:  # may elements are missing afterwards
                state_row_path = self.tree_store.get_path(state_row_iter)
                self.view.expand_to_path(state_row_path)

        if self.__my_selected_sm_id is not None and self.__my_selected_sm_id in self.__expansion_state:
            expansion_state = self.__expansion_state[self.__my_selected_sm_id]
            try:
                for state_path, state_row_expanded in expansion_state.items():
                    if state_path in self.state_row_iter_dict_by_state_path:
                        if state_row_expanded:
                            set_expansion_state(state_path)
                    else:
                        if not ignore_not_existing_rows and self._selected_sm_model and \
                                self._selected_sm_model.state_machine.get_state_by_path(state_path, as_check=True):
                            state = self._selected_sm_model.state_machine.get_state_by_path(state_path)
                            if isinstance(state, LibraryState) or state.is_root_state_of_library or \
                                    state.get_next_upper_library_root_state():
                                continue
                            logger.error("State not in StateMachineTree but in StateMachine, {0}.".format(state_path))

            except (TypeError, KeyError):
                logger.error("Expansion state of state machine {0} could not be restored"
                             "".format(self.__my_selected_sm_id))

    def update(self, changed_state_model=None, with_expand=False):
        """Checks if all states are in tree and if tree has states which were deleted

        :param changed_state_model: Model that row has to be updated
        :param with_expand: The expand flag for the tree
        """
        if not self.view_is_registered:
            return

        # define initial state-model for update
        if changed_state_model is None:
            # reset all
            parent_row_iter = None
            self.state_row_iter_dict_by_state_path.clear()
            self.tree_store.clear()
            if self._selected_sm_model:
                changed_state_model = self._selected_sm_model.root_state
            else:
                return
        else:  # pick
            if changed_state_model.state.is_root_state:
                parent_row_iter = self.state_row_iter_dict_by_state_path[changed_state_model.state.get_path()]
            else:
                if changed_state_model.state.is_root_state_of_library:
                    # because either lib-state or lib-state-root is in tree the next higher hierarchy state is updated
                    changed_upper_state_m = changed_state_model.parent.parent
                else:
                    changed_upper_state_m = changed_state_model.parent
                # TODO check the work around of the next 2 lines while refactoring -> it is a check to be more robust
                while changed_upper_state_m.state.get_path() not in self.state_row_iter_dict_by_state_path:
                    # show Warning because because avoided method states_update
                    logger.warning("Take a parent state because this is not in.")
                    changed_upper_state_m = changed_upper_state_m.parent
                parent_row_iter = self.state_row_iter_dict_by_state_path[changed_upper_state_m.state.get_path()]

        # do recursive update
        self.insert_and_update_recursively(parent_row_iter, changed_state_model, with_expand)

    def get_row_iter_for_state_model(self, state_model):
        if state_model.state.get_path() not in self.state_row_iter_dict_by_state_path:
            if isinstance(state_model, LibraryStateModel) and \
                    state_model.state_copy.state.get_path() in self.state_row_iter_dict_by_state_path:
                return self.state_row_iter_dict_by_state_path[state_model.state_copy.state.get_path()]
            else:
                logger.error("For state model {0} no row iter could be found to be updated.".format(state_model))
                return
        else:
            return self.state_row_iter_dict_by_state_path[state_model.state.get_path()]

    def update_tree_store_row(self, state_model):
        state_row_iter = self.get_row_iter_for_state_model(state_model)
        if state_row_iter is None:
            return
        state_row_iter = self.state_row_iter_dict_by_state_path[state_model.state.get_path()]
        state_row_path = self.tree_store.get_path(state_row_iter)

        if not type(state_model.state).__name__ == self.tree_store[state_row_path][self.TYPE_NAME_STORAGE_ID] or \
                not state_model.state.name == self.tree_store[state_row_path][self.NAME_STORAGE_ID]:
            self.tree_store[state_row_path][self.NAME_STORAGE_ID] = state_model.state.name
            self.tree_store[state_row_path][self.TYPE_NAME_STORAGE_ID] = type(state_model.state).__name__
            self.tree_store[state_row_path][self.MODEL_STORAGE_ID] = state_model
            self.tree_store.row_changed(state_row_path, state_row_iter)

    def show_content(self, state_model):
        """Check state machine tree specific show content flag.

        Is returning true if the upper most library state of a state model has a enabled show content flag or if there
        is no library root state above this state.

        :param rafcon.gui.models.abstract_state.AbstractStateModel state_model: The state model to check
        """
        upper_most_lib_state_m = None
        if isinstance(state_model, LibraryStateModel):
            uppermost_library_root_state = state_model.state.get_uppermost_library_root_state()
            if uppermost_library_root_state is None:
                upper_most_lib_state_m = state_model
            else:
                upper_lib_state = uppermost_library_root_state.parent
                upper_most_lib_state_m = self._selected_sm_model.get_state_model_by_path(upper_lib_state.get_path())
        if upper_most_lib_state_m:
            return upper_most_lib_state_m.show_content()
        else:
            return True

    def insert_and_update_recursively(self, parent_iter, state_model, with_expand=False):
        """ Insert and/or update the handed state model in parent tree store element iterator

        :param parent_iter: Parent tree store iterator the insert should be performed in
        :param StateModel state_model: Model of state that has to be insert and/or updated
        :param bool with_expand: Trigger to expand tree
        :return:
        """
        # the case handling of this method
        # 0 - create - common state
        # 0.1 - modify attributes of common state which is already in the list
        # 1 - create - library with show content -> add library root state
        # 2 - create - library without show content -> add library state
        # 3 - in as library with show content -> switch library without show content, remove children + LibRootState
        # 3.1 - in as library with show content -> nothing to do
        # 4 - in as library without show content -> switch library with show content, add children + remove LibState
        # 4.1 - in as library without show content -> nothing to do

        # if state model is LibraryStateModel with enabled show content state_model becomes the library root state model
        if isinstance(state_model, LibraryStateModel) and self.show_content(state_model) and state_model.state_copy_initialized:
            _state_model = state_model
            state_model = state_model.state_copy
        else:
            _state_model = state_model

        if self._state_which_is_updated is None:
            self._state_which_is_updated = _state_model

        # TODO remove this workaround for removing LibraryStateModel or there root states by default
        if isinstance(_state_model, LibraryStateModel) and _state_model.state_copy_initialized:
            state_row_iter = None
            if _state_model.state.get_path() in self.state_row_iter_dict_by_state_path:
                state_row_iter = self.state_row_iter_dict_by_state_path[_state_model.state.get_path()]
            if state_model.state.get_path() in self.state_row_iter_dict_by_state_path:
                state_row_iter = self.state_row_iter_dict_by_state_path[state_model.state.get_path()]

            if state_row_iter:
                self.remove_tree_children(state_row_iter)
                del self.state_row_iter_dict_by_state_path[self.tree_store.get_value(state_row_iter, self.STATE_PATH_STORAGE_ID)]
                self.tree_store.remove(state_row_iter)

        # if library root state is used instate of library state show both in type and state id
        _state_id = _state_model.state.state_id
        # _state_id += '' if _state_model is state_model else '/' + state_model.state.state_id  TODO enable this line
        _state_type = type(_state_model.state).__name__
        _state_type += '' if _state_model is state_model else '/' + type(state_model.state).__name__

        # check if in
        state_path = state_model.state.get_path()
        if state_path not in self.state_row_iter_dict_by_state_path:
            # if not in -> insert it
            state_row_iter = self.tree_store.insert_before(parent=parent_iter, sibling=None,
                                                           row=(state_model.state.name,
                                                                _state_id,
                                                                _state_type,
                                                                _state_model,
                                                                state_model.state.get_path()))
            self.state_row_iter_dict_by_state_path[state_path] = state_row_iter
            if with_expand:
                parent_path = self.tree_store.get_path(state_row_iter)
                self.view.expand_to_path(parent_path)
        else:
            # if in -> check if up to date
            state_row_iter = self.state_row_iter_dict_by_state_path[state_model.state.get_path()]
            self.update_tree_store_row(state_model)

        # check children
        # - check if ALL children are in
        if isinstance(state_model, ContainerStateModel):
            for child_state_id, child_state_model in state_model.states.items():
                self.insert_and_update_recursively(state_row_iter, child_state_model, with_expand=False)

        # - check if TOO MUCH children are in
        # if state_model.state.get_library_root_state() is not None or isinstance(state_model, LibraryStateModel):
        for n in reversed(range(self.tree_store.iter_n_children(state_row_iter))):
            child_iter = self.tree_store.iter_nth_child(state_row_iter, n)
            child_state_path = self.tree_store.get_value(child_iter, self.STATE_PATH_STORAGE_ID)
            child_model = None
            if self._selected_sm_model.state_machine.get_state_by_path(child_state_path, as_check=True):
                child_model = self._selected_sm_model.get_state_model_by_path(child_state_path)
            child_id = self.tree_store.get_value(child_iter, self.ID_STORAGE_ID)

            # check if there are left over rows of old states (switch from HS or CS to S and so on)
            show_content_flag = isinstance(child_model, LibraryStateModel) and self.show_content(child_model) and \
                                child_model.state_copy_initialized
            child_is_lib_with_show_content = isinstance(child_model, LibraryStateModel) and show_content_flag
            child_is_lib_without_show_content = isinstance(child_model, LibraryStateModel) and not show_content_flag

            if not isinstance(state_model, ContainerStateModel) or child_model is None or \
                    child_id not in state_model.states and not child_is_lib_with_show_content \
                    and isinstance(child_model, LibraryStateModel) and child_id == child_model.state.state_copy.state_id \
                    or child_is_lib_without_show_content and child_id == child_model.state.state_copy.state_id or \
                    isinstance(_state_model, LibraryStateModel) and not self.show_content(_state_model) or \
                    child_model.state.is_root_state_of_library and not self.show_content(child_model.parent):

                self.remove_tree_children(child_iter)
                del self.state_row_iter_dict_by_state_path[self.tree_store.get_value(child_iter, self.STATE_PATH_STORAGE_ID)]
                self.tree_store.remove(child_iter)

        if self._state_which_is_updated is _state_model:
            self._state_which_is_updated = None

    def remove_tree_children(self, child_tree_iter):
        for n in reversed(range(self.tree_store.iter_n_children(child_tree_iter))):
            child_iter = self.tree_store.iter_nth_child(child_tree_iter, n)
            if self.tree_store.iter_n_children(child_iter):
                self.remove_tree_children(child_iter)
            del self.state_row_iter_dict_by_state_path[self.tree_store.get_value(child_iter, self.STATE_PATH_STORAGE_ID)]
            # self.tree_store.remove(child_iter)

    def get_state_machine_selection(self):
        """Getter state machine selection

        :return: selection object, filtered set of selected states
        :rtype: rafcon.gui.selection.Selection, set
        """
        if self._selected_sm_model:
            return self._selected_sm_model.selection, self._selected_sm_model.selection.states
        else:
            return None, set()

    def mouse_click(self, widget, event=None):
        if event.type == Gdk.EventType._2BUTTON_PRESS:
            return self._handle_double_click(event)

    def _handle_double_click(self, event):
        """ Double click with left mouse button focuses the state and toggles the collapse status"""
        if event.get_button()[1] == 1:  # Left mouse button
            path_info = self.tree_view.get_path_at_pos(int(event.x), int(event.y))
            if path_info:  # Valid entry was clicked on
                path = path_info[0]
                iter = self.tree_store.get_iter(path)
                state_model = self.tree_store.get_value(iter, self.MODEL_STORAGE_ID)

                # Set focus to StateModel
                selection = self._selected_sm_model.selection
                selection.focus = state_model

                # Toggle collapse status if applicable for this kind of state
                if self.view.row_expanded(path):
                    self.view.collapse_row(path)
                else:
                    if isinstance(state_model, ContainerStateModel) or \
                                    isinstance(state_model, LibraryStateModel) and self.show_content(state_model):
                        self.view.expand_to_path(path)

    @TreeViewController.observe("sm_selection_changed_signal", signal=True)
    def assign_notification_selection(self, state_machine_m, signal_name, signal_msg):
        if state_machine_m is None and self._selected_sm_model and \
                self._selected_sm_model.selection.get_selected_state():
            self.update_selection_sm_prior()
        elif signal_msg and self.tree_store.get_iter_first():
            if any(issubclass(cls, self.CORE_ELEMENT_CLASS) for cls in signal_msg.arg.affected_core_element_classes):
                self.update_selection_sm_prior()
