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
.. module:: modification_history
   :synopsis: A module that holds the controller to list and access the modification-history.

"""

from gi.repository import GObject
from gi.repository import Gtk
from builtins import str

from rafcon.gui import singleton as gui_singletons
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.gui.helpers.label import react_to_event
from rafcon.gui.helpers.meta_data import check_gaphas_state_machine_meta_data_consistency
from rafcon.gui.models.state_machine_manager import StateMachineManagerModel
from rafcon.gui.models.signals import MetaSignalMsg, StateTypeChangeSignalMsg, ActionSignalMsg
from rafcon.gui.singleton import global_gui_config
from rafcon.utils import log

logger = log.get_logger(__name__)

# TODO Comment


class ModificationHistoryTreeController(ExtendedController):
    string_substitution_dict = {}  # will be used to substitute strings for better and shorter tree columns

    def __init__(self, model, view):
        """Constructor
        :param model StateMachineModel should be exchangeable
        """
        assert isinstance(model, StateMachineManagerModel)

        ExtendedController.__init__(self, model, view)
        self.view_is_registered = False

        self._mode = 'branch'
        self.with_tree = True
        self.tree_folded = False

        assert self._mode in ['trail', 'branch']
        self.history_tree_store = Gtk.TreeStore(GObject.TYPE_STRING, GObject.TYPE_STRING, GObject.TYPE_STRING, GObject.TYPE_STRING, GObject.TYPE_STRING, GObject.TYPE_STRING, GObject.TYPE_PYOBJECT, GObject.TYPE_STRING, GObject.TYPE_STRING)
        if view is not None:
            view['history_tree'].set_model(self.history_tree_store)
        view['history_tree'].set_tooltip_column(8)

        # view.set_hover_expand(True)

        self.__my_selected_sm_id = None
        self._selected_sm_model = None

        self.doing_update = False
        self.no_cursor_observation = False
        self.next_activity_focus_self = True
        self.on_toggle_mode_check_gaphas_view_is_meta_data_consistent = True

        self.register()

    @ExtendedController.observe("selected_state_machine_id", assign=True)
    def state_machine_manager_notification(self, model, property, info):
        self.register()

    def register(self):
        """
        Change the state machine that is observed for new selected states to the selected state machine.
        :return:
        """
        # logger.debug("StateMachineEditionChangeHistory register state_machine old/new sm_id %s/%s" %
        #              (self.__my_selected_sm_id, self.model.selected_state_machine_id))

        # relieve old models
        if self.__my_selected_sm_id is not None:  # no old models available
            self.relieve_model(self._selected_sm_model.history)

        if self.model.selected_state_machine_id is not None and global_gui_config.get_config_value('HISTORY_ENABLED'):

            # set own selected state machine id
            self.__my_selected_sm_id = self.model.selected_state_machine_id

            # observe new models
            self._selected_sm_model = self.model.state_machines[self.__my_selected_sm_id]
            if self._selected_sm_model.history:
                self.observe_model(self._selected_sm_model.history)
                self.update(None, None, None)
            else:
                logger.warning("The history is enabled but not generated {0}. {1}"
                               "".format(self._selected_sm_model.state_machine.state_machine_id,
                                         self._selected_sm_model.state_machine.file_system_path))
        else:
            if self.__my_selected_sm_id is not None:
                self.doing_update = True
                self.history_tree_store.clear()
                self.doing_update = False
            self.__my_selected_sm_id = None
            self._selected_sm_model = None

    def register_view(self, view):
        super(ModificationHistoryTreeController, self).register_view(view)
        view['history_tree'].connect('cursor-changed', self.on_cursor_changed)
        view['undo_button'].connect('clicked', self.on_undo_button_clicked)
        view['redo_button'].connect('clicked', self.on_redo_button_clicked)
        view['reset_button'].connect('clicked', self.on_reset_button_clicked)
        if self._mode == 'branch':
            view['branch_checkbox'].set_active(True)
        view['branch_checkbox'].connect('toggled', self.on_toggle_mode)
        if self.tree_folded:
            view['folded_checkbox'].set_active(True)
        view['folded_checkbox'].connect('toggled', self.on_toggle_tree_folded)
        self.view_is_registered = True

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.gui.shortcut_manager.ShortcutManager shortcut_manager:
        """
        shortcut_manager.add_callback_for_action("undo", self.undo)
        shortcut_manager.add_callback_for_action("redo", self.redo)

    def on_undo_button_clicked(self, widget, event=None):
        self.undo(None, None)

    def on_redo_button_clicked(self, widget, event=None):
        self.redo(None, None)

    def on_reset_button_clicked(self, widget, event=None):
        logger.debug("Reset modification history of state machine of {0}.".format(self.model))
        with self._selected_sm_model.storage_lock:
            self._selected_sm_model.history.modifications.reset()

    def on_toggle_mode(self, widget, event=None):
        if self.view['branch_checkbox'].get_active():
            self._mode = 'branch'
        else:
            self._mode = 'trail'
        if self.on_toggle_mode_check_gaphas_view_is_meta_data_consistent:
            check_gaphas_state_machine_meta_data_consistency(self._selected_sm_model, with_logger_messages=True)
        logger.info("modification history mode: {0}".format(self._mode))
        self.update(None, None, None)

    def on_toggle_tree_folded(self, widget, event=None):
        self.tree_folded = self.view['folded_checkbox'].get_active()
        logger.info("modification history tree by default folded: {0}".format(self.tree_folded))
        self.update(None, None, None)

    def on_cursor_changed(self, widget):
        if self.no_cursor_observation:
            return
        (model, row) = self.view['history_tree'].get_selection().get_selected()
        self.no_cursor_observation = True
        if not self.doing_update:
            logger.debug(
                "The view jumps to the selected history element that would be situated on a right click menu in future")
            if len(self.history_tree_store) > 1 and model[row][1] is not None:
                history_id = int(model[row][1].split('.')[-1])

                # do recovery
                self.doing_update = True
                self._selected_sm_model.history.reset_to_history_id(history_id)
                self.doing_update = False
                self.update(None, None, None)
        self.no_cursor_observation = False

    def undo(self, key_value, modifier_mask, **kwargs):
        """Undo for selected state-machine if no state-source-editor is open and focused in states-editor-controller.

        :return: True if a undo was performed, False if focus on source-editor.
        :rtype: bool
        """
        # TODO re-organize as request to controller which holds source-editor-view or any parent to it
        for key, tab in gui_singletons.main_window_controller.get_controller('states_editor_ctrl').tabs.items():
            if tab['controller'].get_controller('source_ctrl') is not None and \
                    react_to_event(self.view, tab['controller'].get_controller('source_ctrl').view.textview,
                                   (key_value, modifier_mask)) or \
                tab['controller'].get_controller('description_ctrl') is not None and \
                    react_to_event(self.view, tab['controller'].get_controller('description_ctrl').view.textview,
                                   (key_value, modifier_mask)):
                return False
        if self._selected_sm_model is not None:
            self._selected_sm_model.history.undo()
            return True
        else:
            logger.debug("Undo is not possible now as long as no state_machine is selected.")

    def redo(self, key_value, modifier_mask, **kwargs):
        """Redo for selected state-machine if no state-source-editor is open and focused in states-editor-controller.

        :return: True if a redo was performed, False if focus on source-editor.
        :rtype: bool
        """
        # TODO re-organize as request to controller which holds source-editor-view or any parent to it
        for key, tab in gui_singletons.main_window_controller.get_controller('states_editor_ctrl').tabs.items():
            if tab['controller'].get_controller('source_ctrl') is not None and \
                    react_to_event(self.view, tab['controller'].get_controller('source_ctrl').view.textview,
                                   (key_value, modifier_mask)) or \
                tab['controller'].get_controller('description_ctrl') is not None and \
                    react_to_event(self.view, tab['controller'].get_controller('description_ctrl').view.textview,
                                   (key_value, modifier_mask)):
                return False
        if self._selected_sm_model is not None:
            self._selected_sm_model.history.redo()
            return True
        else:
            logger.debug("Redo is not possible now as long as no state_machine is selected.")

    @ExtendedController.observe("modifications", after=True)
    def update(self, model, prop_name, info):
        """ The method updates the history (a Gtk.TreeStore) which is the model of respective TreeView.
        It functionality is strongly depends on a consistent history-tree hold by a ChangeHistory-Class.
        """
        # logger.debug("History changed %s\n%s\n%s" % (model, prop_name, info))
        if self.parent is not None and hasattr(self.parent, "focus_notebook_page_of_controller"):
            # request focus -> which has not have to be satisfied
            self.parent.focus_notebook_page_of_controller(self)

        if self._selected_sm_model is None or self._selected_sm_model.history.fake or \
                info is not None and info.method_name not in ["insert_action", "undo", "redo", "reset"]:
            return

        self.doing_update = True
        self.history_tree_store.clear()
        self.list_tree_iter = {}
        trail_actions = self._selected_sm_model.history.modifications.get_current_branch_history_ids()

        def insert_history_element(history_tree_elem, parent_tree_item, init_branch=False):
            """ The function insert a action with respective arguments (e.g. method_name, instance) into a TreeStore.
                - final trail or tree specific gtk-tree-levels -> 'trail' has no levels as well as 'branch' without tree
                - defines which element is marked as active
                - generate branch labels with version-id
            """
            action = history_tree_elem.action
            history_id = history_tree_elem.history_id
            model = action.before_overview.get_affected_model()
            method_name = action.before_overview.get_cause()
            instance = action.before_overview.get_affected_core_element()
            parameters = []
            tool_tip = None
            if action.before_overview.type == 'signal':
                if isinstance(action.before_overview.get_signal_message(), MetaSignalMsg):
                    parameters.append(str(action.meta))
                elif isinstance(action.before_overview.get_signal_message(), ActionSignalMsg):
                    if action.action_type not in ['paste', 'cut']:
                        parameters.append(str(action.before_overview.get_signal_message().kwargs))
                else:
                    logger.warning("no parameters defined for signal: {0}".format(action.before_overview.get_signal_message()))
            else:
                for index, value in enumerate(action.before_overview.get_method_args()):
                    if not index == 0:
                        parameters.append(str(value))
            for name, value in action.before_overview.get_method_kwargs().items():
                parameters.append("{0}: {1}".format(name, value))

            if hasattr(action, 'action_type') and action.action_type == "script_text" and hasattr(action, 'script_diff'):
                line = ""
                for elem in action.script_diff.split('\n'):
                    line = elem
                    if not line.replace(' ', '') == '' and (line.startswith('+') or line.startswith('-')):
                        break
                tool_tip = action.script_diff
                parameters = [line]  # + "\n -> [hover for source script diff in tooltip.]"]
            if hasattr(action, 'action_type') and action.action_type == "description" and hasattr(action, 'description_diff'):
                line = ""
                for elem in action.description_diff.split('\n'):
                    line = elem
                    if not line.replace(' ', '') == '' and (line.startswith('+') or line.startswith('-')):
                        break
                tool_tip = action.description_diff
                parameters = [line]  # + "\n -> [hover for source script diff in tooltip.]"]

            # find active actions in to be marked in view
            active = history_id in  self._selected_sm_model.history.modifications.get_executed_history_ids()

            # generate label to mark branches
            version_label = history_id
            if init_branch:
                version_label = 'b.' + str(history_id)
                tool_tip = "The element '{0}' starts a new branch of actions.".format(version_label)

            tree_row_iter = self.new_change(model, str(method_name).replace('_', ' '), instance, info, version_label, active,
                                            parent_tree_item, ', '.join(parameters), tool_tip)
            self.list_tree_iter[history_id] = (tree_row_iter, parent_tree_item)
            return tree_row_iter

        def insert_history_tree_starting_at_id(history_id, parent_tree_item=None):
            """ The function defines linkage of history-tree-elements in a gtk-tree-view to create a optimal overview.
            """
            if not self._selected_sm_model.history.modifications.current_history_element:
                # history only contains the ActionDummy element
                return
            next_id = history_id
            while next_id is not None:
                history_tree_elem = self._selected_sm_model.history.modifications.get_element_for_history_id(next_id)
                next_id = history_tree_elem.next_id
                prev_tree_elem = None
                if history_tree_elem.prev_id is not None:
                    prev_tree_elem = self._selected_sm_model.history.modifications.get_previous_element(history_tree_elem)
                action = history_tree_elem.action
                in_trail = history_tree_elem.history_id in trail_actions
                if in_trail:
                    # in trail parent is always None
                    if prev_tree_elem is not None and prev_tree_elem.old_next_ids:
                        child_tree_item = insert_history_element(history_tree_elem, None, init_branch=True)
                    else:
                        child_tree_item = insert_history_element(history_tree_elem, None)
                elif prev_tree_elem is not None and prev_tree_elem.old_next_ids:
                    # branch and not trail -> level + 1 ... child of prev_id -> parent_iter is prev_id_iter
                    prev_id_iter = self.list_tree_iter[prev_tree_elem.history_id][0]
                    child_tree_item = insert_history_element(history_tree_elem, prev_id_iter, init_branch=True)
                elif prev_tree_elem is not None and not prev_tree_elem.old_next_ids:
                    # no branch and not trail
                    prev_prev_tree_elem = self._selected_sm_model.history.modifications.get_previous_element(prev_tree_elem)
                    branch_limit_for_extra_ident_level = 1 if prev_tree_elem.prev_id in trail_actions else 0
                    if len(prev_prev_tree_elem.old_next_ids) > branch_limit_for_extra_ident_level:
                        # -> level + 1 as previous element, because to many branches -> so prev_id iter as parent_iter
                        iter_of_prev_id = self.list_tree_iter[prev_tree_elem.history_id][0]
                        child_tree_item = insert_history_element(history_tree_elem, iter_of_prev_id)
                    else:
                        # -> same level as previous element -> so same parent_iter as prev_id
                        parent_iter_of_prev_id = self.list_tree_iter[prev_tree_elem.history_id][1]
                        child_tree_item = insert_history_element(history_tree_elem, parent_iter_of_prev_id)
                else:
                    logger.warning("relative level could not be found -> this should not happen")
                    child_tree_item = insert_history_element(history_tree_elem, parent_tree_item)

                if history_tree_elem.old_next_ids and self._mode == 'branch':
                    old_next_ids = history_tree_elem.old_next_ids
                    for old_next_id in old_next_ids:
                        insert_history_tree_starting_at_id(old_next_id, child_tree_item)

        insert_history_tree_starting_at_id(history_id=0)

        # set selection of Tree
        if self._selected_sm_model.history.modifications.current_history_element and len(self.history_tree_store) > 1:
            searched_row_history_id = self._selected_sm_model.history.modifications.current_history_element.history_id
            row_number = 0
            for action_entry in self.history_tree_store:
                # compare history_id
                if int(action_entry[1].split('.')[-1]) == searched_row_history_id:
                    self.view['history_tree'].set_cursor(row_number)
                    break
                row_number += 1

        # set colors of Tree
        # - is state full and all element which are open to be re-done gray
        self.doing_update = False

    @staticmethod
    def get_color_active(active):
        if active:
            return "white"
        else:
            # foreground = "gray"
            return "#707070"

    def new_change(self, model, method_name, instance, info, history_id, active, parent_tree_item, parameters,
                   tool_tip=None):
        # Nr, Instance, Method, Details, model

        # TODO may useful tooltip
        trail_tip = "any modification-state of state machine can be reached by clicking on respective action"
        branch_tip = "-> actions of old branches can be reached by clicking on respective action"

        # active-state based coloring
        foreground = self.get_color_active(active)

        # force tree to list if 'trail' or without tree enabled
        if self._mode == 'trail' or (self._mode == 'branch' and not self.with_tree):
            parent_tree_item = None

        # different ordering of elements if branch -> seems more natural
        used_func = self.history_tree_store.prepend
        if parent_tree_item is not None:
            used_func = self.history_tree_store.append
        history_row_iter = used_func(parent_tree_item, ('',
                                                        str(history_id),  # '',  # version
                                                        method_name,
                                                        str(instance),
                                                        str(info),
                                                        foreground,
                                                        str(model),
                                                        str(parameters),
                                                        tool_tip))

        # handle expand-mode
        if parent_tree_item is not None:
            if not self.tree_folded:
                history_row_path = self.history_tree_store.get_path(history_row_iter)
                self.view['history_tree'].expand_to_path(history_row_path)
        return history_row_iter
