# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Sebastian Brunner <sebastian.brunner@dlr.de>

import gtk
import gobject
import copy

from gtkmvc import ModelMT

from rafcon.core.id_generator import generate_semantic_data_key
from rafcon.core.states.library_state import LibraryState

from rafcon.gui.controllers.utils.tree_view_controller import TreeViewController
from rafcon.gui.models import AbstractStateModel
from rafcon.gui.views.state_editor.semantic_data_editor import SemanticDataEditorView
from rafcon.gui.helpers.label import react_to_event
from functools import partial

from rafcon.utils import log
from rafcon.utils.vividict import Vividict
from rafcon.utils import type_helpers

logger = log.get_logger(__name__)


class SemanticDataEditorController(TreeViewController):

    """ A controller class to visualize and edit the semantic data of a state

    """
    KEY_STORAGE_ID = 0
    VALUE_STORAGE_ID = 1
    IS_DICT_STORAGE_ID = 2
    ID_STORAGE_ID = 3

    def __init__(self, model, view):
        """Constructor
        """
        assert isinstance(model, AbstractStateModel)
        assert isinstance(view, SemanticDataEditorView)

        if isinstance(model.state, LibraryState):
            model_to_observe = model.state_copy
        else:
            model_to_observe = model

        # define tree store with the values in [key, value Is Dict]
        tree_store = gtk.TreeStore(str, str, bool, gobject.TYPE_PYOBJECT)

        super(SemanticDataEditorController, self).__init__(model_to_observe, view, view["semantic_data_tree_view"],
                                                           tree_store, logger)
        self.semantic_data_counter = 0

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application

        :param rafcon.gui.views.state_editor.semantic_data_editor.SemanticDataEditorView view: An view to show all
            semantic data of a state
        """
        super(SemanticDataEditorController, self).register_view(view)

        if isinstance(self.model.state, LibraryState) or self.model.state.get_library_root_state():
            view['new_entry'].set_sensitive(False)
            view['new_dict_entry'].set_sensitive(False)
            view['delete_entry'].set_sensitive(False)

            for i in range(len(view['semantic_data_tree_view'].get_columns())):
                current_column = view['semantic_data_tree_view'].get_column(i)
                print current_column, dir(current_column)
                current_column.get_cell_renderers()[0].set_property('editable', False)


        view['new_entry'].connect('clicked', self.on_add, False)
        view['new_dict_entry'].connect('clicked', self.on_add, True)
        view['delete_entry'].connect('clicked', self.on_remove)
        self._apply_value_on_edited_and_focus_out(self.widget_columns[view.KEY_COLUMN_ID].get_cell_renderers()[0],
                                                  self.key_edited)
        self._apply_value_on_edited_and_focus_out(self.widget_columns[view.VALUE_COLUMN_ID].get_cell_renderers()[0],
                                                  self.value_edited)
        self.reload_tree_store_data()

    def register_actions(self, shortcut_manager):
        shortcut_manager.add_callback_for_action("delete", self.remove_action_callback)
        shortcut_manager.add_callback_for_action("add", self.add_action_callback)
        shortcut_manager.add_callback_for_action("add_hierarchy_state", partial(self.add_action_callback, True))
        # TODO integrate into clipboard with cut, copy and paste

    @ModelMT.observe("state", after=True)
    def model_changed(self, model, prop_name, info):
        """ This functions listens to all changes regarding the semantic_data field of the state and updates the
        tree store.

        :param model: the state model
        :param prop_name: the changed property
        :param info: all infos regarding the observable
        :return:
        """
        if "semantic_data" in info["method_name"]:
            self.reload_tree_store_data()

    def get_selected_object(self):
        """ Gets the selected object in the treeview

        :return:
        """
        model, paths = self.tree_view.get_selection().get_selected_rows()
        if len(paths) == 1:
            return self.tree_store.get_iter(paths[0]), paths[0]
        else:
            return None, paths

    def on_add(self, widget, new_dict=False):
        """" Adds a new entry to the semantic data of a state. Reloads the tree store.

        :param widget: The source widget of the action
        :param bool new_dict: A flag to indicate if the new value is of type dict
        :return:
        """
        self.semantic_data_counter += 1
        treeiter, path = self.get_selected_object()

        value = dict() if new_dict else "New Value"

        # get target dict path
        if treeiter:
            target_dict_path_as_list = self.tree_store[path][self.ID_STORAGE_ID]
            if not self.tree_store[path][self.IS_DICT_STORAGE_ID]:
                target_dict_path_as_list.pop()
        else:
            target_dict_path_as_list = []

        # generate key
        target_dict = self.model.state.get_semantic_data(target_dict_path_as_list)
        new_key_string = generate_semantic_data_key(target_dict.keys())
        self.model.state.add_semantic_data(target_dict_path_as_list, value, new_key_string)

        self.reload_tree_store_data()

        # jump to new element
        self.select_entry(target_dict_path_as_list + [new_key_string])
        logger.debug("Added new semantic data entry!")
        return True

    def add_action_callback(self, key_value, modifier_mask, a_dict=False):
        """Callback method for add action"""
        if react_to_event(self.view, self.tree_view, event=(key_value, modifier_mask)) and self.active_entry_widget is None:
            self.on_add(None, a_dict)
            return True

    def on_right_click_menu(self):
        # TODO add right click menu for more control e.g. insert of value element on root level if there are only dict values
        pass

    def on_remove(self, widget, data=None):
        """ Removes an entry of semantic data of a state.

        :param widget:
        :return:
        """
        treeiter, path = self.get_selected_object()
        if not treeiter:
            return

        # check if an element is selected
        dict_path_as_list = self.tree_store[path][self.ID_STORAGE_ID]
        logger.debug("Deleting semantic data entry with name {}!".format(dict_path_as_list[-1]))
        self.model.state.remove_semantic_data(dict_path_as_list)
        self.reload_tree_store_data()

        # hold cursor position where the last element was removed
        try:
            self.select_entry(self.tree_store[path][self.ID_STORAGE_ID])
        except IndexError:
            if len(self.tree_store):
                if len(path) > 1:
                    possible_before_path = tuple(list(path[:-1]) + [path[-1] - 1])
                    if possible_before_path[-1] > -1:
                        self.select_entry(self.tree_store[possible_before_path][self.ID_STORAGE_ID])
                    else:
                        self.select_entry(self.tree_store[path[:-1]][self.ID_STORAGE_ID])
                else:
                    self.select_entry(self.tree_store[path[0] - 1][self.ID_STORAGE_ID])
        return True

    def add_items_to_tree_iter(self, input_dict, treeiter, parent_dict_path=None):
        """ Adds all values of the input dict to self.tree_store

        :param input_dict: The input dictionary holds all values, which are going to be added.
        :param treeiter: The pointer inside the tree store to add the input dict
        :return:
        """
        if parent_dict_path is None:
            parent_dict_path = []
        self.get_view_selection()
        for key, value in sorted(input_dict.items()):
            element_dict_path = copy.copy(parent_dict_path) + [key]
            if isinstance(value, dict):
                new_iter = self.tree_store.append(treeiter, [key, "", True, element_dict_path])
                self.add_items_to_tree_iter(value, new_iter, element_dict_path)
            else:
                self.tree_store.append(treeiter, [key, value, False, element_dict_path])

    def reload_tree_store_data(self):
        """ Reloads the data of the tree store

        :return:
        """
        model, paths = self.tree_view.get_selection().get_selected_rows()

        self.tree_store.clear()
        self.add_items_to_tree_iter(self.model.state.semantic_data, None)
        self.tree_view.expand_all()

        try:
            for path in paths:
                self.tree_view.get_selection().select_path(path)
        except ValueError:
            pass

    @staticmethod
    def create_tree_store_path_from_key_string(path):
        """ Creates a tree store path from a path string

        :param str path: The input path string
        :rtype: tuple
        :return:
        """
        return tuple([int(path_elem_str) for path_elem_str in path.split(":")])

    def key_edited(self, path, new_key_str):
        """ Edits the key of a semantic data entry

        :param path: The path inside the tree store to the target entry
        :param str new_key_str: The new value of the target cell
        :return:
        """
        tree_store_path = self.create_tree_store_path_from_key_string(path) if isinstance(path, str) else path
        if self.tree_store[tree_store_path][self.KEY_STORAGE_ID] == new_key_str:
            return

        dict_path = self.tree_store[tree_store_path][self.ID_STORAGE_ID]
        old_value = self.model.state.get_semantic_data(dict_path)
        self.model.state.remove_semantic_data(dict_path)

        if new_key_str == "":
            target_dict = self.model.state.semantic_data
            for element in dict_path[0:-1]:
                target_dict = target_dict[element]
            new_key_str = generate_semantic_data_key(target_dict.keys())

        new_dict_path = self.model.state.add_semantic_data(dict_path[0:-1], old_value, key=new_key_str)
        self._changed_id_to = {':'.join(dict_path): new_dict_path}  # use hashable key (workaround for tree view ctrl)
        self.reload_tree_store_data()

    def value_edited(self, path, new_value_str):
        """ Adds the value of the semantic data entry

        :param path: The path inside the tree store to the target entry
        :param str new_value_str: The new value of the target cell
        :return:
        """
        tree_store_path = self.create_tree_store_path_from_key_string(path) if isinstance(path, str) else path
        if self.tree_store[tree_store_path][self.VALUE_STORAGE_ID] == new_value_str:
            return

        dict_path = self.tree_store[tree_store_path][self.ID_STORAGE_ID]
        self.model.state.add_semantic_data(dict_path[0:-1], new_value_str, key=dict_path[-1])
        self.reload_tree_store_data()

    def get_state_machine_selection(self):
        return None, []

    def update_selection_self_prior(self):
        """Tree view prior update of state machine selection"""
        pass

    def update_selection_sm_prior(self):
        """State machine prior update of tree selection"""
        pass

    def get_path_for_core_element(self, core_element_id):
        """Get path to the row representing core element described by handed core_element_id

        :param list core_element_id: Core element identifier used in the respective list store column
        :rtype: tuple
        :return: path
        """
        def check_function(row_iter, iter_found):
            row_id = self.tree_store.get_value(row_iter, self.ID_STORAGE_ID)
            if len(row_id) == len(core_element_id):
                if row_id == core_element_id:
                    iter_found.append(self.tree_store.get_path(row_iter))

        found_paths = []
        self.iter_tree_with_handed_function(check_function, found_paths)
        return found_paths[0] if found_paths else None
