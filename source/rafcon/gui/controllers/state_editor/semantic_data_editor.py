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
import copy
from gtkmvc import ModelMT

from rafcon.core.id_generator import generate_semantic_data_key

from rafcon.gui.controllers.utils.tree_view_controller import TreeViewController
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.gui.models import AbstractStateModel
from rafcon.gui.views.state_editor.semantic_data_editor import SemanticDataEditorView

from rafcon.utils import log
from rafcon.utils.vividict import Vividict
from rafcon.utils import type_helpers

logger = log.get_logger(__name__)


class SemanticDataEditorController(ExtendedController):

    """ A controller class to visualize and edit the semantic data of a state

    """

    def __init__(self, model, view):
        """Constructor
        """
        assert isinstance(model, AbstractStateModel)
        assert isinstance(view, SemanticDataEditorView)
        ExtendedController.__init__(self, model, view)
        self.tree_view = view["semantic_data_tree_view"]

        self.tree_store = None
        self.set_tree_store()
        self.reload_tree_store()
        self.semantic_data_counter = 0

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
            self.reload_tree_store()

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application

        :param rafcon.gui.views.state_editor.semantic_data_editor.SemanticDataEditorView view: An view to show all
                semantic data of a state
        """
        view['new_entry'].connect('clicked', self.on_add, False)
        view['new_dict_entry'].connect('clicked', self.on_add, True)
        view['delete_entry'].connect('clicked', self.on_remove)

    def set_tree_store(self):
        """ Creates the tree store for the treeview which stores the entrys of the semantic data of a state

        :return:
        """
        key_renderer = gtk.CellRendererText()
        key_renderer.set_property('editable', True)
        col = gtk.TreeViewColumn('Key', key_renderer, text=0)
        self.tree_view.append_column(col)

        value_renderer = gtk.CellRendererText()
        value_renderer.set_property('editable', True)
        col = gtk.TreeViewColumn('Value', value_renderer, text=1)
        self.tree_view.append_column(col)

        is_dict_renderer = gtk.CellRendererText()
        col = gtk.TreeViewColumn('Is Dict', is_dict_renderer, text=2)
        self.tree_view.append_column(col)

        key_renderer.connect('edited', self.key_edited)
        value_renderer.connect('edited', self.value_edited)

    def get_selected_object(self):
        """ Gets the selected object in the treeview

        :return:
        """
        model, paths = self.tree_view.get_selection().get_selected_rows()
        if len(paths) == 1:
            return self.tree_store.get_iter(paths[0]), paths[0]
        else:
            return None, paths

    def on_add(self, widget, new_dict):
        """" Adds a new entry to the semantic data of a state. Reloads the tree store.

        :param widget: The source widget of the action
        :param new_dict: A flag to indicate if the new value is of type dict
        :return:
        """
        self.semantic_data_counter += 1
        treeiter, path = self.get_selected_object()
        value = dict() if new_dict else "New Value"
        if treeiter:
            selection_is_dict = self.tree_store.get_value(treeiter, 2)
            if not selection_is_dict:
                path = path[0:-1]
            dict_path_as_list = self.get_dict_path_from_tree_path_as_list(path)
            # generate key
            target_dict = self.model.state.semantic_data
            for element in dict_path_as_list:
                target_dict = target_dict[element]
            new_key_string = generate_semantic_data_key(target_dict.keys())
            self.model.state.add_semantic_data(dict_path_as_list, value, new_key_string)
            self.reload_tree_store()
        else:
            new_key_string = generate_semantic_data_key(self.model.state.semantic_data.keys())
            self.model.state.add_semantic_data("", value, new_key_string)
            self.reload_tree_store()
        # TODO: jump with selection to new element
        # self.tree_view.get_selection().select_iter(treeiter)
        # self.select_entry(meta_name)
        logger.debug("Added new semantic data entry!")
        return True

    def get_key_of_path(self, path):
        """ Gets the key entry of a tree store path

        :param path: A tree store path
        :return:
        """
        treeiter = self.tree_store.get_iter(path)
        return self.tree_store.get_value(treeiter, 0)

    def get_dict_path_from_tree_path_as_list(self, path):
        """ Gets a key list of a tree store path. This key list can be used to access a vividict.

        :param path: The tree store path
        :return:
        """
        tmp_path = path
        dict_path = list()
        while len(tmp_path) > 0:
            dict_path.insert(0, self.get_key_of_path(tmp_path))
            tmp_path = tmp_path[0:-1]
        return dict_path

    def on_remove(self, widget):
        """ Removes an entry of semantic data of a state.

        :param widget:
        :return:
        """
        treeiter, path = self.get_selected_object()
        # check if an element is selected
        if treeiter:
            dict_path_as_list = self.get_dict_path_from_tree_path_as_list(path)
            logger.debug("Deleting semantic data entry with name {}!".format(dict_path_as_list[-1]))
            self.model.state.remove_semantic_data(dict_path_as_list)
            self.reload_tree_store()
        return True

    def add_items_to_tree_iter(self, input_dict, treeiter):
        """ Adds all values of the input dict to self.tree_store

        :param input_dict: The input dictionary holds all values, which are going to be added.
        :param treeiter: The pointer inside the tree store to add the input dict
        :return:
        """
        for key, value in sorted(input_dict.items()):
            if isinstance(value, dict):
                new_iter = self.tree_store.append(treeiter, [key, "", True])
                self.add_items_to_tree_iter(value, new_iter)
            else:
                self.tree_store.append(treeiter, [key, value, False])

    def reload_tree_store(self):
        """ Reloads the data of the tree store

        :return:
        """
        self.tree_store = gtk.TreeStore(str, str, bool)
        self.tree_view.set_model(self.tree_store)
        self.add_items_to_tree_iter(self.model.state.semantic_data, None)
        self.tree_view.expand_all()

    def craete_tree_store_path_from_key_string(self, path):
        """ Creates a tree store path from a path string

        :param path: The input path string
        :return:
        """
        path_as_string_list = path.split(":")
        path_as_int_list = []
        for elem in path_as_string_list:
            path_as_int_list.append(int(elem))
        return tuple(path_as_int_list)

    def key_edited(self, renderer, path, new_key_string):
        """ Edits the key of a semantic data entry

        :param renderer: The renderer object of the edited cell
        :param path: The path inside the tree store to the target entry
        :param new_key_string: The new value of the target cell
        :return:
        """
        # treeiter = self.tree_store.get_iter(path)
        # self.tree_store.set_value(treeiter, 0, new_value_str)
        tree_store_path = self.craete_tree_store_path_from_key_string(path)
        dict_path = self.get_dict_path_from_tree_path_as_list(tree_store_path)
        old_value = self.model.state.get_semantic_data(dict_path)
        self.model.state.remove_semantic_data(dict_path)

        if new_key_string == "":
            target_dict = self.model.state.semantic_data
            for element in dict_path[0:-1]:
                target_dict = target_dict[element]
            new_key_string = generate_semantic_data_key(target_dict.keys())

        self.model.state.add_semantic_data(dict_path[0:-1], old_value, key=new_key_string)
        self.reload_tree_store()

    def value_edited(self, renderer, path, new_value_str):
        """ Adds the value of the semantic data entry

        :param renderer: The renderer object of the edited cell
        :param path: The path inside the tree store to the target entry
        :param new_value_str: The new value of the target cell
        :return:
        """
        tree_store_path = self.craete_tree_store_path_from_key_string(path)
        dict_path = self.get_dict_path_from_tree_path_as_list(tree_store_path)
        self.model.state.add_semantic_data(dict_path[0:-1], new_value_str, key=dict_path[-1])
        self.reload_tree_store()
