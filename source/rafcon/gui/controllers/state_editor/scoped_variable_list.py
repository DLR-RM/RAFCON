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
.. module:: scoped_variable_list
   :synopsis: A module that holds the controller to list and edit all scoped_variables of a state.

"""

from gi.repository import Gtk
from gi.repository import GObject
from builtins import str

from rafcon.core.states.library_state import LibraryState
from rafcon.core.state_elements.scope import ScopedVariable

from rafcon.gui.controllers.utils.tree_view_controller import ListViewController, react_to_event
from rafcon.gui.views.state_editor.scoped_variables_list import ScopedVariablesListView
from rafcon.gui.models.container_state import ContainerStateModel
from rafcon.gui.clipboard import global_clipboard
import rafcon.gui.helpers.state_machine as gui_helper_state_machine

from rafcon.gui.utils.comparison import compare_variables
from rafcon.utils import log

logger = log.get_logger(__name__)


class ScopedVariableListController(ListViewController):
    """Controller handling the scoped variable list

    :param rafcon.gui.models.state.StateModel model: The state model, holding state data.
    :param rafcon.gui.views.scoped_variables_list.ScopedVariablesListView view: The GTK view showing the list of scoped
        variables.
    """
    NAME_STORAGE_ID = 0
    DATA_TYPE_NAME_STORAGE_ID = 1
    DEFAULT_VALUE_STORAGE_ID = 2
    ID_STORAGE_ID = 3
    MODEL_STORAGE_ID = 4
    CORE_ELEMENT_CLASS = ScopedVariable

    def __init__(self, model, view):
        """Constructor"""
        super(ScopedVariableListController, self).__init__(model, view, view.get_top_widget(),
                                                           self.get_new_list_store(), logger)

        self.next_focus_column = {}
        self.prev_focus_column = {}

        if self.model.get_state_machine_m() is not None:
            self.observe_model(self.model.get_state_machine_m())
        else:
            logger.warning("State model has no state machine model -> state model: {0}".format(self.model))

    @staticmethod
    def get_new_list_store():
        return Gtk.ListStore(GObject.TYPE_STRING, GObject.TYPE_STRING, GObject.TYPE_STRING, int, GObject.TYPE_PYOBJECT)

    def register_view(self, view):
        """Called when the View was registered"""
        super(ScopedVariableListController, self).register_view(view)

        view['name_col'].add_attribute(view['name_text'], 'text', self.NAME_STORAGE_ID)
        if not isinstance(self.model.state, LibraryState) and self.model.state.get_next_upper_library_root_state() is None:
            view['name_text'].set_property("editable", True)
        view['data_type_col'].add_attribute(view['data_type_text'], 'text', self.DATA_TYPE_NAME_STORAGE_ID)
        if not isinstance(self.model.state, LibraryState) and self.model.state.get_next_upper_library_root_state() is None:
            view['data_type_text'].set_property("editable", True)
        if isinstance(view, ScopedVariablesListView):
            view['default_value_col'].add_attribute(view['default_value_text'], 'text', self.DEFAULT_VALUE_STORAGE_ID)
            if not isinstance(self.model.state, LibraryState) and self.model.state.get_next_upper_library_root_state() is None:
                view['default_value_text'].set_property("editable", True)
            self._apply_value_on_edited_and_focus_out(view['default_value_text'],
                                                      self.apply_new_scoped_variable_default_value)

        self._apply_value_on_edited_and_focus_out(view['name_text'], self.apply_new_scoped_variable_name)
        self._apply_value_on_edited_and_focus_out(view['data_type_text'], self.apply_new_scoped_variable_type)

        if isinstance(self.model, ContainerStateModel):
            self.reload_scoped_variables_list_store()

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.gui.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
            between shortcuts and actions.
        """
        shortcut_manager.add_callback_for_action("delete", self.remove_action_callback)
        shortcut_manager.add_callback_for_action("add", self.add_action_callback)
        shortcut_manager.add_callback_for_action("copy", self.copy_action_callback)
        shortcut_manager.add_callback_for_action("cut", self.cut_action_callback)
        shortcut_manager.add_callback_for_action("paste", self.paste_action_callback)

    def paste_action_callback(self, *event, **kwargs):
        """Callback method for paste action

         The method trigger the clipboard paste of the list of scoped variables in the clipboard or in case this list is
         empty and there are other port types selected in the clipboard it will trigger the paste with convert flag.
         The convert flag will cause the insertion of scoped variables with the same names, data types and default values
         the objects of differing port type (in the clipboard) have.
        """
        if react_to_event(self.view, self.tree_view, event) and self.active_entry_widget is None:
            if not global_clipboard.model_copies["scoped_variables"] and \
                    (global_clipboard.model_copies["input_data_ports"] or
                     global_clipboard.model_copies["output_data_ports"]):
                global_clipboard.paste(self.model, limited=['scoped_variables'], convert=True)
            else:
                global_clipboard.paste(self.model, limited=['scoped_variables'])
            return True

    @ListViewController.observe("scoped_variables", after=True)
    def scoped_variables_changed(self, model, prop_name, info):
        # store port selection
        path_list = None
        if self.view is not None:
            model, path_list = self.view.get_top_widget().get_selection().get_selected_rows()
        selected_data_port_ids = [self.list_store[path[0]][self.ID_STORAGE_ID] for path in path_list] if path_list else []
        self.reload_scoped_variables_list_store()
        # recover port selection
        if selected_data_port_ids:
            [self.select_entry(selected_data_port_id, False) for selected_data_port_id in selected_data_port_ids]

    def on_add(self, widget, data=None):
        """Create a new scoped variable with default values"""
        if isinstance(self.model, ContainerStateModel):
            try:
                scoped_var_ids = gui_helper_state_machine.add_scoped_variable_to_selected_states(selected_states=[self.model])
                if scoped_var_ids:
                    self.select_entry(scoped_var_ids[self.model.state])
            except ValueError as e:
                logger.warning("The scoped variable couldn't be added: {0}".format(e))
                return False

            return True

    def remove_core_element(self, model):
        """Remove respective core element of handed scoped variable model

        :param ScopedVariableModel model: Scoped variable model which core element should be removed
        :return:
        """
        assert model.scoped_variable.parent is self.model.state
        gui_helper_state_machine.delete_core_element_of_model(model)

    def apply_new_scoped_variable_name(self, path, new_name):
        """Applies the new name of the scoped variable defined by path

        :param str path: The path identifying the edited variable
        :param str new_name: New name
        """
        data_port_id = self.list_store[path][self.ID_STORAGE_ID]
        try:
            if self.model.state.scoped_variables[data_port_id].name != new_name:
                self.model.state.scoped_variables[data_port_id].name = new_name
        except TypeError as e:
            logger.error("Error while changing port name: {0}".format(e))

    def apply_new_scoped_variable_type(self, path, new_variable_type_str):
        """Applies the new data type of the scoped variable defined by path

        :param str path: The path identifying the edited variable
        :param str new_variable_type_str: New data type as str
        """
        data_port_id = self.list_store[path][self.ID_STORAGE_ID]
        try:
            if self.model.state.scoped_variables[data_port_id].data_type.__name__ != new_variable_type_str:
                self.model.state.scoped_variables[data_port_id].change_data_type(new_variable_type_str)
        except ValueError as e:
            logger.error("Error while changing data type: {0}".format(e))

    def apply_new_scoped_variable_default_value(self, path, new_default_value_str):
        """Applies the new default value of the scoped variable defined by path

        :param str path: The path identifying the edited variable
        :param str new_default_value_str: New default value as string
        """
        data_port_id = self.get_list_store_row_from_cursor_selection()[self.ID_STORAGE_ID]
        try:
            if str(self.model.state.scoped_variables[data_port_id].default_value) != new_default_value_str:
                self.model.state.scoped_variables[data_port_id].default_value = new_default_value_str
        except (TypeError, AttributeError) as e:
            logger.error("Error while changing default value: {0}".format(e))

    def on_right_click_menu(self):
        pass

    def reload_scoped_variables_list_store(self):
        """Reloads the scoped variable list store from the data port models"""

        if isinstance(self.model, ContainerStateModel):
            tmp = self.get_new_list_store()
            for sv_model in self.model.scoped_variables:
                data_type = sv_model.scoped_variable.data_type
                # get name of type (e.g. ndarray)
                data_type_name = data_type.__name__
                # get module of type, e.g. numpy
                data_type_module = data_type.__module__
                # if the type is not a builtin type, also show the module
                if data_type_module not in ['__builtin__', 'builtins']:
                    data_type_name = data_type_module + '.' + data_type_name
                tmp.append([sv_model.scoped_variable.name, data_type_name,
                            str(sv_model.scoped_variable.default_value), sv_model.scoped_variable.data_port_id,
                                sv_model])
            tms = Gtk.TreeModelSort(model=tmp)
            tms.set_sort_column_id(0, Gtk.SortType.ASCENDING)
            tms.set_sort_func(0, compare_variables)
            tms.sort_column_changed()
            tmp = tms
            self.list_store.clear()
            for elem in tmp:
                self.list_store.append(elem[:])
        else:
            raise RuntimeError("The reload_scoped_variables_list_store function should be never called for "
                               "a non Container State Model")

