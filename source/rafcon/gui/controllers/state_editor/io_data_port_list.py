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
.. module:: io_data_port_list
   :synopsis: A module that holds the controller to list and edit all input- and output-data-ports of a state.

"""

from gi.repository import Gtk
from gi.repository import GObject
from builtins import str

from rafcon.core.state_elements.data_port import InputDataPort, OutputDataPort
from rafcon.core.states.library_state import LibraryState

from rafcon.gui.controllers.utils.tree_view_controller import ListViewController, react_to_event
from rafcon.gui.models.abstract_state import AbstractStateModel
from rafcon.gui.clipboard import global_clipboard
from rafcon.gui.views.state_editor.input_port_list import InputPortsListView
from rafcon.gui.views.state_editor.output_port_list import OutputPortsListView
import rafcon.gui.helpers.state_machine as gui_helper_state_machine

from rafcon.gui.utils.comparison import compare_variables
from rafcon.utils import log

logger = log.get_logger(__name__)


class DataPortListController(ListViewController):
    """Controller handling the input and output Data Port List
    """
    NAME_STORAGE_ID = 0
    DATA_TYPE_NAME_STORAGE_ID = 1
    DEFAULT_VALUE_STORAGE_ID = 2
    ID_STORAGE_ID = 3
    USE_RUNTIME_VALUE_STORAGE_ID = 4
    RUNTIME_VALUE_STORAGE_ID = 5
    MODEL_STORAGE_ID = 6

    state_data_port_dict = None
    data_port_model_list = None

    def __init__(self):
        raise NotImplementedError("You have to instantiate a subclass of DataPortListController")

    def destroy(self):
        if self.model.state.get_next_upper_library_root_state() is None and \
                (isinstance(self.view, InputPortsListView) or isinstance(self.view, OutputPortsListView)):
            self.view['default_value_col'].set_cell_data_func(self.view['default_value_text'], None)
        super(DataPortListController, self).destroy()

    def register_view(self, view):
        """Called when the View was registered"""
        super(DataPortListController, self).register_view(view)

        view['name_col'].add_attribute(view['name_text'], 'text', self.NAME_STORAGE_ID)
        view['data_type_col'].add_attribute(view['data_type_text'], 'text', self.DATA_TYPE_NAME_STORAGE_ID)
        if not isinstance(self.model.state, LibraryState) and self.model.state.get_next_upper_library_root_state() is None:
            view['name_text'].set_property("editable", True)
            view['data_type_text'].set_property("editable", True)

        # in the linkage overview the the default value is not shown
        if isinstance(view, InputPortsListView) or isinstance(view, OutputPortsListView):
            view['default_value_col'].add_attribute(view['default_value_text'], 'text', self.DEFAULT_VALUE_STORAGE_ID)
            self._apply_value_on_edited_and_focus_out(view['default_value_text'], self._apply_new_data_port_default_value)
            view['default_value_col'].set_cell_data_func(view['default_value_text'],
                                                         self._default_value_cell_data_func)
            if isinstance(self.model.state, LibraryState):
                view['default_value_col'].set_title("Used value")
            if self.model.state.get_next_upper_library_root_state() is None:  # never enabled means it is disabled
                view['default_value_text'].set_property("editable", True)

        self._apply_value_on_edited_and_focus_out(view['name_text'], self._apply_new_data_port_name)
        self._apply_value_on_edited_and_focus_out(view['data_type_text'], self._apply_new_data_port_type)

        if isinstance(self.model.state, LibraryState):
            view['use_runtime_value_toggle'] = Gtk.CellRendererToggle()
            view['use_runtime_value_col'] = Gtk.TreeViewColumn("Use Runtime Value")
            view.make_column_title_elllipsable(view['use_runtime_value_col'],
                                               tooltip="If set, the default value to the left will be used as default "\
                                                       "value. Otherwise, the original default value of the library")
            view['use_runtime_value_col'].set_property("sizing", Gtk.TreeViewColumnSizing.AUTOSIZE)
            view.get_top_widget().append_column(view['use_runtime_value_col'])
            view['use_runtime_value_col'].pack_start(view['use_runtime_value_toggle'], True)
            view['use_runtime_value_col'].add_attribute(view['use_runtime_value_toggle'], 'active',
                                                        self.USE_RUNTIME_VALUE_STORAGE_ID)
            if self.model.state.get_next_upper_library_root_state() is None:
                view['use_runtime_value_toggle'].set_property("activatable", True)
                view['use_runtime_value_toggle'].connect("toggled", self.on_use_runtime_value_toggled)

        self._reload_data_port_list_store()

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.gui.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
            between shortcuts and actions.
        """
        shortcut_manager.add_callback_for_action("copy", self.copy_action_callback)
        shortcut_manager.add_callback_for_action("delete", self.remove_action_callback)
        shortcut_manager.add_callback_for_action("add", self.add_action_callback)
        shortcut_manager.add_callback_for_action("cut", self.cut_action_callback)
        shortcut_manager.add_callback_for_action("paste", self.paste_action_callback)

    def paste_action_callback(self, *event, **kwargs):
        raise NotImplementedError()

    def on_add(self, widget, data=None):
        """callback method for the 'Add' data port button
        """
        self.add_new_data_port()

    def on_use_runtime_value_toggled(self, widget, path):
        """Try to set the use runtime value flag to the newly entered one
        """
        try:
            data_port_id = self.list_store[path][self.ID_STORAGE_ID]
            self.toggle_runtime_value_usage(data_port_id)
        except TypeError as e:
            logger.exception("Error while trying to change the use_runtime_value flag")

    def on_right_click_menu(self):
        logger.debug("Not implemented, yet")

    def add_new_data_port(self):
        """Adds a new port with default values and selects it
        """
        raise NotImplementedError()

    def remove_core_element(self, model):
        """Remove respective core element of handed data port model

        :param DataPortModel model: Data port model which core element should be removed
        """
        raise NotImplementedError()

    def toggle_runtime_value_usage(self, data_port_id):
        """Toggles the data port flag determining which default value is to be used
        
        :param int data_port_id: ID of the data port 
        """
        raise NotImplementedError()

    def set_data_port_runtime_value(self, data_port_id, value):
        """Sets a new value for the data port runtime value
        
        Value is only set, if it differs from the current one.
        
        :param int data_port_id: ID of the data port
        :param str value: New runtime value
        """
        raise NotImplementedError()

    def get_data_port_runtime_configuration(self, data_port_id):
        """Return information about the runtime configuration of a data port
        
        :param int data_port_id: ID of the data port 
        :return: use_runtime_value, runtime_value
        :rtype tuple(bool, any)
        """
        raise NotImplementedError()

    @staticmethod
    def _get_new_list_store():
        return Gtk.ListStore(GObject.TYPE_STRING, GObject.TYPE_STRING, GObject.TYPE_STRING, int, bool, GObject.TYPE_STRING, GObject.TYPE_PYOBJECT)

    def _default_value_cell_data_func(self, tree_view_column, cell, model, iter, data=None):
        """Function set renderer properties for every single cell independently

        The function controls the editable and color scheme for every cell in the default value column according
        the use_runtime_value flag and whether the state is a library state.

        :param tree_view_column: the Gtk.TreeViewColumn to be rendered
        :param cell: the current CellRenderer
        :param model: the Gtk.ListStore or TreeStore that is the model for TreeView
        :param iter: an iterator over the rows of the TreeStore/Gtk.ListStore Model
        :param data: optional data to be passed: see http://dumbmatter.com/2012/02/some-notes-on-porting-from-pygtk-to-pygobject/
        """
        if isinstance(self.model.state, LibraryState):
            use_runtime_value = model.get_value(iter, self.USE_RUNTIME_VALUE_STORAGE_ID)
            if use_runtime_value:
                cell.set_property("editable", True)
                cell.set_property('text', model.get_value(iter, self.RUNTIME_VALUE_STORAGE_ID))
                cell.set_property('foreground', "white")
            else:
                cell.set_property("editable", False)
                cell.set_property('text', model.get_value(iter, self.DEFAULT_VALUE_STORAGE_ID))
                cell.set_property('foreground', "dark grey")

        return

    def _reload_data_port_list_store(self):
        """Reloads the input data port list store from the data port models"""

        tmp = self._get_new_list_store()
        for data_port_m in self.data_port_model_list:
            data_port_id = data_port_m.data_port.data_port_id
            data_type = data_port_m.data_port.data_type
            # get name of type (e.g. ndarray)
            data_type_name = data_type.__name__
            # get module of type, e.g. numpy
            data_type_module = data_type.__module__
            # if the type is not a builtin type, also show the module
            if data_type_module not in ['__builtin__', 'builtins']:
                data_type_name = data_type_module + '.' + data_type_name
            if data_port_m.data_port.default_value is None:
                default_value = "None"
            else:
                default_value = data_port_m.data_port.default_value

            if not isinstance(self.model.state, LibraryState):
                tmp.append([data_port_m.data_port.name, data_type_name, str(default_value), data_port_id,
                            None, None, data_port_m])
            else:
                use_runtime_value, runtime_value = self.get_data_port_runtime_configuration(data_port_id)
                tmp.append([data_port_m.data_port.name,
                            data_type_name,
                            str(default_value),
                            data_port_id,
                            bool(use_runtime_value),
                            str(runtime_value),
                            data_port_m,
                            ])

        tms = Gtk.TreeModelSort(model=tmp)
        tms.set_sort_column_id(0, Gtk.SortType.ASCENDING)
        tms.set_sort_func(0, compare_variables)
        tms.sort_column_changed()
        tmp = tms
        self.list_store.clear()
        for elem in tmp:
            self.list_store.append(elem[:])

    def _apply_new_data_port_name(self, path, new_name):
        """Applies the new name of the data port defined by path

        :param str path: The path identifying the edited data port
        :param str new_name: New name
        """
        try:
            data_port_id = self.list_store[path][self.ID_STORAGE_ID]
            if self.state_data_port_dict[data_port_id].name != new_name:
                self.state_data_port_dict[data_port_id].name = new_name
        except (TypeError, ValueError) as e:
            logger.exception("Error while trying to change data port name")

    def _apply_new_data_port_type(self, path, new_data_type_str):
        """Applies the new data type of the data port defined by path

        :param str path: The path identifying the edited data port
        :param str new_data_type_str: New data type as str
        """
        try:
            data_port_id = self.list_store[path][self.ID_STORAGE_ID]
            if self.state_data_port_dict[data_port_id].data_type.__name__ != new_data_type_str:
                self.state_data_port_dict[data_port_id].change_data_type(new_data_type_str)
        except ValueError as e:
            logger.exception("Error while changing data type")

    def _apply_new_data_port_default_value(self, path, new_default_value_str):
        """Applies the new default value of the data port defined by path

        :param str path: The path identifying the edited variable
        :param str new_default_value_str: New default value as string
        """
        try:
            data_port_id = self.list_store[path][self.ID_STORAGE_ID]
            if isinstance(self.model.state, LibraryState):
                # this always has to be true, as the runtime value column can only be edited
                # if the use_runtime_value flag is True
                if self.list_store[path][self.USE_RUNTIME_VALUE_STORAGE_ID]:
                    self.set_data_port_runtime_value(data_port_id, new_default_value_str)
            else:
                if str(self.state_data_port_dict[data_port_id].default_value) != new_default_value_str:
                    self.state_data_port_dict[data_port_id].default_value = new_default_value_str
        except (TypeError, AttributeError) as e:
            logger.exception("Error while changing default value")

    def _data_ports_changed(self, model):
        """Reload list store and reminds selection when the model was changed"""
        if not isinstance(model, AbstractStateModel):
            return
        # store port selection
        path_list = None
        if self.view is not None:
            model, path_list = self.tree_view.get_selection().get_selected_rows()
        selected_data_port_ids = [self.list_store[path[0]][self.ID_STORAGE_ID] for path in path_list] if path_list else []
        self._reload_data_port_list_store()
        # recover port selection
        if selected_data_port_ids:
            [self.select_entry(selected_data_port_id, False) for selected_data_port_id in selected_data_port_ids]


class InputPortListController(DataPortListController):

    CORE_ELEMENT_CLASS = InputDataPort

    def __init__(self, model, view):
        super(DataPortListController, self).__init__(model, view, view.get_top_widget(), self._get_new_list_store(),
                                                     logger)
        self.state_data_port_dict = self.model.state.input_data_ports
        self.data_port_model_list = self.model.input_data_ports

        if self.model.get_state_machine_m() is not None:
            self.observe_model(self.model.get_state_machine_m())
        else:
            logger.warning("State model '{0}' has no state machine model".format(self.model))

    @ListViewController.observe("input_data_ports", after=True)
    def input_data_ports_changed(self, model, prop_name, info):
        self._data_ports_changed(model)

    @ListViewController.observe("state", after=True)
    def runtime_values_changed(self, model, prop_name, info):
        """Handle cases for the library runtime values"""
        if ("_input_runtime_value" in info.method_name or
                info.method_name in ['use_runtime_value_input_data_ports',
                                     'input_data_port_runtime_values']) and \
                self.model is model:
            self._data_ports_changed(model)

    def paste_action_callback(self, *event, **kwargs):
        """Callback method for paste action

        The method triggers the paste method of the clipboard paste. If there are InputDataPorts in the clipboard, 
        those are pasted. If there are no InputDataPorts, it is tried to paste converted OutputDataPorts and 
        ScopedVariables.
        """
        if react_to_event(self.view, self.tree_view, event) and self.active_entry_widget is None:
            if not global_clipboard.model_copies["input_data_ports"] and \
                    (global_clipboard.model_copies["output_data_ports"] or
                     global_clipboard.model_copies["scoped_variables"]):
                global_clipboard.paste(self.model, limited=['input_data_ports'], convert=True)
            else:
                global_clipboard.paste(self.model, limited=['input_data_ports'])
            return True

    def add_new_data_port(self):
        """Add a new port with default values and select it"""
        try:
            new_data_port_ids = gui_helper_state_machine.add_data_port_to_selected_states('INPUT', int, [self.model])
            if new_data_port_ids:
                self.select_entry(new_data_port_ids[self.model.state])
        except ValueError:
            pass

    def remove_core_element(self, model):
        assert model.data_port.parent is self.model.state
        gui_helper_state_machine.delete_core_element_of_model(model)

    def toggle_runtime_value_usage(self, data_port_id):
        current_flag = self.model.state.use_runtime_value_input_data_ports[data_port_id]
        self.model.state.set_use_input_runtime_value(data_port_id, not current_flag)

    def set_data_port_runtime_value(self, data_port_id, value):
        if str(self.model.state.input_data_port_runtime_values[data_port_id]) != value:
            self.model.state.set_input_runtime_value(data_port_id, value)

    def get_data_port_runtime_configuration(self, data_port_id):
        use_runtime_value = self.model.state.use_runtime_value_input_data_ports[data_port_id]
        runtime_value = self.model.state.input_data_port_runtime_values[data_port_id]
        return use_runtime_value, runtime_value


class OutputPortListController(DataPortListController):

    CORE_ELEMENT_CLASS = OutputDataPort

    def __init__(self, model, view):
        super(DataPortListController, self).__init__(model, view, view.get_top_widget(), self._get_new_list_store(),
                                                     logger)
        self.state_data_port_dict = self.model.state.output_data_ports
        self.data_port_model_list = self.model.output_data_ports

        if self.model.get_state_machine_m() is not None:
            self.observe_model(self.model.get_state_machine_m())
        else:
            logger.warning("State model '{0}' has no state machine model".format(self.model))

    @ListViewController.observe("output_data_ports", after=True)
    def output_data_ports_changed(self, model, prop_name, info):
        self._data_ports_changed(model)

    @ListViewController.observe("state", after=True)
    def runtime_values_changed(self, model, prop_name, info):
        """Handle cases for the library runtime values"""
        if ("_output_runtime_value" in info.method_name or
                info.method_name in ['use_runtime_value_output_data_ports',
                                     'output_data_port_runtime_values']) and \
                self.model is model:
            self._data_ports_changed(model)

    def paste_action_callback(self, *event, **kwargs):
        """Callback method for paste action

        The method triggers the paste method of the clipboard paste. If there are OutputDataPorts in the clipboard, 
        those are pasted. If there are no OutputDataPorts, it is tried to paste converted InputDataPorts and 
        ScopedVariables.
        """
        if react_to_event(self.view, self.tree_view, event) and self.active_entry_widget is None:
            if not global_clipboard.model_copies["output_data_ports"] and \
                    (global_clipboard.model_copies["input_data_ports"] or
                     global_clipboard.model_copies["scoped_variables"]):
                global_clipboard.paste(self.model, limited=['output_data_ports'], convert=True)
            else:
                global_clipboard.paste(self.model, limited=['output_data_ports'])
            return True

    def add_new_data_port(self):
        """Add a new port with default values and select it"""
        try:
            new_data_port_ids = gui_helper_state_machine.add_data_port_to_selected_states('OUTPUT', int, [self.model])
            if new_data_port_ids:
                self.select_entry(new_data_port_ids[self.model.state])
        except ValueError:
            pass

    def remove_core_element(self, model):
        assert model.data_port.parent is self.model.state
        gui_helper_state_machine.delete_core_element_of_model(model)

    def toggle_runtime_value_usage(self, data_port_id):
        current_flag = self.model.state.use_runtime_value_output_data_ports[data_port_id]
        self.model.state.set_use_output_runtime_value(data_port_id, not current_flag)

    def set_data_port_runtime_value(self, data_port_id, value):
        if str(self.model.state.output_data_port_runtime_values[data_port_id]) != value:
            self.model.state.set_output_runtime_value(data_port_id, value)

    def get_data_port_runtime_configuration(self, data_port_id):
        use_runtime_value = self.model.state.use_runtime_value_output_data_ports[data_port_id]
        runtime_value = self.model.state.output_data_port_runtime_values[data_port_id]
        return use_runtime_value, runtime_value
