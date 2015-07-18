import gtk
from gtk import ListStore
import copy

from awesome_tool.utils import log
logger = log.get_logger(__name__)
from awesome_tool.statemachine.states.library_state import LibraryState

from awesome_tool.mvc.controllers.extended_controller import ExtendedController
from awesome_tool.mvc.models.state import StateModel


class DataPortListController(ExtendedController):

    def __init__(self, model, view, io_type):
        """Constructor
        """
        ExtendedController.__init__(self, model, view)
        self.type = io_type
        self.state_data_port_dict = None
        self.data_port_list_store = None

        self.new_port_counter = 0

        if self.type == "input":
            self.state_data_port_dict = self.model.state.input_data_ports
            self.data_port_model_list = self.model.input_data_ports
        elif self.type == "output":
            self.state_data_port_dict = self.model.state.output_data_ports
            self.data_port_model_list = self.model.output_data_ports

        self.data_port_list_store = ListStore(str, str, str, int)
        self.reload_data_port_list_store()

    def register_view(self, view):
        """Called when the View was registered
        """
        # top widget is a tree view => set the model of the tree view to be a list store
        view.get_top_widget().set_model(self.data_port_list_store)
        view.get_top_widget().set_cursor(0)

        view['name_col'].add_attribute(view['name_text'], 'text', 0)
        if not isinstance(self.model.state, LibraryState):
            view['name_text'].set_property("editable", True)
        view['data_type_col'].add_attribute(view['data_type_text'], 'text', 1)
        # if not isinstance(self.model.state, LibraryState):
        view['data_type_text'].set_property("editable", True)
        if view['default_value_col'] and view['default_value_text']:
            view['default_value_col'].add_attribute(view['default_value_text'], 'text', 2)
            view['default_value_text'].set_property("editable", True)
            view['default_value_text'].connect("edited", self.on_default_value_changed)

        view['name_text'].connect("edited", self.on_name_changed)
        view['data_type_text'].connect("edited", self.on_data_type_changed)

    def register_adapters(self):
        """Adapters should be registered in this method call
        """

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param awesome_tool.mvc.shortcut_manager.ShortcutManager shortcut_manager:
        """
        shortcut_manager.add_callback_for_action("delete", self.remove_port)
        shortcut_manager.add_callback_for_action("add", self.add_port)

    def add_port(self, *_):
        """Callback method for add action
        """
        if self.view[self.view.top].has_focus():
            self.on_new_port_button_clicked(None)

    def remove_port(self, *_):
        """Callback method for remove action
        """
        if self.view[self.view.top].has_focus():
            self.on_delete_port_button_clicked(None)

    @ExtendedController.observe("input_data_ports", after=True)
    def input_data_ports_changed(self, model, prop_name, info):
        """Reload list store when the model was changed
        """
        if self.type == "input":
            self.reload_data_port_list_store()

    @ExtendedController.observe("output_data_ports", after=True)
    def output_data_ports_changed(self, model, prop_name, info):
        """Reload list store when the model was changed
        """
        if self.type == "output":
            self.reload_data_port_list_store()

    def on_new_port_button_clicked(self, widget, data=None):
        """Add a new port with default values and select it
        """
        new_port_name = self.type + "_{0}".format(self.new_port_counter)
        self.new_port_counter += 1
        if self.type == "input":
            data_port_id = self.model.state.add_input_data_port(new_port_name, "int", "0")
        else:
            data_port_id = self.model.state.add_output_data_port(new_port_name, "int", "0")
        self.select_entry(data_port_id)

    def on_delete_port_button_clicked(self, widget, data=None):
        """Delete the selected port and select the next one
        """
        path = self.get_path()
        data_port_id = self.get_data_port_id_from_selection()
        if data_port_id is not None:
            if self.type == "input":
                self.model.state.remove_input_data_port(data_port_id)
            else:
                self.model.state.remove_output_data_port(data_port_id)
            if len(self.data_port_list_store) > 0:
                self.view[self.view.top].set_cursor(min(path, len(self.data_port_list_store)-1))

    def get_data_port_id_from_selection(self):
        """Returns the data_port_id of the currently selected port entry"""
        path = self.get_path()
        if path is not None:
            data_port_id = self.data_port_list_store[int(path)][3]
            return data_port_id
        return None

    def select_entry(self, data_port_id):
        """Selects the port entry belonging to the given data_port_id"""
        ctr = 0
        for data_port_entry in self.data_port_list_store:
            # Compare transition ids
            if data_port_entry[3] == data_port_id:
                self.view[self.view.top].set_cursor(ctr)
                break
            ctr += 1

    def get_path(self):
        """Returns the path/index to the currently selected port entry"""
        cursor = self.view[self.view.top].get_cursor()
        if cursor[0] is None:
            return None
        return cursor[0][0]

    def on_name_changed(self, widget, path, text):
        """Try to set the port name to the newly entered one
        """
        try:
            data_port_id = self.get_data_port_id_from_selection()
            self.state_data_port_dict[data_port_id].name = text
        except TypeError as e:
            logger.error("Error while trying to change the port name: {0}".format(e))

    def on_data_type_changed(self, widget, path, text):
        """Try to set the port type the the newly entered one
        """
        try:
            data_port_id = self.get_data_port_id_from_selection()
            self.state_data_port_dict[data_port_id].change_data_type(text, None)
        except (TypeError, AttributeError) as e:
            logger.error("Error while changing data type: {0}".format(e))

    def on_default_value_changed(self, widget, path, text):
        """Try to set the port default value to the newly entered one
        """
        try:
            data_port_id = self.get_data_port_id_from_selection()
            self.state_data_port_dict[data_port_id].default_value = text
        except (TypeError, AttributeError) as e:
            logger.error("Error while changing default value: {0}".format(e))

    def reload_data_port_list_store(self):
        """Reloads the input data port list store from the data port models
        """
        tmp = ListStore(str, str, str, int)
        for idp_model in self.data_port_model_list:
            tmp.append([idp_model.data_port.name, idp_model.data_port.data_type, idp_model.data_port.default_value,
                        idp_model.data_port.data_port_id])
        tms = gtk.TreeModelSort(tmp)
        tms.set_sort_column_id(0, gtk.SORT_ASCENDING)
        tms.set_sort_func(0, StateModel.dataport_compare_method)
        tms.sort_column_changed()
        tmp = tms
        self.data_port_list_store.clear()
        for elem in tmp:
            self.data_port_list_store.append(elem)