import gtk
from gtk import ListStore
import copy

from awesome_tool.utils import log
logger = log.get_logger(__name__)

from awesome_tool.mvc.controllers.extended_controller import ExtendedController
from awesome_tool.mvc.models.state import StateModel


class DataPortListController(ExtendedController):

    def __init__(self, model, view, io_type):
        """Constructor
        """
        ExtendedController.__init__(self, model, view)
        self.type = io_type
        self.state_dataport_dict = None
        self.dataport_list_store = None

        self.new_port_counter = 0

        if self.type == "input":
            self.state_dataport_dict = self.model.state.input_data_ports
            self.dataport_model_list = self.model.input_data_ports
        elif self.type == "output":
            self.state_dataport_dict = self.model.state.output_data_ports
            self.dataport_model_list = self.model.output_data_ports

        self.dataport_list_store = ListStore(str, str, str, int)

    def register_view(self, view):
        """Called when the View was registered
        """

        #top widget is a tree view => set the model of the tree view to be a list store
        view.get_top_widget().set_model(self.dataport_list_store)
        view.get_top_widget().set_cursor(0)

        view['name_col'].add_attribute(view['name_text'], 'text', 0)
        view['name_text'].set_property("editable", True)
        view['data_type_col'].add_attribute(view['data_type_text'], 'text', 1)
        view['data_type_text'].set_property("editable", True)
        view['default_value_col'].add_attribute(view['default_value_text'], 'text', 2)
        view['default_value_text'].set_property("editable", True)

        view['name_text'].connect("edited", self.on_name_changed)
        view['data_type_text'].connect("edited", self.on_data_type_changed)
        view['default_value_text'].connect("edited", self.on_default_value_changed)

    def register_adapters(self):
        """Adapters should be registered in this method call
        """

    @ExtendedController.observe("input_data_ports", after=True)
    def input_data_ports_changed(self, model, prop_name, info):
        if self.type == "input":
            self.reload_data_port_list_store()

    @ExtendedController.observe("output_data_ports", after=True)
    def output_data_ports_changed(self, model, prop_name, info):
        if self.type == "output":
            self.reload_data_port_list_store()

    #new buttons
    def on_new_input_port_button_clicked(self, widget, data=None):
        new_input_port_name = "a_new_input_port%s" % str(self.new_port_counter)
        self.new_port_counter += 1
        self.model.state.add_input_data_port(new_input_port_name, "str", "val")

    def on_new_output_port_button_clicked(self, widget, data=None):
        new_output_port_name = "a_new_output_port%s" % str(self.new_port_counter)
        self.new_port_counter += 1
        self.model.state.add_output_data_port(new_output_port_name, "str", "val")

    def on_delete_input_port_button_clicked(self, widget, data=None):
        tree_view = self.view["input_ports_tree_view"]
        path = tree_view.get_cursor()[0][0]
        if path is not None:
            data_port_id = copy.copy(self.dataport_list_store[int(path)][3])
            self.dataport_list_store.clear()
            self.model.state.remove_input_data_port(data_port_id)

    def on_delete_output_port_button_clicked(self, widget, data=None):
        tree_view = self.view["output_ports_tree_view"]
        path = tree_view.get_cursor()[0][0]
        if path is not None:
            data_port_id = copy.copy(self.dataport_list_store[int(path)][3])
            self.dataport_list_store.clear()
            self.model.state.remove_output_data_port(data_port_id)

    def on_name_changed(self, widget, path, text):
        data_port_id = self.dataport_list_store[int(path)][3]

        if self.type == "input":
            self.model.state.input_data_ports[data_port_id].name = text
        elif self.type == "output":
            self.model.state.output_data_ports[data_port_id].name = text

    def on_data_type_changed(self, widget, path, text):
        data_port_id = self.dataport_list_store[int(path)][3]
        if self.type == "input":
            self.model.state.modify_input_data_port_data_type(text, data_port_id)
        elif self.type == "output":
            self.model.state.modify_output_data_port_data_type(text, data_port_id)

    def on_default_value_changed(self, widget, path, text):
        data_port_id = self.dataport_list_store[int(path)][3]
        if self.type == "input":
            self.model.state.modify_input_data_port_default_value(text, data_port_id)
        elif self.type == "output":
            self.model.state.modify_output_data_port_default_value(text, data_port_id)

    def reload_data_port_list_store(self):
        """Reloads the input data port list store from the data port models
        """
        tmp = ListStore(str, str, str, int)
        if self.type == "input":
            for idp_model in self.model.input_data_ports:
                # print idp_model.parent.state.state_id, self.state.state_id
                tmp.append([idp_model.data_port.name, idp_model.data_port.data_type, idp_model.data_port.default_value,
                            idp_model.data_port.data_port_id])
        else:
            for idp_model in self.model.output_data_ports:
                # print idp_model.parent.state.state_id, self.state.state_id
                tmp.append([idp_model.data_port.name, idp_model.data_port.data_type, idp_model.data_port.default_value,
                            idp_model.data_port.data_port_id])
        tms = gtk.TreeModelSort(tmp)
        tms.set_sort_column_id(0, gtk.SORT_ASCENDING)
        tms.set_sort_func(0, StateModel.dataport_compare_method)
        tms.sort_column_changed()
        tmp = tms
        self.dataport_list_store.clear()
        for elem in tmp:
            self.dataport_list_store.append(elem)