from awesome_tool.utils import log
logger = log.get_logger(__name__)

from awesome_tool.mvc.controllers.extended_controller import ExtendedController
import copy


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
            self.dataport_list_store = self.model.input_data_port_list_store
        elif self.type == "output":
            self.state_dataport_dict = self.model.state.output_data_ports
            self.dataport_model_list = self.model.output_data_ports
            self.dataport_list_store = self.model.output_data_port_list_store

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
        #logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))
        data_port_id = self.dataport_list_store[int(path)][3]

        if self.type == "input":
            self.model.state.modify_input_data_port_name(text, data_port_id)
        elif self.type == "output":
            self.model.state.modify_output_data_port_name(text, data_port_id)

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
