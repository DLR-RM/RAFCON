from utils import log
logger = log.get_logger(__name__)

from gtkmvc import Controller


class DataPortListController(Controller):

    def __init__(self, model, view, io_type):
        """Constructor
        """
        Controller.__init__(self, model, view)
        self.type = io_type
        self.state_dataport_dict = None
        self.dataport_list_store = None

        if self.type == "input":
            self.state_dataport_dict = self.model.state.input_data_ports
            self.dataport_list_store = self.model.input_data_port_list_store
        elif self.type == "output":
            self.state_dataport_dict = self.model.state.output_data_ports
            self.dataport_list_store = self.model.output_data_port_list_store

    def register_view(self, view):
        """Called when the View was registered
        """

        def cell_text(column, cell_renderer, model, iter, container_model):
            container_model_data_ports = None

            if self.type == "input":
                container_model_data_ports = container_model.state.input_data_ports
            elif self.type == "output":
                container_model_data_ports = container_model.state.output_data_ports

            col = column.get_name()
            data_port = model.get_value(iter, 0)
            if col == 'name_col':
                name = container_model_data_ports[data_port.data_port_id].name
                cell_renderer.set_property('text', name)
            elif col == 'data_type_col':
                data_type = container_model_data_ports[data_port.data_port_id].data_type
                cell_renderer.set_property('text', data_type)
            elif col == 'default_value_col':
                default_value = container_model_data_ports[data_port.data_port_id].default_value
                cell_renderer.set_property('text', default_value)
            else:
                logger.error("Unknown column '{col:s}' in DataPortListView".format(col=col))

        #top widget is a tree view => set the model of the tree view to be a list store
        if self.type == "input":
            view.get_top_widget().set_model(self.model.input_data_port_list_store)
        elif self.type == "output":
            view.get_top_widget().set_model(self.model.output_data_port_list_store)

        view['name_col'].set_cell_data_func(view['name_text'], cell_text, self.model)
        view['name_text'].set_property("editable", True)
        view['data_type_col'].set_cell_data_func(view['data_type_text'], cell_text, self.model)
        view['data_type_text'].set_property("editable", True)
        view['default_value_col'].set_cell_data_func(view['default_value_text'], cell_text, self.model)
        view['default_value_text'].set_property("editable", True)

        view['name_text'].connect("edited", self.on_name_changed)
        view['data_type_text'].connect("edited", self.on_data_type_changed)
        view['default_value_text'].connect("edited", self.on_default_value_changed)

    def register_adapters(self):
        """Adapters should be registered in this method call
        """

    def on_name_changed(self, widget, path, text):
        print path
        import copy
        logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))
        print self.view.get_top_widget().get_selection().get_selected_rows()
        key = self.dataport_list_store[int(path)][0].data_port_id
        old_data_port = copy.copy(self.state_dataport_dict[key])

        print self.state_dataport_dict[key]
        del self.state_dataport_dict[key]
        print self.dataport_list_store[int(path)]
        del self.dataport_list_store[int(path)]

        #the text is the new key

        if self.type == "input":
            self.model.state.add_input_data_port(text, old_data_port.data_type, old_data_port.default_value)
            self.dataport_list_store = self.model.input_data_port_list_store
            self.view.get_top_widget().set_model(self.model.input_data_port_list_store)
        elif self.type == "output":
            self.model.state.add_output_data_port(text, old_data_port.data_type, old_data_port.default_value)
            self.dataport_list_store = self.model.output_data_port_list_store
            self.view.get_top_widget().set_model(self.model.output_data_port_list_store)

        #self.model.update_input_data_port_list_store_and_models()
    def on_data_type_changed(self, widget, path, text):
        print path
        old_data_port = self.dataport_list_store[int(path)][0]
        print old_data_port
        self.state_dataport_dict[old_data_port.data_port_id].data_type = text
        self.state_dataport_dict[old_data_port.data_port_id].default_value = None

    def on_default_value_changed(self, widget, path, text):
        print path
        old_data_port = self.dataport_list_store[int(path)][0]
        print old_data_port
        converted_value = None

        #TODO: how to support more data types (especially classes)
        if old_data_port.data_type == "str":
            converted_value = str(text)

        if old_data_port.data_type == "int":
            converted_value = int(text)

        if old_data_port.data_type == "float":
            converted_value = float(text)

        self.state_dataport_dict[old_data_port.data_port_id].default_value = converted_value
