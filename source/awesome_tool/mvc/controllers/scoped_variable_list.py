from utils import log
logger = log.get_logger(__name__)

from gtkmvc import Controller


class ScopedVariableListController(Controller):

    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)

    def register_view(self, view):
        """Called when the View was registered
        """

        def cell_text(column, cell_renderer, model, iter, container_model):
            col = column.get_name()
            scoped_variable = model.get_value(iter, 0)
            if col == 'name_col':
                name = container_model.container_state.scoped_variables[scoped_variable.name].name
                cell_renderer.set_property('text', name)
            elif col == 'data_type_col':
                data_type = container_model.container_state.scoped_variables[scoped_variable.name].data_type
                cell_renderer.set_property('text', data_type)
            elif col == 'default_value_col':
                default_value = container_model.container_state.scoped_variables[scoped_variable.name].default_value
                cell_renderer.set_property('text', default_value)
            else:
                logger.error("Unknown column '{col:s}' in ScopedVariableListView".format(col=col))

        #top widget is a tree view => set the model of the tree view to be a list store

        view.get_top_widget().set_model(self.model.scoped_variables_list_store)

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
        #print self.view.get_top_widget().get_selection().get_selected_rows()
        key = self.model.scopeded_variables[int(path)][0].name
        old_scoped_variable = copy.copy(self.model.container_state.scoped_variables[key])
        del self.model.container_state.scoped_variables[key]
        print old_scoped_variable
        #the text is the new key
        self.model.container_state.add_scoped_variable(text, old_scoped_variable.data_type,
                                                       old_scoped_variable.default_value)
        self.view.get_top_widget().set_model(self.model.scopeded_variables)
        self.model.update_scoped_variables_list_store()


    def on_data_type_changed(self, widget, path, text):
        print path
        old_data_port = self.model.scoped_variables_list_store[int(path)][0]
        print old_data_port
        self.model.container_state.scoped_variables[old_data_port.name].data_type = text
        self.model.container_state.scoped_variables[old_data_port.name].default_value = None

    def on_default_value_changed(self, widget, path, text):
        print path
        old_data_port = self.model.container_state.scoped_variables[int(path)][0]
        print old_data_port
        converted_value = None

        #TODO: how to support more data types (especially classes)
        if old_data_port.data_type == "str":
            converted_value = str(text)

        if old_data_port.data_type == "int":
            converted_value = int(text)

        if old_data_port.data_type == "float":
            converted_value = float(text)

        self.state_dataport_dict[old_data_port.name].default_value = converted_value




