from utils import log
logger = log.get_logger(__name__)

from gtkmvc import Controller
import gtk
import gobject


class ScopedVariableListController(Controller):

    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)

        self.new_sv_counter = 0

    def register_view(self, view):
        """Called when the View was registered
        """

        #top widget is a tree view => set the model of the tree view to be a list store
        if hasattr(self.model, 'scoped_variables_list_store'):
            view.get_top_widget().set_model(self.model.scoped_variables_list_store)
        else:
            view.get_top_widget().set_model(gtk.ListStore(gobject.TYPE_PYOBJECT))

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

    @Controller.observe("scoped_variables", after=True)
    def input_data_ports_changed(self, model, prop_name, info):
        self.model.reload_scoped_variables_list_store()

    def on_new_scoped_variable_button_clicked(self, widget, data=None):
        new_sv_name = "a_new_scoped_variable%s" % str(self.new_sv_counter)
        if hasattr(self.model, 'states'):
            self.new_sv_counter += 1
            self.model.state.add_scoped_variable(new_sv_name, "str", "val")

    def on_delete_scoped_variable_button_clicked(self, widget, data=None):
        tree_view = self.view["scoped_variables_tree_view"]
        if hasattr(self.model, 'states'):
            path = tree_view.get_cursor()[0][0]
            if path is not None:
                scoped_variable_key = self.model.scoped_variables_list_store[int(path)][3]
                self.model.scoped_variables_list_store.clear()
                self.model.state.remove_scoped_variable(scoped_variable_key)

    def on_name_changed(self, widget, path, text):
        scoped_variable_id = self.model.scoped_variables_list_store[int(path)][3]
        self.model.state.modify_scoped_variable_name(text, scoped_variable_id)

    def on_data_type_changed(self, widget, path, text):
        data_port_id = self.model.scoped_variables_list_store[int(path)][3]
        self.model.state.modify_scoped_variable_data_type(text, data_port_id)

    def on_default_value_changed(self, widget, path, text):
        data_port_id = self.model.scoped_variables_list_store[int(path)][3]
        self.model.state.modify_scoped_variable_default_value(text, data_port_id)