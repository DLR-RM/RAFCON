
from utils import log
logger = log.get_logger(__name__)

import gtk
from gtkmvc import Controller
from mvc.controllers.io_data_port_list import DataPortListController
from mvc.controllers.scoped_variable_list import ScopedVariableListController
from gtkmvc import Observer


class StateDataPortEditorController(Controller, Observer):

    #model will be a container state model
    def __init__(self, model, view):
        """Constructor
        """
        #self.model = model
        #self.view = view
        Controller.__init__(self, model, view)
        self.input_data_port_list_controller = DataPortListController(model, view.input_port_list_view, "input")
        self.output_data_port_list_controller = DataPortListController(model, view.output_port_list_view, "output")
        self.scoped_variable_list_controller = ScopedVariableListController(model, view.scoped_variables_list_view)

        view['new_input_port_button'].connect('clicked', self.on_new_input_port_button_clicked)
        view['new_output_port_button'].connect('clicked', self.on_new_output_port_button_clicked)
        view['new_scoped_variable_button'].connect('clicked', self.on_new_scoped_variable_button_clicked)

        view['delete_input_port_button'].connect('clicked', self.on_delete_input_port_button_clicked)
        view['delete_output_port_button'].connect('clicked', self.on_delete_output_port_button_clicked)
        view['delete_scoped_variable_button'].connect('clicked', self.on_delete_scoped_variable_button_clicked)

    def on_new_input_port_button_clicked(self, widget, data=None):
        self.model.state.add_input_data_port("a_new_input_port", "str", "val")

    def on_new_output_port_button_clicked(self, widget, data=None):
        self.model.state.add_output_data_port("a_new_output_port", "str", "val")

    def on_new_scoped_variable_button_clicked(self, widget, data=None):
        self.model.container_state.add_scoped_variable("a_new_scoped_variable", "str", "val")

    def on_delete_input_port_button_clicked(self, widget, data=None):
        tree_view = self.view.input_port_list_view["input_ports_tree_view"]
        #print tree_view
        path = tree_view.get_cursor()[0]
        if path is not None:
            #print path
            key = self.model.input_data_port_list_store[int(path[0])][0].name
            #print key
            self.model.state.remove_input_data_port(key)

    def on_delete_output_port_button_clicked(self, widget, data=None):
        tree_view = self.view.output_port_list_view["output_ports_tree_view"]
        path = tree_view.get_cursor()[0]
        if path is not None:
            key = self.model.output_data_port_list_store[int(path[0])][0].name
            self.model.state.remove_output_data_port(key)

    def on_delete_scoped_variable_button_clicked(self, widget, data=None):
        tree_view = self.view.scoped_variables_list_view["scoped_variables_tree_view"]
        path = tree_view.get_cursor()[0]
        if path is not None:
            key = self.model.scoped_variables_list_store[int(path[0])][0].name
            self.model.container_state.remove_scoped_variable(key)


    def register_view(self, view):

        view['state_dataport_editor'].connect('destroy', gtk.main_quit)

    #TODO: separate functions for inputs, outputs, and scoped vars
    @Observer.observe("state", after=True)
    def assign_notification_state(self, model, prop_name, info):
        print "call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
              (prop_name, info.instance, info.method_name, info.result)
        if info.method_name == "add_input_data_port" or info.method_name == "remove_input_data_port":
            model.update_input_data_port_list_store()
        elif info.method_name == "add_output_data_port" or info.method_name == "remove_output_data_port":
            model.update_output_data_port_list_store()
        elif info.method_name == "add_scoped_variable" or info.method_name == "remove_scoped_variable":
            model.update_scoped_variables_list_store()

