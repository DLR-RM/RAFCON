import gtk
from gtkmvc import Controller
from gtkmvc import Observer

from mvc.controllers.io_data_port_list import DataPortListController
from mvc.controllers.scoped_variable_list import ScopedVariableListController
from utils import log
logger = log.get_logger(__name__)


class StateDataPortEditorController(Controller):

    #model will be a container state model
    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)
        self.idp_list_ctrl = DataPortListController(model, view.input_port_list_view, "input")
        self.odp_list_ctrl = DataPortListController(model, view.output_port_list_view, "output")
        self.sv_list_ctrl = ScopedVariableListController(model, view.scoped_variables_list_view)

        view['new_input_port_button'].connect('clicked', self.idp_list_ctrl.on_new_input_port_button_clicked)
        view['new_output_port_button'].connect('clicked', self.odp_list_ctrl.on_new_output_port_button_clicked)
        view['new_scoped_variable_button'].connect('clicked', self.sv_list_ctrl.on_new_scoped_variable_button_clicked)

        view['delete_input_port_button'].connect('clicked', self.idp_list_ctrl.on_delete_input_port_button_clicked)
        view['delete_output_port_button'].connect('clicked', self.odp_list_ctrl.on_delete_output_port_button_clicked)
        view['delete_scoped_variable_button'].connect('clicked',
                                                      self.sv_list_ctrl.on_delete_scoped_variable_button_clicked)

    def register_view(self, view):
        view['state_dataport_editor'].connect('destroy', gtk.main_quit)

    @Observer.observe("state", after=True)
    def assign_notification_state(self, model, prop_name, info):
        if info.method_name == "add_input_data_port" or info.method_name == "remove_input_data_port":
            model.update_input_data_port_list_store_and_models()
        elif info.method_name == "add_output_data_port" or info.method_name == "remove_output_data_port":
            model.update_output_data_port_list_store_and_models()
        elif info.method_name == "add_scoped_variable" or info.method_name == "remove_scoped_variable":
            model.update_scoped_variables_list_store()