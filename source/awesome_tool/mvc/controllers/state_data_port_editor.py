import gtk

from awesome_tool.mvc.controllers.extended_controller import ExtendedController
from awesome_tool.mvc.controllers.io_data_port_list import DataPortListController
from awesome_tool.mvc.controllers.scoped_variable_list import ScopedVariableListController
from awesome_tool.utils import log
logger = log.get_logger(__name__)


class StateDataPortEditorController(ExtendedController):
    """
    Important Note: This class is only used for debugging purposes. For the GUI look at the StateEditor
    """
    #model will be a container state model
    def __init__(self, model, view):
        """Constructor
        """
        ExtendedController.__init__(self, model, view)
        self.add_controller('idp_list_ctrl', DataPortListController(model, view.input_port_list_view, "input"))
        self.add_controller('odp_list_ctrl', DataPortListController(model, view.output_port_list_view, "output"))
        self.add_controller('sv_list_ctrl', ScopedVariableListController(model, view.scoped_variables_list_view))

        view['new_input_port_button'].connect('clicked', self.get_controller(
            'idp_list_ctrl').on_new_input_port_button_clicked)
        view['new_output_port_button'].connect('clicked', self.get_controller(
            'odp_list_ctrl').on_new_output_port_button_clicked)
        view['new_scoped_variable_button'].connect('clicked', self.get_controller(
            'sv_list_ctrl').on_new_scoped_variable_button_clicked)

        view['delete_input_port_button'].connect('clicked', self.get_controller(
            'idp_list_ctrl').on_delete_input_port_button_clicked)
        view['delete_output_port_button'].connect('clicked', self.get_controller(
            'odp_list_ctrl').on_delete_output_port_button_clicked)
        view['delete_scoped_variable_button'].connect('clicked', self.get_controller(
            'sv_list_ctrl').on_delete_scoped_variable_button_clicked)

        self.get_controller('idp_list_ctrl').reload_data_port_list_store()
        self.get_controller('odp_list_ctrl').reload_data_port_list_store()
        self.get_controller('sv_list_ctrl').reload_scoped_variables_list_store()

    def register_view(self, view):
        view['state_dataport_editor'].connect('destroy', gtk.main_quit)

