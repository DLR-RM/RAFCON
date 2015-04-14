
from awesome_tool.mvc.controllers.extended_controller import ExtendedController
from awesome_tool.mvc.controllers import StateOverviewController, SourceEditorController, \
    DataPortListController, ScopedVariableListController, StateOutcomesEditorController

from awesome_tool.mvc.controllers.state_transitions import StateTransitionsEditorController
from awesome_tool.mvc.controllers.state_data_flows import StateDataFlowsEditorController
from awesome_tool.mvc.models import ContainerStateModel


class StateEditorController(ExtendedController):
    """Controller handles the organization of the Logic-Data oriented State-Editor.
    Widgets concerning logic flow (outcomes and transitions) are grouped in the Logic Linkage expander.
    Widgets concerning data flow (data-ports and data-flows) are grouped in the data linkage expander.
    """

    def __init__(self, model, view):
        """Constructor
        """
        ExtendedController.__init__(self, model, view)

        self.add_controller('properties_ctrl', StateOverviewController(model, view['properties_view']))

        self.add_controller('inputs_ctrl', DataPortListController(model, view['inputs_view'], "input"))
        self.add_controller('outputs_ctrl', DataPortListController(model, view['outputs_view'], "output"))
        self.add_controller('scoped_ctrl', ScopedVariableListController(model, view['scopes_view']))
        self.add_controller('outcomes_ctrl', StateOutcomesEditorController(model, view['outcomes_view']))

        self.add_controller('source_ctrl', SourceEditorController(model, view['source_view']))
        self.add_controller('transitions_ctrl', StateTransitionsEditorController(model, view['transitions_view']))
        self.add_controller('data_flows_ctrl', StateDataFlowsEditorController(model, view['data_flows_view']))

        view['inputs_view'].show()
        view['outputs_view'].show()
        view['scopes_view'].show()
        view['outcomes_view'].show()
        view['source_view'].show()
        view['transitions_view'].show()
        view['data_flows_view'].show()

        if isinstance(model, ContainerStateModel):
            self.get_controller('scoped_ctrl').reload_scoped_variables_list_store()

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        view['new_input_port_button'].connect('clicked',
            self.get_controller('inputs_ctrl').on_new_port_button_clicked)
        view['new_output_port_button'].connect('clicked',
            self.get_controller('outputs_ctrl').on_new_port_button_clicked)
        view['new_scoped_variable_button'].connect('clicked',
            self.get_controller('scoped_ctrl').on_new_scoped_variable_button_clicked)

        view['delete_input_port_button'].connect('clicked',
            self.get_controller('inputs_ctrl').on_delete_port_button_clicked)
        view['delete_output_port_button'].connect('clicked',
            self.get_controller('outputs_ctrl').on_delete_port_button_clicked)
        view['delete_scoped_variable_button'].connect('clicked',
            self.get_controller('scoped_ctrl').on_delete_scoped_variable_button_clicked)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        #self.adapt(self.__state_property_adapter("name", "input_name"))
