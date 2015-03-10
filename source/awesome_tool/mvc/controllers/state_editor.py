
import gtk

from awesome_tool.mvc.controllers.extended_controller import ExtendedController
from awesome_tool.mvc.controllers import StateOverviewController, StateConnectionsEditorController, SourceEditorController, \
    DataPortListController, ScopedVariableListController, StateOutcomesEditorController

from awesome_tool.mvc.controllers.state_transitions import StateTransitionsEditorController
from awesome_tool.mvc.controllers.state_data_flows import StateDataFlowsEditorController
from awesome_tool.mvc.models import StateModel


class StateEditorController(ExtendedController):
    """Controller handling the view of properties/attributes of the
    """

    # TODO Missing functions

    def __init__(self, model, view):
        """Constructor
        """
        ExtendedController.__init__(self, model, view)
        assert isinstance(model, StateModel)

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

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        view['new_input_port_button'].connect('clicked', self.get_controller(
            'inputs_ctrl').on_new_input_port_button_clicked)
        view['new_output_port_button'].connect('clicked', self.get_controller(
            'outputs_ctrl').on_new_output_port_button_clicked)
        view['new_scoped_variable_button'].connect('clicked', self.get_controller(
            'scoped_ctrl').on_new_scoped_variable_button_clicked)

        view['delete_input_port_button'].connect('clicked', self.get_controller(
            'inputs_ctrl').on_delete_input_port_button_clicked)
        view['delete_output_port_button'].connect('clicked', self.get_controller(
            'outputs_ctrl').on_delete_output_port_button_clicked)
        view['delete_scoped_variable_button'].connect('clicked', self.get_controller(
            'scoped_ctrl').on_delete_scoped_variable_button_clicked)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        #self.adapt(self.__state_property_adapter("name", "input_name"))

    def register_actions(self, shortcut_manager):
        pass

    def model_changed(self, model):
        for controller in self.get_child_controllers():
            controller.model = model

    @ExtendedController.observe("input_data_ports", after=True)
    def input_data_ports_changed(self, model, prop_name, info):
        self.model.reload_input_data_port_list_store()

    @ExtendedController.observe("output_data_ports", after=True)
    def output_data_ports_changed(self, model, prop_name, info):
        self.model.reload_output_data_port_list_store()


class StateEditorEggController(ExtendedController):
    """Controller handling the view of properties/attributes of the
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

        self.arrangement_dict = {}

        view['inputs_view'].show()
        view['outputs_view'].show()
        view['scopes_view'].show()
        view['outcomes_view'].show()
        view['source_view'].show()
        view['transitions_view'].show()
        view['data_flows_view'].show()

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        view['new_input_port_button'].connect('clicked', self.get_controller(
            'inputs_ctrl').on_new_input_port_button_clicked)
        view['new_output_port_button'].connect('clicked', self.get_controller(
            'outputs_ctrl').on_new_output_port_button_clicked)
        view['new_scoped_variable_button'].connect('clicked', self.get_controller(
            'scoped_ctrl').on_new_scoped_variable_button_clicked)

        view['delete_input_port_button'].connect('clicked', self.get_controller(
            'inputs_ctrl').on_delete_input_port_button_clicked)
        view['delete_output_port_button'].connect('clicked', self.get_controller(
            'outputs_ctrl').on_delete_output_port_button_clicked)
        view['delete_scoped_variable_button'].connect('clicked', self.get_controller(
            'scoped_ctrl').on_delete_scoped_variable_button_clicked)

        view['port_expander'].connect('size-request', self.resize_port_widget)
        view['port_expander1'].connect('size-request', self.resize_port_widget)
        view['port_expander2'].connect('size-request', self.resize_port_widget)
        view['port_expander3'].connect('size-request', self.resize_port_widget)
        view['port_expander4'].connect('size-request', self.resize_port_widget)
        view['connections_expander'].connect('size-request', self.resize_connections_widget)
        view['transitions_expander'].connect('size-request', self.resize_connections_widget)
        view['dataflows_expander'].connect('size-request', self.resize_connections_widget)

        self.arrangement_dict['properties_viewport'] = {'view': 'properties_view', 'list_store': None, 'min': 55, 'max': 170, 'act': 25}
        self.arrangement_dict['port_expander'] = {'view': None, 'list_store': None, 'min': 25, 'max': 110, 'act': 25}
        self.arrangement_dict['port_expander1'] = {'view': 'inputs_view', 'list_store': None, 'min': 55, 'max': 170, 'act': 25}
        self.arrangement_dict['port_expander2'] = {'view': 'outputs_view', 'list_store': None, 'min': 55, 'max': 170, 'act': 25}
        self.arrangement_dict['port_expander3'] = {'view': 'scopess_view', 'list_store': None, 'min': 55, 'max': 170, 'act': 25}
        self.arrangement_dict['port_expander4'] = {'view': 'outcomes_view', 'list_store': None, 'min': 55, 'max': 170, 'act': 25}
        self.arrangement_dict['connections_expander'] = {'view': None, 'list_store': None, 'min': 25, 'max': 65, 'act': 25}
        self.arrangement_dict['transitions_expander'] = {'view': 'transitions_view', 'list_store': None, 'min': 55, 'max': 170, 'act': 25}
        self.arrangement_dict['dataflows_expander'] = {'view': 'data_flows_view', 'list_store': None, 'min': 55, 'max': 170, 'act': 25}

        self.arrangement_dict['port_expander1']['list_store'] = self.model.input_data_port_list_store
        self.arrangement_dict['port_expander2']['list_store'] = self.model.output_data_port_list_store
        self.arrangement_dict['port_expander4']['list_store'] = self.get_controller(
            'outcomes_ctrl').oc_list_ctrl.tree_store

        if hasattr(self.model, 'states'):
            self.arrangement_dict['port_expander3']['list_store'] = self.model.scoped_variables_list_store
            self.port_expander = ['port_expander1', 'port_expander2', 'port_expander3', 'port_expander4']
        else:
            self.view['port_expander3'].destroy()
            self.port_expander = ['port_expander1', 'port_expander2', 'port_expander4']
        self.arrangement_dict['transitions_expander']['list_store'] = self.get_controller(
            'transitions_ctrl').trans_list_ctrl.tree_store
        self.arrangement_dict['dataflows_expander']['list_store'] = self.get_controller(
            'data_flows_ctrl').df_list_ctrl.tree_store

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        #self.adapt(self.__state_property_adapter("name", "input_name"))

    def register_actions(self, shortcut_manager):
        pass

    def resize_port_widget(self, expander, x):

        if self.view['port_expander'].get_expanded():
            count = 0
            count_rows = 0
            active_l = {}
            port_exp_num = len(self.port_expander)
            for expand_id in self.port_expander:

                # if self.arrangement_dict[expand_id]['list_store'] is None:
                #     self.view[expand_id].set_size_request(width=-1, height=1)
                if self.view[expand_id].get_expanded():
                    # print "%s holds %s rows" % (expand_id, len(self.arrangement_dict[expand_id]['list_store']))
                    if self.arrangement_dict[expand_id]['list_store'] is not None:
                        active_l[expand_id] = len(self.arrangement_dict[expand_id]['list_store'])
                        if active_l[expand_id] == 0:
                            active_l[expand_id] = 1
                    else:
                        active_l[expand_id] = -1
                    if active_l[expand_id] > 4:
                        count_rows += 5
                        self.view[expand_id].set_size_request(width=-1, height=70+22*5)
                    else:
                        count_rows += active_l[expand_id]
                        self.view[expand_id].set_size_request(width=-1, height=70+22*active_l[expand_id])
                    # print "%s is expanded" % ('port_expander'+str(i))
                    count += 1
                    # print self.view[expand_id].get_parent()
                else:
                    if self.view[expand_id] == expander:
                        self.view[expand_id].set_size_request(width=-1, height=-1)

            if count_rows*22+port_exp_num*25+count*60 > 500:
                self.view['port_expander'].set_size_request(width=-1, height=500)
                # for expand_id in active_l.keys():
                #     avg_l = (500-count*60-110)/count
                #     if avg_l < active_l[key]:
                #         self.view[expand_id].set_size_request(width=-1, height=60+int(22*avg_l)+22)
            else:
                self.view['port_expander'].set_size_request(width=-1, height=count*70+22*count_rows+port_exp_num*25)
        else:
            self.view['port_expander'].set_size_request(width=-1, height=-1)
            for i in range(1, 5):
                expand_id = 'port_expander'+str(i)
                self.view[expand_id].set_size_request(width=-1, height=-1)

    def resize_connections_widget(self, expander, x=None):

        alloc = self.view['source_expander'].get_allocation()
        # print "Size of Source_expander: height %s, width %s" % (alloc.height, alloc.width)
        if self.view['connections_expander'].get_expanded():
            count = 0
            count_rows = 0
            active_l = {}
            for expand_id in ['transitions_expander', 'dataflows_expander']:
                if self.view[expand_id].get_expanded():
                    # print "%s holds %s rows" % (expand_id, len(self.arrangement_dict[expand_id]['list_store']))
                    active_l[expand_id] = len(self.arrangement_dict[expand_id]['list_store'])
                    if active_l[expand_id] > 4:
                        count_rows += 5
                        self.view[expand_id].set_size_request(width=-1, height=60+22*5+22)
                    else:
                        count_rows += len(self.arrangement_dict[expand_id]['list_store'])
                        self.view[expand_id].set_size_request(width=-1, height=60+22*active_l[expand_id]+22)
                    # print "%s is expanded" % expand_id
                    count += 1
                else:
                    if self.view[expand_id] == expander:
                        self.view[expand_id].set_size_request(width=-1, height=-1)

            if count_rows*22+65+count*60 > 360:
                self.view['connections_expander'].set_size_request(width=-1, height=350)
                # for expand_id in active_l.keys():
                #     avg_l = (350-count*60-55)/count
                #     if avg_l < active_l[key]:
                #         self.view[expand_id].set_size_request(width=-1, height=60+22*avg_l+22)
            else:
                self.view['connections_expander'].set_size_request(width=-1, height=count*60+22*count_rows+65)

            #self.view['vpaned1'].set_position(1000)
        else:
            self.view['connections_expander'].set_size_request(width=-1, height=-1)
            for expand_id in ['transitions_expander', 'dataflows_expander']:
                self.view[expand_id].set_size_request(width=-1, height=-1)
            #self.view['vpaned1'].set_position(1000)
            # print "position: %s" % self.view['vpaned1'].get_position()

    @ExtendedController.observe("input_data_ports", after=True)
    def input_data_ports_changed(self, model, prop_name, info):
        self.model.reload_input_data_port_list_store()

    @ExtendedController.observe("output_data_ports", after=True)
    def output_data_ports_changed(self, model, prop_name, info):
        self.model.reload_output_data_port_list_store()


class StateEditorLDController(ExtendedController):
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

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        view['new_input_port_button'].connect('clicked', self.get_controller(
            'inputs_ctrl').on_new_input_port_button_clicked)
        view['new_output_port_button'].connect('clicked', self.get_controller(
            'outputs_ctrl').on_new_output_port_button_clicked)
        view['new_scoped_variable_button'].connect('clicked', self.get_controller(
            'scoped_ctrl').on_new_scoped_variable_button_clicked)

        view['delete_input_port_button'].connect('clicked', self.get_controller(
            'inputs_ctrl').on_delete_input_port_button_clicked)
        view['delete_output_port_button'].connect('clicked', self.get_controller(
            'outputs_ctrl').on_delete_output_port_button_clicked)
        view['delete_scoped_variable_button'].connect('clicked', self.get_controller(
            'scoped_ctrl').on_delete_scoped_variable_button_clicked)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        #self.adapt(self.__state_property_adapter("name", "input_name"))

    def register_actions(self, shortcut_manager):
        pass

    @ExtendedController.observe("input_data_ports", after=True)
    def input_data_ports_changed(self, model, prop_name, info):
        self.model.reload_input_data_port_list_store()

    @ExtendedController.observe("output_data_ports", after=True)
    def output_data_ports_changed(self, model, prop_name, info):
        self.model.reload_output_data_port_list_store()