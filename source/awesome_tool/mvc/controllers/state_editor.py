
import gtk
from gtkmvc import Controller, Observer

from mvc.controllers import StateOverviewController, StateConnectionsEditorController, SourceEditorController, \
    DataPortListController, ScopedVariableListController, StateOutcomesEditorController

from mvc.controllers.state_transitions import StateTransitionsEditorController
from mvc.controllers.state_data_flows import StateDataFlowsEditorController


class StateEditorController(Controller):
    """Controller handling the view of properties/attributes of the
    """

    # TODO Missing functions

    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)
        self.properties_ctrl = StateOverviewController(model, view['properties_view'])

        self.inputs_ctrl = DataPortListController(model, view['inputs_view'], "input")  #ContainerStateController(model, view['inputs_view'])
        self.outputs_ctrl = DataPortListController(model, view['outputs_view'], "output")  # ContainerStateController(model, view['outputs_view'])
        self.scoped_ctrl = ScopedVariableListController(model, view['scopes_view'])
        self.outcomes_ctrl = StateOutcomesEditorController(model, view['outcomes_view'])

        self.source_ctrl = SourceEditorController(model, view['source_view'])
        self.transitions_ctrl = StateTransitionsEditorController(model, view['transitions_view'])
        self.data_flows_ctrl = StateDataFlowsEditorController(model, view['data_flows_view'])

        self.new_ip_counter = 0
        self.new_op_counter = 0
        self.new_sv_counter = 0

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
        view['new_input_port_button'].connect('clicked', self.on_new_input_port_button_clicked)
        view['new_output_port_button'].connect('clicked', self.on_new_output_port_button_clicked)
        view['new_scoped_variable_button'].connect('clicked', self.on_new_scoped_variable_button_clicked)

        view['delete_input_port_button'].connect('clicked', self.on_delete_input_port_button_clicked)
        view['delete_output_port_button'].connect('clicked', self.on_delete_output_port_button_clicked)
        view['delete_scoped_variable_button'].connect('clicked', self.on_delete_scoped_variable_button_clicked)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        #self.adapt(self.__state_property_adapter("name", "input_name"))

    #new buttons
    def on_new_input_port_button_clicked(self, widget, data=None):
        new_iport_name = "a_new_intput_port%s" % str(self.new_ip_counter)
        self.new_ip_counter += 1
        self.model.state.add_input_data_port(new_iport_name, "str", "val")

    def on_new_output_port_button_clicked(self, widget, data=None):
        new_oport_name = "a_new_output_port%s" % str(self.new_op_counter)
        self.new_op_counter += 1
        self.model.state.add_output_data_port(new_oport_name, "str", "val")

    def on_new_scoped_variable_button_clicked(self, widget, data=None):
        new_sv_name = "a_new_scoped_variable%s" % str(self.new_sv_counter)
        self.new_sv_counter += 1
        self.model.container_state.add_scoped_variable(new_sv_name, "str", "val")

    #delete buttons
    def on_delete_input_port_button_clicked(self, widget, data=None):
        tree_view = self.view['inputs_view']["input_ports_tree_view"]
        path = tree_view.get_cursor()[0]
        # print "pathremove: %s" % path
        if path is not None:
            key = self.model.input_data_port_list_store[int(path[0])][0].data_port_id
            # print "remove: %s" % key
            self.model.state.remove_input_data_port(key)

    def on_delete_output_port_button_clicked(self, widget, data=None):
        tree_view = self.view['outputs_view']["output_ports_tree_view"]
        path = tree_view.get_cursor()[0]
        # print "pathremove: %s" % path
        if path is not None:
            key = self.model.output_data_port_list_store[int(path[0])][0].data_port_id
            # print "remove: %s" % key
            self.model.state.remove_output_data_port(key)

    def on_delete_scoped_variable_button_clicked(self, widget, data=None):
        tree_view = self.view['scopes_view']["scoped_variables_tree_view"]
        path = tree_view.get_cursor()[0]
        # print "pathremove: %s" % path
        if path is not None:
            key = self.model.scoped_variables_list_store[int(path[0])][0].data_port_id
            # print "remove: %s" % key
            self.model.container_state.remove_scoped_variable(key)

    @Observer.observe("state", after=True)
    def assign_notification_state(self, model, prop_name, info):
        #print "call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
        #      (prop_name, info.instance, info.method_name, info.result)
        #model.update_input_data_port_list_store_and_models()
        if info.method_name == "add_input_data_port" or info.method_name == "remove_input_data_port":
            model.update_input_data_port_list_store_and_models()
        elif info.method_name == "add_output_data_port" or info.method_name == "remove_output_data_port":
            model.update_output_data_port_list_store_and_models()
        elif info.method_name == "add_scoped_variable" or info.method_name == "remove_scoped_variable":
            model.update_scoped_variables_list_store()


class StateEditorEggController(Controller):
    """Controller handling the view of properties/attributes of the
    """

    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)
        self.properties_ctrl = StateOverviewController(model, view['properties_view'])

        self.inputs_ctrl = DataPortListController(model, view['inputs_view'], "input")  #ContainerStateController(model, view['inputs_view'])
        self.outputs_ctrl = DataPortListController(model, view['outputs_view'], "output")  # ContainerStateController(model, view['outputs_view'])
        self.scoped_ctrl = ScopedVariableListController(model, view['scopes_view'])
        self.outcomes_ctrl = StateOutcomesEditorController(model, view['outcomes_view'])

        self.source_ctrl = SourceEditorController(model, view['source_view'])
        self.transitions_ctrl = StateTransitionsEditorController(model, view['transitions_view'])
        self.data_flows_ctrl = StateDataFlowsEditorController(model, view['data_flows_view'])

        self.new_ip_counter = 0
        self.new_op_counter = 0
        self.new_sv_counter = 0

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
        view['new_input_port_button'].connect('clicked', self.on_new_input_port_button_clicked)
        view['new_output_port_button'].connect('clicked', self.on_new_output_port_button_clicked)
        view['new_scoped_variable_button'].connect('clicked', self.on_new_scoped_variable_button_clicked)

        view['delete_input_port_button'].connect('clicked', self.on_delete_input_port_button_clicked)
        view['delete_output_port_button'].connect('clicked', self.on_delete_output_port_button_clicked)
        view['delete_scoped_variable_button'].connect('clicked', self.on_delete_scoped_variable_button_clicked)

        view['port_expander'].connect('size-request', self.resize_port_widget)
        view['port_expander1'].connect('size-request', self.resize_port_widget)
        view['port_expander2'].connect('size-request', self.resize_port_widget)
        view['port_expander3'].connect('size-request', self.resize_port_widget)
        view['port_expander4'].connect('size-request', self.resize_port_widget)
        view['connections_expander'].connect('size-request', self.resize_connections_widget)
        view['transitions_expander'].connect('size-request', self.resize_connections_widget)
        view['dataflows_expander'].connect('size-request', self.resize_connections_widget)

        self.arrangement_dict = {}
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
        self.arrangement_dict['port_expander3']['list_store'] = self.model.scoped_variables_list_store
        self.arrangement_dict['port_expander4']['list_store'] = self.outcomes_ctrl.oc_list_ctrl.tree_store

        self.arrangement_dict['transitions_expander']['list_store'] = self.transitions_ctrl.trans_list_ctrl.tree_store
        self.arrangement_dict['dataflows_expander']['list_store'] = self.data_flows_ctrl.df_list_ctrl.tree_store

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        #self.adapt(self.__state_property_adapter("name", "input_name"))

    def resize_port_widget(self, expander, x):

        if self.view['port_expander'].get_expanded():
            count = 0
            count_rows = 0
            exp_str = ''
            active_l = {}
            for i in range(1, 5):
                expand_id = 'port_expander'+str(i)
                if self.view[expand_id].get_expanded():
                    print "%s holds %s rows" % (expand_id, len(self.arrangement_dict[expand_id]['list_store']))
                    active_l[expand_id] = len(self.arrangement_dict[expand_id]['list_store'])
                    if active_l[expand_id] > 4:
                        count_rows += 5
                        self.view[expand_id].set_size_request(width=-1, height=60+22*5+22)
                    else:
                        count_rows += len(self.arrangement_dict[expand_id]['list_store'])
                        self.view[expand_id].set_size_request(width=-1, height=60+22*active_l[expand_id]+22)
                    # print "%s is expanded" % ('port_expander'+str(i))
                    count += 1
                    print self.view[expand_id].get_parent()
                else:
                    if self.view[expand_id] == expander:
                        self.view[expand_id].set_size_request(width=-1, height=-1)

                if expander is self.view[expand_id]:
                    exp_str = expand_id
                    print "expander is %s" % exp_str
            if count_rows*22+110+count*60 > 500:
                self.view['port_expander'].set_size_request(width=-1, height=500)
                # for expand_id in active_l.keys():
                #     avg_l = (500-count*60-110)/count
                #     if avg_l < active_l[key]:
                #         self.view[expand_id].set_size_request(width=-1, height=60+int(22*avg_l)+22)
            else:
                self.view['port_expander'].set_size_request(width=-1, height=count*60+22*count_rows+110)
        else:
            self.view['port_expander'].set_size_request(width=-1, height=-1)
            for i in range(1, 5):
                expand_id = 'port_expander'+str(i)
                self.view[expand_id].set_size_request(width=-1, height=-1)

    def resize_connections_widget(self, expander, x=None):

        alloc = self.view['source_expander'].get_allocation()
        print "Size of Source_expander: height %s, width %s" % (alloc.height, alloc.width)
        if self.view['connections_expander'].get_expanded():
            count = 0
            count_rows = 0
            active_l = {}
            for expand_id in ['transitions_expander', 'dataflows_expander']:
                if self.view[expand_id].get_expanded():
                    print "%s holds %s rows" % (expand_id, len(self.arrangement_dict[expand_id]['list_store']))
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
                if expander is self.view[expand_id]:
                        exp_str = expand_id
                        print "expander is %s" % exp_str

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

    #new buttons
    def on_new_input_port_button_clicked(self, widget, data=None):
        new_iport_name = "a_new_intput_port%s" % str(self.new_ip_counter)
        self.new_ip_counter += 1
        self.model.state.add_input_data_port(new_iport_name, "str", "val")

    def on_new_output_port_button_clicked(self, widget, data=None):
        new_oport_name = "a_new_output_port%s" % str(self.new_op_counter)
        self.new_op_counter += 1
        self.model.state.add_output_data_port(new_oport_name, "str", "val")

    def on_new_scoped_variable_button_clicked(self, widget, data=None):
        new_sv_name = "a_new_scoped_variable%s" % str(self.new_sv_counter)
        self.new_sv_counter += 1
        self.model.container_state.add_scoped_variable(new_sv_name, "str", "val")

    #delete buttons
    def on_delete_input_port_button_clicked(self, widget, data=None):
        tree_view = self.view['inputs_view']["input_ports_tree_view"]
        path = tree_view.get_cursor()[0]
        # print "pathremove: %s" % path
        if path is not None:
            key = self.model.input_data_port_list_store[int(path[0])][0].data_port_id
            # print "remove: %s" % key
            self.model.state.remove_input_data_port(key)

    def on_delete_output_port_button_clicked(self, widget, data=None):
        tree_view = self.view['outputs_view']["output_ports_tree_view"]
        path = tree_view.get_cursor()[0]
        # print "pathremove: %s" % path
        if path is not None:
            key = self.model.output_data_port_list_store[int(path[0])][0].data_port_id
            # print "remove: %s" % key
            self.model.state.remove_output_data_port(key)

    def on_delete_scoped_variable_button_clicked(self, widget, data=None):
        tree_view = self.view['scopes_view']["scoped_variables_tree_view"]
        path = tree_view.get_cursor()[0]
        # print "pathremove: %s" % path
        if path is not None:
            key = self.model.scoped_variables_list_store[int(path[0])][0].data_port_id
            # print "remove: %s" % key
            self.model.container_state.remove_scoped_variable(key)

    @Observer.observe("state", after=True)
    def assign_notification_state(self, model, prop_name, info):
        #print "call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
        #      (prop_name, info.instance, info.method_name, info.result)
        #model.update_input_data_port_list_store_and_models()
        if info.method_name == "add_input_data_port" or info.method_name == "remove_input_data_port":
            model.update_input_data_port_list_store_and_models()
        elif info.method_name == "add_output_data_port" or info.method_name == "remove_output_data_port":
            model.update_output_data_port_list_store_and_models()
        elif info.method_name == "add_scoped_variable" or info.method_name == "remove_scoped_variable":
            model.update_scoped_variables_list_store()


class StateEditorLDController(Controller):
    """Controller handling the view of properties/attributes of the
    """

    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)
        self.properties_ctrl = StateOverviewController(model, view['properties_view'])

        self.inputs_ctrl = DataPortListController(model, view['inputs_view'], "input")  #ContainerStateController(model, view['inputs_view'])
        self.outputs_ctrl = DataPortListController(model, view['outputs_view'], "output")  # ContainerStateController(model, view['outputs_view'])
        self.scoped_ctrl = ScopedVariableListController(model, view['scopes_view'])
        self.outcomes_ctrl = StateOutcomesEditorController(model, view['outcomes_view'])

        self.source_ctrl = SourceEditorController(model, view['source_view'])
        self.transitions_ctrl = StateTransitionsEditorController(model, view['transitions_view'])
        self.data_flows_ctrl = StateDataFlowsEditorController(model, view['data_flows_view'])

        self.new_ip_counter = 0
        self.new_op_counter = 0
        self.new_sv_counter = 0

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
        view['new_input_port_button'].connect('clicked', self.on_new_input_port_button_clicked)
        view['new_output_port_button'].connect('clicked', self.on_new_output_port_button_clicked)
        view['new_scoped_variable_button'].connect('clicked', self.on_new_scoped_variable_button_clicked)

        view['delete_input_port_button'].connect('clicked', self.on_delete_input_port_button_clicked)
        view['delete_output_port_button'].connect('clicked', self.on_delete_output_port_button_clicked)
        view['delete_scoped_variable_button'].connect('clicked', self.on_delete_scoped_variable_button_clicked)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        #self.adapt(self.__state_property_adapter("name", "input_name"))

    #new buttons
    def on_new_input_port_button_clicked(self, widget, data=None):
        new_iport_name = "a_new_intput_port%s" % str(self.new_ip_counter)
        self.new_ip_counter += 1
        self.model.state.add_input_data_port(new_iport_name, "str", "val")

    def on_new_output_port_button_clicked(self, widget, data=None):
        new_oport_name = "a_new_output_port%s" % str(self.new_op_counter)
        self.new_op_counter += 1
        self.model.state.add_output_data_port(new_oport_name, "str", "val")

    def on_new_scoped_variable_button_clicked(self, widget, data=None):
        new_sv_name = "a_new_scoped_variable%s" % str(self.new_sv_counter)
        self.new_sv_counter += 1
        self.model.container_state.add_scoped_variable(new_sv_name, "str", "val")

    #delete buttons
    def on_delete_input_port_button_clicked(self, widget, data=None):
        tree_view = self.view['inputs_view']["input_ports_tree_view"]
        path = tree_view.get_cursor()[0]
        # print "pathremove: %s" % path
        if path is not None:
            key = self.model.input_data_port_list_store[int(path[0])][0].data_port_id
            # print "remove: %s" % key
            self.model.state.remove_input_data_port(key)

    def on_delete_output_port_button_clicked(self, widget, data=None):
        tree_view = self.view['outputs_view']["output_ports_tree_view"]
        path = tree_view.get_cursor()[0]
        # print "pathremove: %s" % path
        if path is not None:
            key = self.model.output_data_port_list_store[int(path[0])][0].data_port_id
            # print "remove: %s" % key
            self.model.state.remove_output_data_port(key)

    def on_delete_scoped_variable_button_clicked(self, widget, data=None):
        tree_view = self.view['scopes_view']["scoped_variables_tree_view"]
        path = tree_view.get_cursor()[0]
        # print "pathremove: %s" % path
        if path is not None:
            key = self.model.scoped_variables_list_store[int(path[0])][0].data_port_id
            # print "remove: %s" % key
            self.model.container_state.remove_scoped_variable(key)

    @Observer.observe("state", after=True)
    def assign_notification_state(self, model, prop_name, info):
        #print "call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
        #      (prop_name, info.instance, info.method_name, info.result)
        #model.update_input_data_port_list_store_and_models()
        if info.method_name == "add_input_data_port" or info.method_name == "remove_input_data_port":
            model.update_input_data_port_list_store_and_models()
        elif info.method_name == "add_output_data_port" or info.method_name == "remove_output_data_port":
            model.update_output_data_port_list_store_and_models()
        elif info.method_name == "add_scoped_variable" or info.method_name == "remove_scoped_variable":
            model.update_scoped_variables_list_store()


if __name__ == '__main__':
    from mvc.views import StateEditorView, SingleWidgetWindowView
    from mvc.controllers import SingleWidgetWindowController

    from statemachine.states.execution_state import ExecutionState as State
    from mvc.models import StateModel, ContainerStateModel

    state1 = State('state2')
    m = StateModel(state1)

    import mvc.main as main

    main.setup_path()
    main.check_requirements()
    [ctr_model, logger, ctr_state] = main.main()

    v = SingleWidgetWindowView(StateEditorView, width=550, height=550, title='State Editor')
    c = SingleWidgetWindowController(ctr_model, v, StateEditorController)
    #c = StateEditorController(m, v)

    gtk.main()