

import gtk
from gtkmvc import View, Controller
from gtkmvc import Observer

from mvc.controllers import StateOverviewController, StateConnectionsEditorController, SourceEditorController, \
    DataPortListController, ScopedVariableListController, StateOutcomesEditorController, StateOutcomesTreeController


class StateEditorController(Controller):
    """Controller handling the view of properties/attributes of the ContainerStateModel

    This :class:`gtkmvc.Controller` class is the interface between the GTK widget view
    :class:`mvc.views.source_editor.SourceEditorView` and the properties of the
    :class:`mvc.models.state.StateModel`. Changes made in
    the GUI are written back to the model and vice versa.

    :param mvc.models.StateModel model: The state model containing the data
    :param mvc.views.SourceEditorView view: The GTK view showing the data as a table
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
        self.connections_ctrl = StateConnectionsEditorController(model, view['connections_view'])

        self.new_ip_counter = 0
        self.new_op_counter = 0
        self.new_sv_counter = 0

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        view['new_input_port_button'].connect('clicked', self.on_new_input_port_button_clicked)
        view['new_output_port_button'].connect('clicked', self.on_new_output_port_button_clicked)
        view['new_scoped_variable_button'].connect('clicked', self.on_new_scoped_variable_button_clicked)
        #view['new_outcome_button'].connect('clicked', self.outcomes_ctrl.on_add)

        view['delete_input_port_button'].connect('clicked', self.on_delete_input_port_button_clicked)
        view['delete_output_port_button'].connect('clicked', self.on_delete_output_port_button_clicked)
        view['delete_scoped_variable_button'].connect('clicked', self.on_delete_scoped_variable_button_clicked)
        #view['delete_outcome_button'].connect('clicked', self.outcomes_ctrl.on_remove)
        #view['entry_name'].connect('focus-out-event', self.change_name)
        #view['entry_name'].set_text(self.model.state.name)

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
        print "pathremove: %s" % path
        if path is not None:
            key = self.model.input_data_port_list_store[int(path[0])][0].name
            print "remove: %s" % key
            self.model.state.remove_input_data_port(key)

    def on_delete_output_port_button_clicked(self, widget, data=None):
        tree_view = self.view['outputs_view']["output_ports_tree_view"]
        path = tree_view.get_cursor()[0]
        print "pathremove: %s" % path
        if path is not None:
            key = self.model.output_data_port_list_store[int(path[0])][0].name
            print "remove: %s" % key
            self.model.state.remove_output_data_port(key)

    def on_delete_scoped_variable_button_clicked(self, widget, data=None):
        tree_view = self.view['scopes_view']["scoped_variables_tree_view"]
        path = tree_view.get_cursor()[0]
        print "pathremove: %s" % path
        if path is not None:
            key = self.model.scoped_variables_list_store[int(path[0])][0].name
            print "remove: %s" % key
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