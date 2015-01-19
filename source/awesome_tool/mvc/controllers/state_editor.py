import gtk
from gtkmvc import View, Controller
from mvc.controllers.source_editor import SourceEditorController
from mvc.controllers.container_state import ContainerStateController
from mvc.controllers.connections_editor import StateConnectionsEditorController
from mvc.controllers.state_overview import StateOverviewController

from mvc.controllers.input_data_port_list import DataPortListController
from gtkmvc import Observer


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
        #self.scopes_ctrl = ContainerStateController(model, view['scopes_view'])
        self.outcomes_ctrl = ContainerStateController(model, view['outcomes_view'])
        self.source_ctrl = SourceEditorController(model, view['source_view'])
        self.connections_ctrl = StateConnectionsEditorController(model, view['connections_view'])

    def register_view(self, view):
        """Called when the View was registered

        Can be used e.g. to connect signals. Here, the destroy signal is connected to close the application
        """
        #view['entry_name'].connect('focus-out-event', self.change_name)
        #view['entry_name'].set_text(self.model.state.name)
        view['port_expander'].connect('size-request', self.resize_port_widget)
        view['port_expander1'].connect('size-request', self.resize_port_widget)
        view['port_expander2'].connect('size-request', self.resize_port_widget)
        view['port_expander3'].connect('size-request', self.resize_port_widget)
        view['port_expander4'].connect('size-request', self.resize_port_widget)
        view['connections_expander'].connect('size-request', self.resize_connections_widget)
        view['connections_view']['transitions_expander'].connect('size-request', self.resize_connections_widget)
        view['connections_view']['dataflow_expander'].connect('size-request', self.resize_connections_widget)
        view.get_top_widget().connect('destroy', gtk.main_quit)

    def register_adapters(self):
        """Adapters should be registered in this method call

        Each property of the state should have its own adapter, connecting a label in the View with the attribute of
        the State.
        """
        #self.adapt(self.__state_property_adapter("name", "input_name"))

    #TODO: separate functions for inputs and outputs
    @Observer.observe("state", after=True)
    def assign_notification_state(self, model, prop_name, info):
        print "call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
              (prop_name, info.instance, info.method_name, info.result)
        model.update_input_data_port_list_store()

    def resize_port_widget(self, expander, x):

        if self.view['port_expander'].get_expanded():
            count = 0
            for i in range(1, 5):
                if self.view['port_expander'+str(i)].get_expanded():
                    print "%s is expanded" % ('port_expander'+str(i))
                    count += 1
            self.view['port_expander'].set_size_request(width=-1, height=count*150+100)
        else:
            self.view['port_expander'].set_size_request(width=-1, height=-1)

    def resize_connections_widget(self, expander, x):

        if self.view['connections_expander'].get_expanded():
            count = 0
            for expand_id in ['transitions_expander', 'dataflows_expander']:
                if self.view['connections_view'][expand_id].get_expanded():
                    print "%s is expanded" % expand_id
                    count += 1
            self.view['connections_expander'].set_size_request(width=-1, height=count*150+75)
            self.view['vpaned1'].set_position(1000)
        else:
            self.view['connections_expander'].set_size_request(width=-1, height=-1)
            self.view['vpaned1'].set_position(1000)
            #print "position: %s" % self.view['vpaned1'].get_position()


if __name__ == '__main__':
    from statemachine.states.execution_state import ExecutionState as State
    from statemachine.states.container_state import ContainerState

    from mvc.models import StateModel, ContainerStateModel
    from mvc.views.state_editor import StateEditorView
    #from mvc.controllers.state_editor import StateEditorController

    state1 = State('Rico2')
    m = StateModel(state1)

    #w = gtk.Window()
    v = StateEditorView()
    c = StateEditorController(m, v)
    #w.add(v.get_top_widget())#['main_frame'])
    #w.show_all()

    gtk.main()