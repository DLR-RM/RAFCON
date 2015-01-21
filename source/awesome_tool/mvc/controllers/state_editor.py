import gtk
from gtkmvc import View, Controller
from mvc.controllers.source_editor import SourceEditorController
from mvc.controllers.container_state import ContainerStateController
from mvc.controllers.connections_editor import StateConnectionsEditorController
from mvc.controllers.state_overview import StateOverviewController

from mvc.controllers import DataPortListController
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