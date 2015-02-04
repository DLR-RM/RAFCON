import gtk
from gtkmvc import View

from mvc.views.state_transitions import StateTransitionsListView,StateTransitionsEditorView
from mvc.views.state_data_flows import StateDataFlowsListView,StateDataFlowsEditorView


class StateConnectionsEditorView(View):

    builder = './glade/connections_editor_widget.glade'
    top = 'connections_editor_widget'

    def __init__(self):
        View.__init__(self)

        # self.transitions_view = StateTransitionsListView()
        # self.data_flows_view = StateDataFlowsListView()
        self.transitions_view = StateTransitionsEditorView()
        self.data_flows_view = StateDataFlowsEditorView()

        self['transitions_viewport'].add(self.transitions_view.get_top_widget())
        self['data_flows_viewport'].add(self.data_flows_view.get_top_widget())

    pass


if __name__ == '__main__':
    from mvc.controllers import StateConnectionsEditorController, SingleWidgetWindowController
    from mvc.views import SingleWidgetWindowView

    import mvc.main as main

    main.setup_path()
    main.check_requirements()
    [ctr_model, logger, ctr_state, gvm_model, emm_model] = main.create_models()

    v = SingleWidgetWindowView(StateConnectionsEditorView, width=500, height=200, title='Connection Editor')
    c = SingleWidgetWindowController(ctr_model, v, StateConnectionsEditorController)

    gtk.main()