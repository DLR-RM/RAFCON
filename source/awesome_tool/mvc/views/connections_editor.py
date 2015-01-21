import gtk
from gtkmvc import View

from mvc.views.transition_list import TransitionListView
from mvc.views.data_flow_list import DataFlowListView


class StateConnectionsEditorView(View):

    builder = './glade/connections_editor_widget.glade'
    top = 'connections_editor_widget'

    def __init__(self):
        View.__init__(self)

        self.transitions_view = TransitionListView()
        self.dataflows_view = DataFlowListView()

        self['transitions_scroller'].add(self.transitions_view.get_top_widget())
        self['dataflows_scroller'].add(self.dataflows_view.get_top_widget())

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