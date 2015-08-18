import gtk
from gtkmvc import View

from rafcon.mvc.views.state_transitions import StateTransitionsEditorView
from rafcon.mvc.views.state_data_flows import StateDataFlowsEditorView


class StateConnectionsEditorView(View):

    builder = './glade/connections_editor_widget.glade'
    top = 'connections_editor_widget'

    def __init__(self):
        View.__init__(self)

        self.transitions_view = StateTransitionsEditorView()
        self.data_flows_view = StateDataFlowsEditorView()

        self['transitions_viewport'].add(self.transitions_view.get_top_widget())
        self['data_flows_viewport'].add(self.data_flows_view.get_top_widget())

    pass
