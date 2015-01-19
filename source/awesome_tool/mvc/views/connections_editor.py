
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