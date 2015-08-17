
from gtkmvc import View

from rafcon.mvc.views.state_transitions import StateTransitionsListView
from rafcon.mvc.views.state_data_flows import StateDataFlowsListView


class ContainerStateView(View):

    builder = './glade/container_state_widget_frame.glade'
    top = 'container_state_widget'

    def __init__(self):
        View.__init__(self)

        self.transition_list_view = StateTransitionsListView()
        self.data_flow_list_view = StateDataFlowsListView()

        self['transition_scroller'].add(self.transition_list_view.get_top_widget())
        self['data_flow_scroller'].add(self.data_flow_list_view.get_top_widget())

    pass