from gtkmvc import View

from rafcon.mvc.utils import constants
from rafcon.mvc import gui_helper


class StateDataFlowsListView(View):
    builder = './glade/data_flow_list_widget.glade'
    top = 'tree_view'

    def __init__(self):
        View.__init__(self)
        self.tree_view = self['tree_view']

    def get_top_widget(self):
        return self.tree_view


class StateDataFlowsEditorView(View):
    builder = './glade/state_data_flows_widget.glade'
    top = 'data_flows_container'

    def __init__(self):
        View.__init__(self)

        gui_helper.set_label_markup(self['data_flows_label'], 'DATA FLOWS', letter_spacing=constants.LETTER_SPACING_1PT)

        self.data_flows_listView = StateDataFlowsListView()
        self['dataflows_scroller'].add(self.data_flows_listView.get_top_widget())

        self['internal_d_checkbutton'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['connected_to_d_checkbutton'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['add_d_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['remove_d_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
