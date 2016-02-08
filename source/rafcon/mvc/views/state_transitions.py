from gtkmvc import View
from rafcon.mvc.utils import constants


class StateTransitionsListView(View):
    builder = './glade/transition_list_widget.glade'
    top = 'tree_view'

    def __init__(self):
        View.__init__(self)
        self.tree_view = self['tree_view']

    def get_top_widget(self):
        return self.tree_view


class StateTransitionsEditorView(View):
    builder = './glade/state_transitions_widget.glade'
    top = 'vbox2'

    def __init__(self):
        View.__init__(self)
        self.transitions_listView = StateTransitionsListView()
        self['transitions_scroller'].add(self.transitions_listView.get_top_widget())

        self['internal_t_checkbutton'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['connected_to_t_checkbutton'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['add_t_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['remove_t_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
