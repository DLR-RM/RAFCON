from gtkmvc import View

from rafcon.mvc import gui_helper
from rafcon.mvc.utils import constants


class StateTransitionsListView(View):
    builder = constants.get_glade_path("transition_list_widget.glade")
    top = 'tree_view'

    def __init__(self):
        View.__init__(self)
        self.tree_view = self['tree_view']

    def get_top_widget(self):
        return self.tree_view


class StateTransitionsEditorView(View):
    builder = constants.get_glade_path("state_transitions_widget.glade")
    top = 'vbox2'

    def __init__(self):
        View.__init__(self)

        gui_helper.set_label_markup(self['transitions_label'], 'TRANSITIONS',
                                    letter_spacing=constants.LETTER_SPACING_1PT)

        self.transitions_listView = StateTransitionsListView()
        self['transitions_scroller'].add(self.transitions_listView.get_top_widget())

        self['internal_t_checkbutton'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['connected_to_t_checkbutton'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['add_t_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['remove_t_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
