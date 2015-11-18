from gtkmvc import View
from rafcon.utils import constants


class StateOverviewView(View):
    builder = './glade/state_overview_widget.glade'
    top = 'properties_widget'

    def __init__(self):
        View.__init__(self)

        self['properties_widget'].set_border_width(constants.BORDER_WIDTH)
        self['is_start_state_checkbutton'].set_border_width(constants.BUTTON_BORDER_WIDTH)
