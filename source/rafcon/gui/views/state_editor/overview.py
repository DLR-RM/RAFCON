from gtkmvc import View

from rafcon.gui.utils import constants


class StateOverviewView(View):
    builder =  constants.get_glade_path("state_overview_widget.glade")
    top = 'properties_widget_bg'

    def __init__(self):
        View.__init__(self)

        self['properties_widget'].set_border_width(constants.GRID_SIZE * 2)
        # self['is_start_state_checkbutton'].set_border_width(constants.BUTTON_BORDER_WIDTH)
