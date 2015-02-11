import gtk
from gtkmvc import View


class StateOverviewView(View):
    builder = './glade/state_overview_widget.glade'
    top = 'properties_widget'

    def __init__(self):
        View.__init__(self)
