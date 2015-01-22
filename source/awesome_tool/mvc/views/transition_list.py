
from gtkmvc import View

class TransitionListView(View):
    builder = './glade/TransitionListWidget.glade'
    top = 'transition_list_view'

    def __init__(self):
        View.__init__(self)

    pass