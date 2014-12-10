
from gtkmvc import View

from views.transition_list import TransitionListView

class ContainerStateView(View):

    builder = './glade/ContainerStateWidget.glade'
    top = 'container_state_widget'

    def __init__(self):
        View.__init__(self)

        self.transition_list_view = TransitionListView()
        # for i in iter(self.transition_list_view):
        #     print i
       # print "top", self.transition_list_view.get_top_widget()
        self['transition_scroller'].add(self.transition_list_view.get_top_widget())
        #self.show()

    pass