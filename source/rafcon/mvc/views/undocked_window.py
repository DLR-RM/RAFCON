from gtkmvc import View
from rafcon.mvc.views.top_tool_bar import TopToolBarView

class UndockedWindowView(View):
    """

    """
    builder = './glade/undocked_window.glade'
    top = 'undock_window'

    def __init__(self, title):
        View.__init__(self)

        self.top_tool_bar = TopToolBarView()
        self['top_menu_hbox'].remove(self['top_tool_bar_placeholder'])
        self['top_menu_hbox'].pack_end(self.top_tool_bar.get_top_widget(), expand=True, fill=True, padding=0)
        self['top_menu_hbox'].reorder_child(self.top_tool_bar.get_top_widget(), 1)
        self.get_top_widget().set_decorated(False)
        self.get_top_widget().set_title(title)
