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

    def reset_title(self, title, notebook):
        """Triggered whenever a notebook tab is switched in the left bar.

        Resets the title of the un-docked window to the format 'upper_open_tab / lower_open_tab'

        :param title: The name of the newly selected tab
        :param notebook: string taking one of two values 'up' or 'down' indicating which notebook was changed
        """
        current_title = self.get_top_widget().get_title()
        title_up = current_title.split('/')[0]
        title_down = current_title.split('/')[1]
        if notebook == 'up':
            new_title = title + ' / ' + title_down
        else:
            new_title = title_up + ' / ' + title
        self.get_top_widget().set_title(new_title)
