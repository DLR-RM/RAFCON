from gtkmvc import View
import gtk


class TopToolBarView(View):
    builder = './glade/top_tool_bar.glade'
    top = 'top_toolbar'

    def __init__(self):
        View.__init__(self)

        self.get_top_widget().set_events(gtk.gdk.POINTER_MOTION_MASK | gtk.gdk.POINTER_MOTION_HINT_MASK |
                                         gtk.gdk.BUTTON_PRESS_MASK)