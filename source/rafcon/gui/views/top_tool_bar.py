import gtk
from gtkmvc import View
from rafcon.gui.utils import constants
from rafcon.gui import gui_helper


class TopToolBarView(View):
    builder = constants.get_glade_path("top_tool_bar.glade")
    top = 'top_toolbar'

    def __init__(self):
        View.__init__(self)

        self.get_top_widget().set_events(gtk.gdk.POINTER_MOTION_MASK | gtk.gdk.POINTER_MOTION_HINT_MASK |
                                         gtk.gdk.BUTTON_PRESS_MASK)

        close_button = self['close_button']
        maximize_button = self['maximize_button']
        minimize_button = self['minimize_button']
        redock_button = self['redock_button']

        close_label = gui_helper.create_button_label(constants.BUTTON_CLOSE)
        close_button.set_label_widget(close_label)

        maximize_label = gui_helper.create_button_label(constants.BUTTON_EXP)
        maximize_button.set_label_widget(maximize_label)

        minimize_button.set_label('_')

        redock_label = gui_helper.create_button_label(constants.BUTTON_UNDOCK)
        redock_button.set_label_widget(redock_label)
