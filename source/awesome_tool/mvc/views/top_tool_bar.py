from gtkmvc import View
import gtk
from awesome_tool.utils import constants


class TopToolBarView(View):
    builder = './glade/top_tool_bar.glade'
    top = 'top_toolbar'

    def __init__(self):
        View.__init__(self)

        self.get_top_widget().set_events(gtk.gdk.POINTER_MOTION_MASK | gtk.gdk.POINTER_MOTION_HINT_MASK |
                                         gtk.gdk.BUTTON_PRESS_MASK)

        cb = self['close_button']
        maxb = self['maximize_button']
        minb = self['minimize_button']

        label = gtk.Label()
        label.set_markup('<span font_desc="%s %s">&#x%s;</span>' % (constants.ICON_FONT,
                                                                    constants.FONT_SIZE_BIG,
                                                                    constants.BUTTON_CLOSE))  # close button sign
        cb.set_label_widget(label)

        label = gtk.Label()
        label.set_markup('<span font_desc="%s %s">&#x%s;</span>' % (constants.ICON_FONT,
                                                                      constants.FONT_SIZE_BIG,
                                                                      constants.BUTTON_EXP))  # expand button sign
        maxb.set_label_widget(label)

        minb.set_label("_")