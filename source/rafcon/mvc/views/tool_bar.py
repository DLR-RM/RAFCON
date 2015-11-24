import gtk
from gtkmvc import View
from rafcon.utils import constants
from rafcon.mvc.config import global_gui_config as gui_config


class ToolBarView(View):
    builder = './glade/tool_bar.glade'
    top = 'toolbar'

    def __init__(self):
        View.__init__(self)

        button_new = self['button_new']
        button_new.set_label_widget(self.get_label_widget(constants.BUTTON_NEW, "New Statemachine"))

        button_refresh = self['button_refresh']
        button_refresh.set_label_widget(self.get_label_widget(constants.BUTTON_REFR, "Refresh"))

        button_open = self['button_open']
        button_open.set_label_widget(self.get_label_widget(constants.BUTTON_OPEN, "Open Statemachine"))

        button_save = self['button_save']
        button_save.set_label_widget(self.get_label_widget(constants.BUTTON_SAVE, "Save Statemachine"))

        button_refresh_libs = self['button_refresh_libs']
        button_refresh_libs.set_label_widget(self.get_label_widget(constants.BUTTON_REFR, "Refresh Libraries"))

        self.get_top_widget().set_border_width(constants.BORDER_WIDTH)

    @staticmethod
    def get_label_widget(icon, text):
        hbox = gtk.HBox()

        icon_label = gtk.Label()
        icon_label.set_markup('<span font_desc="%s %s">&#x%s;</span>' % (constants.ICON_FONT,
                                                                         constants.FONT_SIZE_NORMAL,
                                                                         icon))
        icon_label.show()
        hbox.pack_start(icon_label, False, True, 2)

        text_label = gtk.Label()
        text_label.set_markup('<span font_desc="%s %s" letter_spacing="%s">%s</span>' %
                              (constants.FONT_NAMES[0], constants.FONT_SIZE_NORMAL,
                               constants.LETTER_SPACING_075PT, text))
        text_label.show()
        hbox.pack_start(text_label, True, True, 2)

        hbox.show()
        return hbox
