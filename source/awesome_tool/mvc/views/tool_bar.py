from gtkmvc import View
import gtk


class ToolBarView(View):
    builder = './glade/tool_bar.glade'
    top = 'toolbar'

    button_font = "FontAwesome 12"

    def __init__(self):
        View.__init__(self)

        button_new = self['button_new']
        button_new.set_label_widget(self.get_label_widget("f016", "New Statemachine"))

        button_refresh = self['button_refresh']
        button_refresh.set_label_widget(self.get_label_widget("f021", "Refresh"))

        button_open = self['button_open']
        button_open.set_label_widget(self.get_label_widget("f115", "Open"))

        button_save = self['button_save']
        button_save.set_label_widget(self.get_label_widget("f0c7", "Save Statemachine"))

        button_refresh_libs = self['button_refresh_libs']
        button_refresh_libs.set_label_widget(self.get_label_widget("f021", "Refresh Libraries"))

    def get_label_widget(self, icon, text):
        hbox = gtk.HBox()

        icon_label = gtk.Label()
        icon_label.set_markup('<span font_desc="%s">&#x%s;</span>' % (self.button_font, icon))
        icon_label.show()
        hbox.pack_start(icon_label, False, True, 2)

        text_label = gtk.Label(text)
        text_label.show()
        hbox.pack_start(text_label, True, True, 2)

        hbox.show()
        return hbox