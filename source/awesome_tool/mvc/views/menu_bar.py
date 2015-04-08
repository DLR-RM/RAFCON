from gtkmvc import View
import gtk
import pango
from utils import constants


class MenuBarView(View):
    builder = './glade/menu_bar.glade'
    top = 'menubar'

    buttons = {
        # File
        'new': "f016",
        'open': "f115",
        'save': "f0c7",
        'save_as': "f0c7",
        'menu_properties': "f0ad",
        'refresh_all': "f021",
        'refresh_libraries': "f021",
        'quit': "f08b",
        # Edit
        'cut_selection': "f0c4",
        'copy_selection': "f0c5",
        'paste_clipboard': "f0ea",
        'delete': "f1f8",
        'undo': "f0e2",
        'redo': "f01e",
        # Execution
        'start': "f04b",
        'pause': "f04c",
        'stop': "f04d",
        'step': "f051",
        'backward_step_mode': "f048"
        }

    def __init__(self, top_window):
        View.__init__(self)

        self.win = top_window['main_window']

        for key in self.buttons.viewkeys():
            self.set_image_for_button(key)

    def set_image_for_button(self, button_name):
        button = self[button_name]

        layout = pango.Layout(gtk.TextView().get_pango_context())

        pixmap, mask = gtk.gdk.pixmap_create_from_xpm_d(self.win.window,
                                                        self.win.get_style().bg[gtk.STATE_NORMAL],
                                                        self.create_empty_pixbuf_xpm(constants.ICON_SIZE_IN_PIXEL,
                                                                                     constants.ICON_SIZE_IN_PIXEL))

        layout.set_markup('<span fgcolor="%s" font_desc="%s %s">&#x%s;</span>' %
                          (constants.TEXT_COLOR,
                           constants.DEFAULT_FONT,
                           constants.FONT_SIZE_NORMAL,
                           self.buttons[button_name]))

        pixmap.draw_layout(pixmap.new_gc(), 2, 2, layout)
        mask.draw_layout(mask.new_gc(), 2, 2, layout)

        button.set_image(gtk.image_new_from_pixmap(pixmap, mask))

    def create_empty_pixbuf_xpm(self, width, height):
        xpm = ["%i %i 1 1" % (width, height), "  c None"]
        for i in range(0, height):
            xpm.append(" " * width)
        return xpm