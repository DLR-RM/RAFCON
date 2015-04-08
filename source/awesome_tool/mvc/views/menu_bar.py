from gtkmvc import View
import gtk
import pango
from awesome_tool.utils import constants


class MenuBarView(View):
    builder = './glade/menu_bar.glade'
    top = 'menubar'

    buttons = {
        # File
        'new': constants.BUTTON_NEW,
        'open': constants.BUTTON_OPEN,
        'save': constants.BUTTON_SAVE,
        'save_as': constants.BUTTON_SAVE,
        'menu_properties': constants.BUTTON_PROP,
        'refresh_all': constants.BUTTON_REFR,
        'refresh_libraries': constants.BUTTON_REFR,
        'quit': constants.BUTTON_QUIT,
        # Edit
        'cut_selection': constants.BUTTON_CUT,
        'copy_selection': constants.BUTTON_COPY,
        'paste_clipboard': constants.BUTTON_PASTE,
        'add_state': constants.BUTTON_ADD,
        'group_states': constants.BUTTON_GROUP,
        'ungroup_states': constants.BUTTON_UNGR,
        'delete': constants.BUTTON_DEL,
        'undo': constants.BUTTON_UNDO,
        'redo': constants.BUTTON_REDO,
        # View
        'expert_view': constants.BUTTON_VIEW,
        # Execution
        'start': constants.BUTTON_START,
        'pause': constants.BUTTON_PAUSE,
        'stop': constants.BUTTON_STOP,
        'step_mode': constants.BUTTON_STEPM,
        'step': constants.BUTTON_STEP,
        'backward_step_mode': constants.BUTTON_BACKW,
        # Help
        'about': constants.BUTTON_ABOUT
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