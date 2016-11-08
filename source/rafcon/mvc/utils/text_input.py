import gtk
from enum import Enum

from rafcon.mvc.utils import constants


class ButtonDialog(Enum):
    __order__ = "OPTION_1 OPTION_2 OPTION_3 OPTION_4 OPTION_5"
    OPTION_1 = 1
    OPTION_2 = 2
    OPTION_3 = 3
    OPTION_4 = 4
    OPTION_5 = 5


class RAFCONText(gtk.Entry):

    def __init__(self, parent=None):

        if parent:
            self.set_transient_for(parent)

        window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        window.set_size_request(200, 100)
        window.set_title("GTK Entry")
        window.connect("delete_event", lambda w,e: gtk.main_quit())

        vbox = gtk.VBox(False, 0)
        window.add(vbox)
        vbox.show()

        entry = gtk.Entry()
        entry.set_max_length(50)
        entry.connect("activate", self.enter_callback, entry)
        entry.set_text("hello")
        entry.insert_text(" world", len(entry.get_text()))
        entry.select_region(0, len(entry.get_text()))
        vbox.pack_start(entry, True, True, 0)
        entry.show()

        hbox = gtk.HBox(False, 0)
        vbox.add(hbox)
        hbox.show()
        check = gtk.CheckButton("Editable")
        hbox.pack_start(check, True, True, 0)
        check.connect("toggled", self.entry_toggle_editable, entry)
        check.set_active(True)
        check.show()

        check = gtk.CheckButton("Visible")
        hbox.pack_start(check, True, True, 0)
        check.connect("toggled", self.entry_toggle_visibility, entry)
        check.set_active(True)
        check.show()

        button = gtk.Button(stock=gtk.STOCK_CLOSE)
        button.connect("clicked", lambda w: gtk.main_quit())
        vbox.pack_start(button, True, True, 0)
        button.set_flags(gtk.CAN_DEFAULT)
        button.grab_default()
        button.show()
        window.show()

    def set_markup(self, markup_text):
        from cgi import escape
        super(RAFCONText, self).set_markup(escape(markup_text))

    def finalize(self, callback, *args):
        self.connect('response', callback, *args)

        # Use HBox instead of ButtonBox to align buttons within the action area
        button_box = self.get_action_area()

        vbox = button_box.get_parent()
        hbox = gtk.HBox(homogeneous=False, spacing=constants.GRID_SIZE)
        buttons = [button for button in button_box.get_children()]
        for button in buttons:
            button_box.remove(button)
            button.set_relief(gtk.RELIEF_NORMAL)
            hbox.pack_end(button)
        vbox.remove(button_box)
        align_action_area = gtk.Alignment(xalign=1, yalign=0.0, xscale=0.0, yscale=0.0)
        align_action_area.add(hbox)
        vbox.pack_end(align_action_area)
        align_action_area.show()
        hbox.show()
        self.show()


class RAFCONTextDialog(RAFCONText):
    def __init__(self, markup_text, button_texts, callback, callback_args=(), type=gtk.MESSAGE_INFO, parent=None,
                 width=None):
        super(RAFCONTextDialog, self).__init__(type, gtk.BUTTONS_NONE, gtk.DIALOG_MODAL, parent)
        if isinstance(width, int):
            hbox = self.get_action_area()
            vbox = hbox.parent
            msg_ctr = vbox.get_children()[0]
            text_ctr = msg_ctr.get_children()[1]
            text_ctr.get_children()[0].set_size_request(width, -1)
            text_ctr.get_children()[1].set_size_request(width, -1)
        self.set_markup(markup_text)
        for button_text, option in zip(button_texts, ButtonDialog):
            self.add_button(button_text, option.value)
        self.finalize(callback, *callback_args)
        self.grab_focus()
        self.run()

