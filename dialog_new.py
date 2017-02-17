import gtk

from rafcon.gui.utils import constants
from rafcon.utils import log
logger = log.get_logger(__name__)


class RAFCONMessageDialog(gtk.MessageDialog):

    def __init__(self, callback=None, callback_args=(),
                 markup_text=None,
                 type=gtk.MESSAGE_INFO, flags=gtk.DIALOG_MODAL, parent=None, buttons=gtk.BUTTONS_OK):

        super(RAFCONMessageDialog, self).__init__(type=type, buttons=buttons, flags=flags)

        if parent:
            super(RAFCONMessageDialog, self).set_transient_for(parent)

        from cgi import escape
        super(RAFCONMessageDialog, self).set_markup(escape(markup_text))

        if callback:
            self.add_callback(callback, *callback_args)

        self.show_all()
        self.grab_focus()
        self.run() if callback else None

    def add_callback(self, callback, *args):
        self.connect('response', callback, *args)


class RAFCONButtonDialog(RAFCONMessageDialog):

    def __init__(self, callback=None, callback_args=(),
                 markup_text=None, button_texts=None,
                 type=gtk.MESSAGE_INFO, flags=gtk.DIALOG_MODAL, parent=None):

        super(RAFCONButtonDialog, self).__init__(callback, callback_args, markup_text, type, flags, parent, buttons=gtk.BUTTONS_NONE)

        hbox = gtk.HBox(homogeneous=False, spacing=constants.GRID_SIZE)
        for index, button in enumerate(button_texts):
                button = gtk.Button(button)
                button.connect('clicked', self.on_clicked, index)
                hbox.pack_start(button, True, True, 1)

        align_action_area = gtk.Alignment(xalign=1, yalign=0.0, xscale=0.0, yscale=0.0)

        align_action_area.add(hbox)
        self.get_action_area().get_parent().pack_end(align_action_area)

        self.show_all()
        self.run() if callback else None

    def on_clicked(self, widget, index):
        self.response(index)


class RAFCONInputDialog(RAFCONButtonDialog):

    def __init__(self, callback=None, callback_args=(),
                 markup_text=None, button_texts=None, checkbox_text=None,
                 type=gtk.MESSAGE_INFO, flags=gtk.DIALOG_MODAL, parent=None):

        super(RAFCONInputDialog, self).__init__(callback, callback_args, markup_text, button_texts, type, flags, parent)

        # Setup new text entry line
        self.entry = gtk.Entry()
        self.entry.set_editable(1)
        self.entry.set_activates_default(True)
        self.entry.set_width_chars(10)

        self.entry.connect('activate', lambda w: self.response(0))

        vbox = self.get_content_area()
        # create a new gtk.Hbox to put in the checkbox and the entry
        hbox = gtk.HBox(homogeneous=False, spacing=constants.GRID_SIZE)
        # add the hbox to the content area
        vbox.add(hbox)

        if checkbox_text:
            # If a checkbox is asked for by the caller, create one.
            self.check = gtk.CheckButton(checkbox_text)
            hbox.pack_end(self.check, True, True, 1)

        hbox.pack_end(self.entry, True, True, 1)

        self.show_all()
        self.run() if callback else None

    def get_entry(self):
        return self.entry.get_text()

    def get_checkbox_state(self):
        return bool(self.check.get_state())


class RAFCONColumnCheckBoxDialog(RAFCONButtonDialog):

    def __init__(self, callback=None, callback_args=(),
                 markup_text=None, button_texts=None, checkbox_texts=None,
                 type=gtk.MESSAGE_INFO, flags=gtk.DIALOG_MODAL, parent=None):

        super(RAFCONColumnCheckBoxDialog, self).__init__(callback, callback_args, markup_text, button_texts, type, flags, parent)

        vbox = self.get_content_area()
        checkbox_vbox = gtk.VBox(homogeneous=False, spacing=constants.GRID_SIZE)
        vbox.add(checkbox_vbox)
        self.checkboxes = []
        for checkbox in checkbox_texts:
            self.checkboxes.append(gtk.CheckButton(checkbox))

        for checkbox in self.checkboxes:
            checkbox_vbox.pack_start(checkbox, True, True, 1)

        self.show_all()
        self.run() if callback else None

    def get_checkbox_state_by_name(self, checkbox_text):
        pass

    def get_checkbox_state_by_index(self, checkbox_index):
        return bool(self.checkboxes[checkbox_index].get_state())

    def get_checkboxes(self):
        return self.checkboxes

# TODO: Rico, please put your checbox tree dialog here, i dont want to do it and "claim" the code by myself

# class RAFCONMultiColumnCheckBoxDialog(RAFCONButtonDialog):
#
#     def __init__(self, callback=None, callback_args=(), markup_text=None, button_texts=None, checkbox_texts=None, num_columns=1, titles=None, type=gtk.MESSAGE_INFO, flags=gtk.DIALOG_MODAL, parent=None, autorun=False):
#
#         super(RAFCONMultiColumnCheckBoxDialog, self).__init__(callback, callback_args, markup_text, button_texts, type, flags, parent)
#         self.rows = []
#         vbox = self.get_content_area()
#         for checkbox_row in checkbox_texts:
#             row = []
#             row.append(gtk.HBox(homogeneous=False, spacing=constants.GRID_SIZE))
#             self.rows.append(row)
#             row.append(gtk.CheckButton(checkbox_row))
#             row[1].show()
#             row[0].pack_end(row[1], True, True, 1)
#             for x in xrange(num_columns-1):
#                 row.append(gtk.CheckButton())
#                 row[x+2].show()
#                 row[0].pack_start(row[x+2], True, True, 1)
#             row[0].show()
#
#         for row in self.rows:
#             vbox.add(row[0])
#
#         vbox.show()
#
#         self.show()
#         self.grab_focus()
#         self.run() if autorun else None

