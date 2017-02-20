import gtk

from rafcon.gui.utils import constants
from rafcon.utils import log
logger = log.get_logger(__name__)


class RAFCONMessageDialog(gtk.MessageDialog):

    def __init__(self, callback=None, callback_args=(),
                 markup_text=None,
                 type=gtk.MESSAGE_INFO, flags=gtk.DIALOG_MODAL, parent=None, buttons=gtk.BUTTONS_OK,
                 standalone=False):

        super(RAFCONMessageDialog, self).__init__(type=type, buttons=buttons, flags=flags)

        if parent:
            super(RAFCONMessageDialog, self).set_transient_for(parent)

        from cgi import escape
        super(RAFCONMessageDialog, self).set_markup(escape(markup_text))

        if callback:
            self.add_callback(callback, *callback_args)

        self.show_all()
        # Only grab focus in the highest class, the inheriting classes should have the focus as well because they all
        # execute the init of this class
        self.grab_focus()

        # Run by yourself if callbacks exist. If not,
        # it doesnt make sense because the window can't be destroyed properly
        self.run() if standalone else None

    def add_callback(self, callback, *args):
        self.connect('response', callback, *args)


class RAFCONButtonDialog(RAFCONMessageDialog):

    def __init__(self, callback=None, callback_args=(),
                 markup_text=None, button_texts=None,
                 type=gtk.MESSAGE_INFO, flags=gtk.DIALOG_MODAL, parent=None,
                 standalone=False):

        super(RAFCONButtonDialog, self).__init__(callback, callback_args, markup_text, type, flags, parent, buttons=gtk.BUTTONS_NONE)

        hbox = gtk.HBox(homogeneous=False, spacing=constants.GRID_SIZE)
        for index, button in enumerate(button_texts):
                button = gtk.Button(button)
                button.connect('clicked', self.add_response, index)
                hbox.pack_start(button, True, True, 1)

        # alignment area to resize the buttons to their label size
        align_action_area = gtk.Alignment(xalign=1, yalign=0.0, xscale=0.0, yscale=0.0)

        align_action_area.add(hbox)
        self.get_action_area().get_parent().pack_end(align_action_area)

        self.show_all()
        self.run() if standalone else None

    def add_response(self, widget, index):
        # add responses to the 'clicked' event of the button. First button gets 1 as response Second 2 etc.
        self.response(index+1)


class RAFCONInputDialog(RAFCONButtonDialog):

    def __init__(self, callback=None, callback_args=(),
                 markup_text=None, button_texts=None, checkbox_text=None,
                 type=gtk.MESSAGE_INFO, flags=gtk.DIALOG_MODAL, parent=None,
                 standalone=False):

        super(RAFCONInputDialog, self).__init__(callback, callback_args, markup_text, button_texts, type, flags, parent)

        # create a new gtk.Hbox to put in the checkbox and the entry
        hbox = gtk.HBox(homogeneous=False, spacing=constants.GRID_SIZE)
        self.get_content_area().add(hbox)

        # Setup new text entry line
        self.entry = gtk.Entry()
        self.entry.set_editable(1)
        self.entry.set_activates_default(True)
        self.entry.set_width_chars(10)

        # Hitting the enter button responds 1 from the widget
        # This is the same as the first button, so the first button should always be sth. approving the content of the
        # window. Probably a configurable flag would also make sense.
        self.entry.connect('activate', self.add_response, 0)
        hbox.pack_start(self.entry, True, True, 1)

        if isinstance(checkbox_text, str):
            # If a checkbox_text is specified by the caller, we can assume that one should be used.
            self.check = gtk.CheckButton(checkbox_text)
            hbox.pack_end(self.check, True, True, 1)

        self.show_all()
        self.run() if standalone else None

    def get_entry(self):
        return self.entry.get_text()

    def get_checkbox_state(self):
        return bool(self.check.get_state())


class RAFCONColumnCheckBoxDialog(RAFCONButtonDialog):

    def __init__(self, callback=None, callback_args=(),
                 markup_text=None, button_texts=None, checkbox_texts=None,
                 type=gtk.MESSAGE_INFO, flags=gtk.DIALOG_MODAL, parent=None,
                 standalone=False):

        super(RAFCONColumnCheckBoxDialog, self).__init__(callback, callback_args, markup_text, button_texts, type, flags, parent)

        checkbox_vbox = gtk.VBox(homogeneous=False, spacing=constants.GRID_SIZE)
        self.get_content_area().add(checkbox_vbox)
        # this is not really needed i guess if I can get the checkboxes over the content area anyway
        # TODO change this to a solution without the list.
        self.checkboxes = []
        for index, checkbox in enumerate(checkbox_texts):
            self.checkboxes.append(gtk.CheckButton(checkbox))
            checkbox_vbox.pack_start(self.checkboxes[index], True, True, 1)

        self.show_all()
        self.run() if standalone else None

    def get_checkbox_state_by_name(self, checkbox_text):
        return [bool(checkbox.get_state()) for checkbox in self.checkboxes if checkbox.get_label() == checkbox_text]

    def get_checkbox_state_by_index(self, checkbox_index):
        return bool(self.checkboxes[checkbox_index].get_state())

    def get_checkboxes(self):
        return self.checkboxes

# TODO: Rico, please put your checkbox tree dialog here, i don't want to do it and "claim" the code by myself :)
