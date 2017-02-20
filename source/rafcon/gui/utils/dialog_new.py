import gtk

from rafcon.gui.utils import constants
from rafcon.utils import log
logger = log.get_logger(__name__)


class RAFCONMessageDialog(gtk.MessageDialog):
    """A dialog which consists of a gtk button and a markup text. This can be used for informing the user about
    important things happening

    :param callback: A callback function which should be executed on the end of the run() method
    :param callback_args: Arguments passed to the callback function
    :param markup_text: The text inside the dialog
    :param type: The gtk type of the dialog, e.g. gtk.MESSAGE_INFO, gtk.MESSAGE_QUESTION etc.
    :param flags: gtk flags passed to the __init__ of gtk.MessageDialog
    :param parent: The parent widget of this dialog
    :param buttons: a standard gtk Button passed to the gtk.MessageDialog e.g. BUTTONS_OK or BUTTONS_CANCEL
    :param standalone: specify if the dialog should run by itself and is only cancelable by a callback function
    """

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

        # Run by yourself if this is the requested dialog. If not,
        # it doesnt make sense because the window can't be destroyed properly
        self.run() if standalone else None

    def add_callback(self, callback, *args):
        self.connect('response', callback, *args)


class RAFCONButtonDialog(RAFCONMessageDialog):
    """A dialog which holds a markup text and a configurable amount of buttons specified in button_texts as a string list.
    The buttons' response ids are there indexes in the buttons_texts list plus 1, so the first button responds with 1 on
    a 'clicked' event.

    :param callback: A callback function which should be executed on the end of the run() method
    :param callback_args: Arguments passed to the callback function
    :param markup_text: The text inside the dialog
    :param button_texts: A list containing all buttons_texts to be created as gtk Buttons
    :param type: The gtk type of the dialog, e.g. gtk.MESSAGE_INFO, gtk.MESSAGE_QUESTION etc.
    :param flags: gtk flags passed to the __init__ of gtk.MessageDialog
    :param parent: The parent widget of this dialog
    :param standalone: specify if the dialog should run by itself and is only cancelable by a callback function
    """

    def __init__(self, callback=None, callback_args=(),
                 markup_text=None, button_texts=None,
                 type=gtk.MESSAGE_INFO, flags=gtk.DIALOG_MODAL, parent=None,
                 standalone=False):

        super(RAFCONButtonDialog, self).__init__(callback, callback_args, markup_text, type, flags, parent, buttons=gtk.BUTTONS_NONE)

        # remove the button box as it is no longer needed
        vbox = self.get_action_area().get_parent()
        vbox.remove(self.get_action_area())

        hbox = gtk.HBox(homogeneous=False, spacing=constants.GRID_SIZE)
        for index, button in enumerate(button_texts):
                button = gtk.Button(button)
                button.connect('clicked', self.add_response, index)
                hbox.pack_start(button, True, True, 1)

        # alignment area to resize the buttons to their label size
        align_action_area = gtk.Alignment(xalign=1, yalign=0.0, xscale=0.0, yscale=0.0)

        align_action_area.add(hbox)
        vbox.pack_end(align_action_area)

        self.show_all()
        self.run() if standalone else None

    def add_response(self, widget, index):
        # add responses to the 'clicked' event of the button. First button gets 1 as response Second 2 etc.
        self.response(index+1)


class RAFCONInputDialog(RAFCONButtonDialog):
    """A dialog containing the a number of buttons specified in button_texts, an optional checkbox specified in
    checkbox_text and an entry line to write in. The state of the checkbox can be returned by get_checkbox_state(),
    the content of the entry box can be returned by get_entry(). Hitting the enter button results in the same response
    signal as the first button, so it is recommended to use "ok" or sth. similar as first button.

    :param callback: A callback function which should be executed on the end of the run() method
    :param callback_args: Arguments passed to the callback function
    :param markup_text: The text inside the dialog
    :param button_texts: A list containing all buttons_texts to be created as gtk Buttons
    :param checkbox_text: Define the text of the checkbox next to the entry line. If no present, no checkbox is created
    :param type: The gtk type of the dialog, e.g. gtk.MESSAGE_INFO, gtk.MESSAGE_QUESTION etc.
    :param flags: gtk flags passed to the __init__ of gtk.MessageDialog
    :param parent: The parent widget of this dialog
    :param standalone: specify if the dialog should run by itself and is only cancelable by a callback function
    """

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
    """A dialog containing a column of checkboxes in addition to a number of buttons and a markup text.
    All checkboxes can be returned by get_checkboxes() in addition the state of a single checkbox can be either returned
    by index with get_checkbox_by_index() or by name with get_chackbox_by_name()

    :param callback: A callback function which should be executed on the end of the run() method
    :param callback_args: Arguments passed to the callback function
    :param markup_text: The text inside the dialog
    :param button_texts: A list containing all buttons_texts to be created as gtk Buttons
    :param checkbox_texts: The labels for checkboxes, also defines the number of checkboxes
    :param type: The gtk type of the dialog, e.g. gtk.MESSAGE_INFO, gtk.MESSAGE_QUESTION etc.
    :param flags: gtk flags passed to the __init__ of gtk.MessageDialog
    :param parent: The parent widget of this dialog
    :param standalone: specify if the dialog should run by itself and is only cancelable by a callback function
    """

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
