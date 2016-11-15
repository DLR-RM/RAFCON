import gtk
from enum import Enum

from rafcon.mvc.utils import constants
from rafcon.utils import log
logger = log.get_logger(__name__)


class ButtonDialog(Enum):
    __order__ = "OPTION_1 OPTION_2 OPTION_3 OPTION_4 OPTION_5"
    OPTION_1 = 1
    OPTION_2 = 2
    OPTION_3 = 3
    OPTION_4 = 4
    OPTION_5 = 5


class RAFCONDialog(gtk.MessageDialog):

    def __init__(self, type=gtk.MESSAGE_INFO, buttons=gtk.BUTTONS_NONE, flags=gtk.DIALOG_MODAL, parent=None):

        super(RAFCONDialog, self).__init__(type=type, buttons=buttons, flags=flags)

        # Prevent dialog window from being hidden by the main window
        if parent:
            self.set_transient_for(parent)

    def set_markup(self, markup_text):
        from cgi import escape
        super(RAFCONDialog, self).set_markup(escape(markup_text))

    def finalize(self, callback, *args):

        # If no special callback function was defined, don't connect one
        if callback:
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


class RAFCONButtonDialog(RAFCONDialog):
    def __init__(self, markup_text, button_texts, callback, callback_args=(), standalone=True, type=gtk.MESSAGE_INFO,
                 parent=None, width=None):

        super(RAFCONButtonDialog, self).__init__(type, gtk.BUTTONS_NONE, gtk.DIALOG_MODAL, parent)

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

        # This assures that if a ButtonInputDialog is created, the init of this class doesnt already run the window.
        # Maybe it is better to run this window from the caller not from its own init, this also gives the possibility
        # to get the return of the run method or otherwise make a new class which only runs on init
        if standalone:
            self.finalize(callback, *callback_args)
            self.grab_focus()
            self.run()


class RAFCONButtonInputDialog(RAFCONButtonDialog):

    def __init__(self, markup_text, button_texts, callback=None, callback_args=(), checkbox=False,
                 checkbox_text='check',
                 button_box_flag=False, type=gtk.MESSAGE_QUESTION, parent=None):

        super(RAFCONButtonInputDialog, self).__init__(markup_text, button_texts, callback, callback_args,
                                                      button_box_flag, type, parent)

        # Setup new text entry line
        self.entry = gtk.Entry()
        self.entry.set_max_length(60)
        self.entry.set_editable(1)
        self.entry.set_activates_default(True)
        self.entry.set_width_chars(10)

        self.entry.connect('activate', lambda w: self.response(1))

        self.entry.show()

        vbox = self.get_content_area()
        # create a new gtk.Hbox to put in the checkbox and the entry
        hbox = gtk.HBox(homogeneous=False, spacing=constants.GRID_SIZE)
        # add the hbox to the content area
        vbox.add(hbox)

        if checkbox:
            # If a checkbox is asked for by the caller, create one.
            self.check = gtk.CheckButton(checkbox_text)
            self.check.show()
            hbox.pack_end(self.check, True, True, 1)

        hbox.pack_end(self.entry, True, True, 1)

        vbox.show()
        hbox.show()

        self.show()
        self.finalize(callback, *callback_args)
        self.grab_focus()

    def return_text(self):
        return self.entry.get_text()

    def return_check(self):
        return self.check.get_state()
