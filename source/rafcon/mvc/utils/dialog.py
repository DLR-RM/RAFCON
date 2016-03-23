import gtk

from rafcon.mvc.utils import constants


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
