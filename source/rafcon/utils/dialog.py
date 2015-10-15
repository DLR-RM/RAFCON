from gtk import MessageDialog, HBox, MESSAGE_INFO, BUTTONS_NONE, DIALOG_MODAL, Alignment

from constants import BUTTON_MIN_WIDTH, BUTTON_MIN_HEIGHT


class RAFCONDialog(MessageDialog):

    def __init__(self, type=MESSAGE_INFO, buttons=BUTTONS_NONE, flags=DIALOG_MODAL):
        super(RAFCONDialog, self).__init__(type=type, buttons=buttons, flags=flags)

    def set_markup(self, markup_text):
        from cgi import escape
        super(RAFCONDialog, self).set_markup(escape(markup_text))

    def finalize(self, callback, *args):
        self.connect('response', callback, *args)
        margin = 15
        button_box = self.get_action_area()
        vbox = button_box.get_parent()
        hbox = HBox(homogeneous=False, spacing=margin/2.)
        buttons = [button for button in button_box.get_children()]
        for button in buttons:
            button_box.remove(button)
            label = button.get_children()[0]
            label_size = label.size_request()
            button.set_size_request(max(BUTTON_MIN_WIDTH, label_size[0] + 2 * margin), BUTTON_MIN_HEIGHT)
            hbox.pack_end(button)
        vbox.remove(button_box)
        align_action_area = Alignment(xalign=1, yalign=0.0, xscale=0.0, yscale=0.0)
        align_action_area.add(hbox)
        vbox.pack_end(align_action_area)
        align_action_area.show()
        hbox.show()
        self.show()