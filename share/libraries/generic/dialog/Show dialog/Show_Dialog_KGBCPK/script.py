from future.utils import string_types
from gi.repository import Gtk
from rafcon.gui.utils.dialog import get_root_window, RAFCONMessageDialog


class RAFCONShowDialog(RAFCONMessageDialog):

    def __init__(self, text, subtext, options, key_mapping, logger):

        RAFCONMessageDialog.__init__(self, message_type=Gtk.MessageType.INFO, flags=Gtk.DialogFlags.MODAL,
                                     parent=get_root_window(), title="RAFCON State Machine Dialog - ShowDialog")

        self.set_size_request(600, -1)

        markup_text = text
        if isinstance(markup_text, string_types):
            from cgi import escape
            self.set_markup(escape(str(markup_text)))
        else:
            logger.debug("The specified message '{1}' text is not a string, but {0}".format(markup_text,
                                                                                            type(markup_text)))

        if isinstance(subtext, string_types) and len(subtext) > 0:
            subtext = "<span size='20000'>" + subtext + "</span>"
            self.format_secondary_markup(subtext)

        buttons = {}
        for i, option in enumerate(options):
            button = Gtk.Button()

            label = Gtk.Label("<span size='20000'>" + option + "</span>")
            label.set_use_markup(True)
            label.show()
            button.add(label)
            # button.set_size_request(300, 300)  # TODO maybe parameter
            button.show()
            self.add_action_widget(button, i)
            buttons[i] = button

        # dialog.add_events(Gdk.KEY_PRESS_MASK)
        if key_mapping:
            self.connect('key-press-event', self.on_dialog_key_press, key_mapping, buttons)

    def on_dialog_key_press(self, dialog, event, key_mapping, buttons):
        from gi.repository import Gdk
        key_name = str.lower(str(Gdk.keyval_name(event.keyval)))
        for i, desired_key in enumerate(key_mapping):
            if i >= len(buttons):
                break
            if isinstance(desired_key, list):
                key_list = desired_key
                for desired_key in key_list:
                    if str.lower(str(desired_key)) == key_name:
                        buttons[i].clicked()
            elif str.lower(desired_key) == key_name:
                buttons[i].clicked()
        pass


def show_dialog(event, text, subtext, options, key_mapping, result, logger):

    dialog = RAFCONShowDialog(text, subtext, options, key_mapping, logger)

    result[1] = dialog

    res = dialog.run()
    dialog.destroy()

    # Group all default response type to -1
    if res < 0:
        res = -1
    
    result[0] = res
    event.set()


def execute(self, inputs, outputs, gvm):
    from gi.repository import GLib

    # self_preempted is a threading.Event object
    event = self._preempted
    result = [None, None]  # first entry is the dialog return value, second one is the dialog object
    
    text = inputs['text']
    subtext = inputs['subtext']
    options = inputs['options']
    key_mapping = inputs['key_mapping']

    GLib.idle_add(show_dialog, event, text, subtext, options, key_mapping, result, self.logger)

    # Event is either set by the dialog or by an external preemption request
    event.wait()

    option, dialog = result

    # The dialog was not closed by the user, but we got a preemption request
    if option is None:
        GLib.idle_add(dialog.destroy)
        return "preempted"

    event.clear()
    if option < 0:
        return "aborted"

    self.logger.debug("User decided: {0} => {1}".format(option, options[option]))
    outputs['option'] = option

    return "done"
