from gi.repository import Gtk
from rafcon.gui.utils.dialog import get_root_window, RAFCONButtonDialog


class RAFCONShowDialog(RAFCONButtonDialog):

    def __init__(self, text, subtext, options, key_mapping, logger):
        subtext = subtext or text
        RAFCONButtonDialog.__init__(self, message_type=Gtk.MessageType.INFO, flags=Gtk.DialogFlags.MODAL,
                                    parent=get_root_window(),
                                    title=text, markup_text=subtext,
                                    button_texts=options)

        if key_mapping:
            self.connect('key-press-event', self.on_dialog_key_press, key_mapping, options)

    def on_dialog_key_press(self, dialog, event, key_mapping, buttons):
        from gi.repository import Gdk
        key_name = Gdk.keyval_name(event.keyval).lower()
        for i, key_list in enumerate(key_mapping, 1):
            if i > len(buttons):
                break
            if not isinstance(key_list, list):
                key_list = [key_list]
            for desired_key in key_list:
                if desired_key.lower() == key_name:
                    self.response(i)


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
    
    text = '' if inputs['text'] is None else inputs['text']
    subtext = '' if inputs['subtext'] is None else inputs['subtext']
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

    option -= 1  # output numeration starts with 0, response id starts with 1
    self.logger.debug("User decided: {0} => {1}".format(option, options[option]))
    outputs['option'] = option

    return "done"
