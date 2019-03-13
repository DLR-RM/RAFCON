import warnings
from gi.repository import GLib
from gi.repository import Gtk
from rafcon.gui.utils.dialog import RAFCONMessageDialog, get_root_window
from rafcon.utils.log import RAFCONDeprecationWarning


def execute(self, inputs, outputs, gvm):
    warnings.warn("Please substitute MessageDialog with 'generic/dialog/Dialog [Info]'. "
                  "See the library description for details on the new API.", RAFCONDeprecationWarning)
    self.logger.debug("Creating message dialog")
    markup_text = inputs['message_text']
    abort = inputs['abort_on_quit']

    def run_dialog(event, result, logger):
        dialog_window = RAFCONMessageDialog(markup_text=markup_text,
                                            message_type=Gtk.MessageType.INFO, flags=Gtk.DialogFlags.MODAL,
                                            parent=get_root_window())
        result[1] = dialog_window
        result[0] = dialog_window.run()
        dialog_window.destroy()

        event.set()

    event = self._preempted
    result = [None, None]  # first entry is the dialog return value, second one is the dialog object
    GLib.idle_add(run_dialog, event, result, self.logger)

    # Event is either set by the dialog or by an external preemption request
    event.wait()

    response_id, dialog = result

    # The dialog was not closed by the user, but we got a preemption request
    if response_id is None:
        GLib.idle_add(dialog.destroy)
        return "preempted"

    event.clear()

    if response_id == -5:
        return 0
    
    if abort:
        return 'aborted'
    else:
        return 0
