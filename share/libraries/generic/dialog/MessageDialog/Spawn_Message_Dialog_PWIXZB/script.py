from gi.repository import GObject
from gi.repository import Gtk
from rafcon.gui.utils.dialog import RAFCONMessageDialog, get_root_window


def execute(self, inputs, outputs, gvm):
    self.logger.debug("Creating message dialog")
    markup_text = inputs['message_text']
    abort = inputs['abort_on_quit']

    def run_dialog(event, result, logger):
        dialog_window = RAFCONMessageDialog(markup_text=markup_text,
                                            message_type=Gtk.MessageType.INFO, flags=Gtk.DialogFlags.MODAL,
                                            parent=get_root_window())

        response_id = dialog_window.run()
        result.append(response_id)
        result.append(dialog_window)

        event.set()

    event = self._preempted
    result = []
    GObject.idle_add(run_dialog, event, result, self.logger)

    # Event is either set by the dialog or by an external preemption request
    event.wait()

    response_id = result[0]
    dialog = result[1]

    # The dialog was not closed by the user, but we got a preemption request
    dialog.destroy()
    if response_id is None:
        return "preempted"

    event.clear()

    if response_id == -5:
        return 0
    
    if abort:
        return 'aborted'
    else:
        return 0
