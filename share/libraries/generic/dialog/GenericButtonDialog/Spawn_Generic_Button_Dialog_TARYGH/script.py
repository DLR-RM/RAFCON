from gi.repository import GObject
from gi.repository import Gtk
from rafcon.gui.utils.dialog import RAFCONButtonDialog, get_root_window


def execute(self, inputs, outputs, gvm):
    self.logger.debug("Creating a generic button dialog")
    
    def run_dialog(event, result, logger):
        dialog_window = RAFCONButtonDialog(markup_text=inputs['message_text'],
                                           button_texts=inputs['buttons'], flags=Gtk.DialogFlags.MODAL,
                                           parent=get_root_window())
        result[1] = dialog_window
        result[0] = dialog_window.run()
        dialog_window.destroy()

        event.set()

    event = self._preempted
    result = [None, None]  # first entry is the dialog return value, second one is the dialog object
    GObject.idle_add(run_dialog, event, result, self.logger)

    # Event is either set by the dialog or by an external preemption request
    event.wait()

    response_id, dialog = result

    # The dialog was not closed by the user, but we got a preemption request
    if response_id is None:
        dialog.destroy()
        return "preempted"

    event.clear()

    outputs['response_id'] = response_id
    
    if response_id >= 1:
        return 0
    if inputs['abort_on_quit']: 
        return "aborted"
    else:
        return 0
