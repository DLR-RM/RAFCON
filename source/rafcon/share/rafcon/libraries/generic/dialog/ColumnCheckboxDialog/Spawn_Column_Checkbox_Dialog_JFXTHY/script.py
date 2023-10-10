from gi.repository import GLib
from gi.repository import Gtk
from rafcon.gui.utils.dialog import RAFCONColumnCheckboxDialog, get_root_window


def execute(self, inputs, outputs, gvm):
    self.logger.debug("Creating column checkbox dialog")

    if len(inputs['buttons']) != 2:
        self.logger.error("Please specify exactly two buttons as a list")
        return "aborted"

    def run_dialog(event, result, logger):
        dialog_window = RAFCONColumnCheckboxDialog(markup_text=inputs['message_text'],
                                                   button_texts=inputs['buttons'],
                                                   checkbox_texts=inputs['checkbox_texts'], flags=Gtk.DialogFlags.MODAL,
                                                   parent=get_root_window())
        result[1] = dialog_window
        result[0] = dialog_window.run()
        outputs['checkbox_states'] = dialog_window.get_checkbox_states()
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

    abort = inputs['abort_on_quit']

    if response_id == 1:
        return 0

    elif response_id == 2:
        return 1

    if abort:
        return "aborted"
    else:
        return 1
