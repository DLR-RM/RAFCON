from gi.repository import GObject
from gi.repository import Gtk
from rafcon.gui.utils.dialog import RAFCONInputDialog, get_root_window


def execute(self, inputs, outputs, gvm):
    self.logger.debug("Creating input dialog")
    
    if len(inputs['buttons']) != 2:
        self.logger.error("Please specify exactly two buttons as a list")
        return "aborted"
        
    def run_dialog(event, result, logger):
        dialog_window = RAFCONInputDialog(markup_text=inputs['message_text'],
                                          button_texts=inputs['buttons'],
                                          checkbox_text=inputs['checkbox_text'], flags=Gtk.DialogFlags.MODAL,
                                          parent=get_root_window(), width=600., height=-1)

        dialog_window.show_all()
        # dialog_window.set_resizable(True)

        response_id = dialog_window.run()

        outputs['entered_text'] = dialog_window.get_entry_text()
        outputs['checkbox_state'] = dialog_window.get_checkbox_state()
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
        
    abort = inputs['abort_on_quit']
    
    if response_id == 1:
        return 0
        
    elif response_id == 2:
        return 1
    
    if abort: 
        return "aborted"
    else:
        return 1