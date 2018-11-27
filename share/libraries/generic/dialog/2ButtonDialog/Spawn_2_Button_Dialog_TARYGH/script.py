from gi.repository import GObject
from rafcon.gui.utils.dialog import RAFCONButtonDialog

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Creating button dialog")
    
    if len(inputs['buttons']) != 2:
        self.logger.error("Please specify exactly two buttons as a list")
        return "aborted"
       
    def run_dialog(event, result, logger):
        dialog_window = RAFCONButtonDialog(markup_text=inputs['message_text'],
                                      button_texts=inputs['buttons'])
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
    dialog  = result[1]
    dialog.destroy()

    # The dialog was not closed by the user, but we got a preemption request
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