from rafcon.gui.utils.dialog import RAFCONInputDialog

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Creating input dialog")
    
    if len(inputs['buttons']) != 2:
        self.logger.error("Please specify exactly two buttons as a list")
        return "aborted"
        
    dialog_window = RAFCONInputDialog(markup_text=inputs['message_text'],
                                      button_texts=inputs['buttons'],
                                      checkbox_text=inputs['checkbox_text'])
    abort = inputs['abort_on_quit']
    
    response_id = dialog_window.run()
    
    outputs['entered_text'] = dialog_window.get_entry_text()
    outputs['checkbox_state'] = dialog_window.get_checkbox_state()
    
    dialog_window.destroy()
    
    if response_id == 1:
        return 0
        
    elif response_id == 2:
        return 1
    
    if abort: 
        return "aborted"
    else:
        return 1