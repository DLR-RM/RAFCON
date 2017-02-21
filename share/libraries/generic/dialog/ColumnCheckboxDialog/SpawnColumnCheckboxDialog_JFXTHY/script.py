from rafcon.gui.utils import dialog

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Creating columncheckbox dialog")
    
    if len(inputs['buttons']) != 2:
        self.logger.error("Please specify exactly two buttons as a list")
        return "aborted"
        
    dialog_window = dialog.RAFCONColumnCheckboxDialog(markup_text=inputs['message_text'],
                                               button_texts=inputs['buttons'],
                                               checkbox_texts=inputs['checkbox_texts'])
    abort = inputs['abort_on_quit']
    
    response_id = dialog_window.run()
    
    outputs['checkbox_states'] = dialog_window.get_checkbox_states()
    
    dialog_window.destroy()
    
    if response_id == 1:
        return 0
        
    elif response_id == 2:
        return 1
    
    if abort: 
        return "aborted"
    else:
        return 1