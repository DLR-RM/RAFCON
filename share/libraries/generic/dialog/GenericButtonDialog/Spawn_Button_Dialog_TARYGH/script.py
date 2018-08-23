from rafcon.gui.utils.dialog import RAFCONButtonDialog

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Creating a generic button dialog")
        
    dialog_window = RAFCONButtonDialog(markup_text=inputs['message_text'],
                                       button_texts=inputs['buttons'])
    
    response_id = dialog_window.run()
    
    dialog_window.destroy()
    outputs['response_id'] = response_id
    
    if response_id >= 1:
        return 0
    if inputs['abort_on_quit']: 
        return "aborted"
    else:
        return 1