from rafcon.gui.utils.dialog import RAFCONMessageDialog

def execute(self, inputs, outputs, gvm):
    self.logger.debug("Creating message dialog")
    markup_text = inputs['message_text']
    abort = inputs['abort_on_quit']
    
    dialog = RAFCONMessageDialog(markup_text=markup_text)
    
    response_id = dialog.run()                       
    dialog.destroy()
    if response_id == -5:
        return 0
    
    if abort:
        return 'aborted'
    else:
        return 0 
        