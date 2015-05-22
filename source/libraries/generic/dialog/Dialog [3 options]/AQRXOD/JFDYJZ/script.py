def show_dialog(self, event, text, subtext, options, result):
    import gtk
    dialog = gtk.MessageDialog(type=gtk.MESSAGE_INFO, buttons=gtk.BUTTONS_NONE, flags=gtk.DIALOG_MODAL)
    text = "<span size='50000'>" + text + "</span>"
    dialog.set_markup(text)
    
    if isinstance(subtext, basestring) and len(subtext) > 0:
        subtext = "<span size='20000'>" + subtext + "</span>"
        dialog.format_secondary_markup(subtext)
        
    for i, option in enumerate(options):
        button = gtk.Button()
        label = gtk.Label("<span size='20000'>" + option + "</span>")
        label.set_use_markup(True)
        label.show()
        button.add(label)
        button.set_size_request(300, 300)
        button.show()
        dialog.add_action_widget(button, i)

    res = dialog.run()
    dialog.destroy()
    
    # Group all default response type to -1
    if res < 0:
        res = -1
    
    result[0] = res
    event.set()

def execute(self, inputs, outputs, gvm):
    import gobject
    import threading
    event = threading.Event()
    result = [0]
    
    text = inputs['text']
    subtext = inputs['subtext']
    options = inputs['options']
    
    gobject.idle_add(show_dialog, self, event, text, subtext, options, result)
    event.wait()
    option = result[0]
    
    if option < 0:
        return "aborted"
        
    self.logger.debug("User decided: {0} => {1}".format(option, options[option]))
    outputs['option'] = option
    
    return "done"

