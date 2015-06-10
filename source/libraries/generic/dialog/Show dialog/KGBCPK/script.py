
def on_dialog_key_press(dialog, event, key_mapping, buttons):
    from gtk.gdk import keyval_name
    key_name = str.lower(keyval_name(event.keyval))
    for i, desired_key in enumerate(key_mapping):
        if i >= len(buttons):
            break
        if isinstance(desired_key, list):
            key_list = desired_key
            for desired_key in key_list:
                if str.lower(desired_key) == key_name:
                    buttons[i].clicked()
        elif str.lower(desired_key) == key_name:
            buttons[i].clicked()
    pass

def show_dialog(self, event, text, subtext, options, key_mapping, result):
    import gtk
    dialog = gtk.MessageDialog(type=gtk.MESSAGE_INFO, buttons=gtk.BUTTONS_NONE, flags=gtk.DIALOG_MODAL)
    result[1] = dialog
    text = "<span size='50000'>" + text + "</span>"
    dialog.set_markup(text)
    
    if isinstance(subtext, basestring) and len(subtext) > 0:
        subtext = "<span size='20000'>" + subtext + "</span>"
        dialog.format_secondary_markup(subtext)
        
    buttons = {}
    for i, option in enumerate(options):
        button = gtk.Button()
        label = gtk.Label("<span size='20000'>" + option + "</span>")
        label.set_use_markup(True)
        label.show()
        button.add(label)
        button.set_size_request(300, 300)
        button.show()
        dialog.add_action_widget(button, i)
        buttons[i] = button
               
    dialog.add_events(gtk.gdk.KEY_PRESS_MASK)
    dialog.connect('key-press-event', on_dialog_key_press, key_mapping, buttons)

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
    
    # self_preempted is a threading.Event object
    event = self._preempted
    result = [None, None]  # first entry is the dialog return value, second one is the dialog object
    
    text = inputs['text']
    subtext = inputs['subtext']
    options = inputs['options']
    key_mapping = inputs['key_mapping']
    
    gobject.idle_add(show_dialog, self, event, text, subtext, options, key_mapping, result)
    
    # Event is either set by the dialog or by an external preemption request
    event.wait()

    option = result[0]
    dialog = result[1]
    
    # The dialog was not closed by the user, but we got a preemption request
    if option is None:
        dialog.destroy()
        return "preempted"
        
    event.clear()
    if option < 0:
        return "aborted"
        
    self.logger.debug("User decided: {0} => {1}".format(option, options[option]))
    outputs['option'] = option
    
    return "done"

