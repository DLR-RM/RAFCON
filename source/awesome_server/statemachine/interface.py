

# def open_folder_cmd_line(query):
#     user_input = raw_input(query + ': ')
#     if len(user_input) == 0:
#         return None
#     return user_input
#
# open_folder_func = open_folder_cmd_line


def open_folder(query):
    import gtk
    dialog = gtk.FileChooserDialog(query,
                                   None,
                                   gtk.FILE_CHOOSER_ACTION_SELECT_FOLDER,
                                   (gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
                                   gtk.STOCK_OPEN, gtk.RESPONSE_OK))
    response = dialog.run()

    if response != gtk.RESPONSE_OK:
        dialog.destroy()
        return None

    path = dialog.get_filename()
    dialog.destroy()

    return path

open_folder_func = open_folder


def create_folder_cmd_line(query):
    import os
    user_input = raw_input(query + ': ')
    if len(user_input) == 0 or os.path.exists(user_input):
        return None

    try:
        os.makedirs(user_input)
    except OSError:
        return None
    return user_input

create_folder_func = create_folder_cmd_line
