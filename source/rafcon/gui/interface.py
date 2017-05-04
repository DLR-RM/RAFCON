from rafcon.core import interface as core_interface
from rafcon.gui.runtime_config import global_runtime_config
from rafcon.gui.singleton import main_window_controller, library_manager


def open_folder(query):
    import gtk
    from os.path import expanduser, pathsep
    last_path = global_runtime_config.get_config_value('LAST_PATH_OPEN_SAVE', None)
    selected_filename = None
    if not last_path:
        last_path = expanduser('~')
    else:
        selected_filename = last_path.split()[-1]
        last_path = pathsep.join(last_path.split()[:-1])

    dialog = gtk.FileChooserDialog(query,
                                   None,
                                   gtk.FILE_CHOOSER_ACTION_SELECT_FOLDER,
                                   (gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
                                    gtk.STOCK_OPEN, gtk.RESPONSE_OK))
    # Allows confirming with Enter and double-click
    dialog.set_default_response(gtk.RESPONSE_OK)
    if main_window_controller:
        dialog.set_transient_for(main_window_controller.view.get_top_widget())
    dialog.set_current_folder(last_path)
    if selected_filename is not None:
        dialog.select_filename(selected_filename)

    dialog.set_show_hidden(False)

    library_paths = library_manager.library_paths
    library_keys = sorted(library_paths)
    for library_key in library_keys:
        dialog.add_shortcut_folder(library_paths[library_key])

    response = dialog.run()

    if response != gtk.RESPONSE_OK:
        dialog.destroy()
        return None

    path = dialog.get_filename()
    dialog.destroy()

    global_runtime_config.set_config_value('LAST_PATH_OPEN_SAVE', path)
    return path


# overwrite the open_folder_func of the interface: thus the user input is now retrieved from a dialog box and not
# from raw input any more
core_interface.open_folder_func = open_folder


def create_folder(query):
    import gtk
    from os.path import expanduser, dirname, join, exists, isdir
    from rafcon.core.storage.storage import STATEMACHINE_FILE
    last_path = global_runtime_config.get_config_value('LAST_PATH_OPEN_SAVE', None)
    suggested_folder_name = global_runtime_config.get_config_value('CURRENT_SUGGESTED_FOLDER_NAME', '')

    if isdir(last_path) and not exists(join(last_path, STATEMACHINE_FILE)):
        pass
    elif last_path:
        last_path = dirname(last_path)
    else:
        last_path = expanduser('~')

    dialog = gtk.FileChooserDialog(query,
                                   None,
                                   gtk.FILE_CHOOSER_ACTION_CREATE_FOLDER,
                                   (gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
                                    gtk.STOCK_SAVE, gtk.RESPONSE_OK))
    # Allows confirming with Enter and double-click
    dialog.set_default_response(gtk.RESPONSE_OK)
    if main_window_controller:
        dialog.set_transient_for(main_window_controller.view.get_top_widget())
    dialog.set_current_folder(last_path)
    dialog.set_current_name(suggested_folder_name)
    dialog.set_show_hidden(False)

    library_paths = library_manager.library_paths
    library_keys = sorted(library_paths)
    for library_key in library_keys:
        dialog.add_shortcut_folder(library_paths[library_key])

    response = dialog.run()

    if response != gtk.RESPONSE_OK:
        dialog.destroy()
        return None

    path = dialog.get_filename()
    dialog.destroy()

    global_runtime_config.set_config_value('LAST_PATH_OPEN_SAVE', path)
    return path

# overwrite the create_folder_func of the interface: thus the user input is now retrieved from a dialog box and not
# from raw input any more
core_interface.create_folder_func = create_folder


def show_notice(query):
    import gtk
    from rafcon.gui.helpers.label import set_button_children_size_request
    from xml.sax.saxutils import escape
    dialog = gtk.MessageDialog(flags=gtk.DIALOG_MODAL, type=gtk.MESSAGE_INFO, buttons=gtk.BUTTONS_OK)
    if main_window_controller:
        dialog.set_transient_for(main_window_controller.view.get_top_widget())
    dialog.set_markup(escape(query))
    set_button_children_size_request(dialog)
    dialog.run()
    dialog.destroy()

# overwrite the show_notice_func of the interface: thus the user input is now retrieved from a dialog box and not
# from raw input any more
core_interface.show_notice_func = show_notice
