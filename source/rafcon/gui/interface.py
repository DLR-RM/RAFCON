# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import os
import gtk
import glib

from rafcon.core import interface as core_interface
from rafcon.gui.runtime_config import global_runtime_config
from rafcon.gui.singleton import main_window_controller, library_manager


def add_library_root_path_to_shortcut_folders_of_dialog(dialog):
    library_paths = library_manager.library_root_paths
    library_keys = sorted(library_paths)
    for library_key in library_keys:
        try:
            dialog.add_shortcut_folder(library_paths[library_key])
        except glib.GError, e:
            # this occurs if the shortcut file already exists
            # unfortunately dialog.list_shortcut_folders() does not work
            # that's why the error is caught
            pass

def open_folder(query, default_path=None):
    """Shows a user dialog for folder selection
    
    A dialog is opened with the prompt `query`. The current path is set to the last path that was opened/created. The 
    roots of all libraries is added to the list of shortcut folders.
    
    :param str query: Prompt asking the user for a specific folder
    :param str default_path: Path to use if user does not specify one 
    :return: Path selected by the user or `default_path` if no path was specified or None if none of the paths is valid
    :rtype: str
    """
    from os.path import expanduser, pathsep, dirname, isdir
    last_path = global_runtime_config.get_config_value('LAST_PATH_OPEN_SAVE', "")
    selected_filename = None
    if last_path and isdir(last_path):
        selected_filename = last_path.split(pathsep)[-1]
        last_path = dirname(last_path)
    else:
        last_path = expanduser('~')

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

    # Add library roots to list of shortcut folders
    add_library_root_path_to_shortcut_folders_of_dialog(dialog)

    response = dialog.run()

    if response != gtk.RESPONSE_OK:
        dialog.destroy()
        if default_path and os.path.isdir(default_path):
            return default_path
        return None

    path = dialog.get_filename()
    dialog.destroy()

    if os.path.isdir(path):
        global_runtime_config.set_config_value('LAST_PATH_OPEN_SAVE', path)
        return path
    return None


# overwrite the open_folder_func of the interface: thus the user input is now retrieved from a dialog box and not
# from raw input any more
core_interface.open_folder_func = open_folder


def create_folder(query, default_name=None, default_path=None):
    """Shows a user dialog for folder creation
    
    A dialog is opened with the prompt `query`. The current path is set to the last path that was opened/created. The 
    roots of all libraries is added to the list of shortcut folders.
    
    :param str query: Prompt asking the user for a specific folder
    :param str default_name: Default name of the folder to be created 
    :param str default_path: Path in which the folder is created if the user doesn't specify a path 
    :return: Path created by the user or `default_path`\`default_name` if no path was specified or None if none of the
      paths is valid
    :rtype: str
    """
    from os.path import expanduser, dirname, join, exists, isdir
    from rafcon.core.storage.storage import STATEMACHINE_FILE
    last_path = global_runtime_config.get_config_value('LAST_PATH_OPEN_SAVE', "")

    if last_path and isdir(last_path) and not exists(join(last_path, STATEMACHINE_FILE)):
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
    if default_name:
        dialog.set_current_name(default_name)
    dialog.set_show_hidden(False)

    # Add library roots to list of shortcut folders
    add_library_root_path_to_shortcut_folders_of_dialog(dialog)

    response = dialog.run()

    if response != gtk.RESPONSE_OK:
        dialog.destroy()
        if default_path and default_name:
            default = os.path.join(default_path, default_name)
            if os.path.isdir(default):
                return default
        return None

    path = dialog.get_filename()
    dialog.destroy()

    if os.path.isdir(path):
        global_runtime_config.set_config_value('LAST_PATH_OPEN_SAVE', path)
        return path
    return None

# overwrite the create_folder_func of the interface: thus the user input is now retrieved from a dialog box and not
# from raw input any more
core_interface.create_folder_func = create_folder


def save_folder(query, default_name=None):
    """Shows a user dialog to save a folder/file

    A dialog is opened with the prompt `query`. The current path is set to the last path that was opened/created. The
    roots of all libraries is added to the list of shortcut folders. The file name can be folder or file.
    No existence check is performed.

    :param str query: Prompt asking the user for a specific path and a name in the entry widget
    :param str default_name: Default name of mounting point/library root key
    :return: Path handed by the user or `last_path`\`default_name` if no path was specified and None if directory
      does not exist (parent of name)
    :rtype: str
    """
    from os.path import expanduser, dirname, join, exists, isdir
    from rafcon.core.storage.storage import STATEMACHINE_FILE
    last_path = global_runtime_config.get_config_value('LAST_PATH_OPEN_SAVE', "")

    if last_path and isdir(last_path) and not exists(join(last_path, STATEMACHINE_FILE)):
        pass
    elif last_path:
        last_path = dirname(last_path)
    else:
        last_path = expanduser('~')

    dialog = gtk.FileChooserDialog(query,
                                   None,
                                   gtk.FILE_CHOOSER_ACTION_SAVE,
                                   (gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
                                    gtk.STOCK_OPEN, gtk.RESPONSE_OK))
    # Allows confirming with Enter and double-click
    dialog.set_default_response(gtk.RESPONSE_OK)
    if main_window_controller:
        dialog.set_transient_for(main_window_controller.view.get_top_widget())
    dialog.set_current_folder(last_path)
    if default_name:
        dialog.set_current_name(default_name)
    dialog.set_show_hidden(False)

    # Add library roots to list of shortcut folders
    add_library_root_path_to_shortcut_folders_of_dialog(dialog)

    response = dialog.run()

    if response != gtk.RESPONSE_OK:
        dialog.destroy()
        return None

    path = dialog.get_filename()
    dialog.destroy()

    # check path existence
    if not exists(dirname(path)):
        return None
    return path

# overwrite the save_folder_func of the interface: thus the user input is now retrieved from a dialog box and not
# from raw input any more
core_interface.save_folder_func = save_folder


def show_notice(query):
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
