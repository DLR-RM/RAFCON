"""
.. module:: singleton
   :platform: Unix, Windows
   :synopsis: A module to hold all singletons of the GTK GUI

.. moduleauthor:: Rico Belder, Franz Steinmetz


"""
from rafcon.statemachine.singleton import state_machine_manager
from rafcon.statemachine.singleton import global_variable_manager
from rafcon.mvc.models.state_machine_manager import StateMachineManagerModel
from rafcon.mvc.models.global_variable_manager import GlobalVariableManagerModel
from rafcon.statemachine import interface
from rafcon.mvc.runtime_config import global_runtime_config
from os.path import expanduser

global_focus = None


def open_folder(query):
    import gtk
    last_path = global_runtime_config.get_config_value('LAST_PATH_OPEN_SAVE', None)
    if not last_path:
        last_path = expanduser('~')

    dialog = gtk.FileChooserDialog(query,
                                   None,
                                   gtk.FILE_CHOOSER_ACTION_SELECT_FOLDER,
                                   (gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
                                   gtk.STOCK_OPEN, gtk.RESPONSE_OK))
    dialog.set_current_folder(last_path)
    response = dialog.run()

    if response != gtk.RESPONSE_OK:
        dialog.destroy()
        return None

    path = dialog.get_filename()
    dialog.destroy()

    global_runtime_config.set_config_value('LAST_PATH_OPEN_SAVE', path)
    return path

interface.open_folder_func = open_folder


def create_folder(query):
    import gtk
    last_path = global_runtime_config.get_config_value('LAST_PATH_OPEN_SAVE', None)
    if not last_path:
        last_path = expanduser('~')

    dialog = gtk.FileChooserDialog(query,
                                   None,
                                   gtk.FILE_CHOOSER_ACTION_CREATE_FOLDER,
                                   (gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
                                    gtk.STOCK_SAVE, gtk.RESPONSE_OK))
    dialog.set_current_folder(last_path)
    response = dialog.run()

    if response != gtk.RESPONSE_OK:
        dialog.destroy()
        return None

    path = dialog.get_filename()
    dialog.destroy()

    global_runtime_config.set_config_value('LAST_PATH_OPEN_SAVE', path)
    return path

interface.create_folder_func = create_folder


def show_notice(query):
    import gtk
    dialog = gtk.MessageDialog(flags=gtk.DIALOG_MODAL, type=gtk.MESSAGE_INFO, buttons=gtk.BUTTONS_OK)
    dialog.set_markup(query)
    from rafcon.utils.gui_helper import set_button_children_size_request
    set_button_children_size_request(dialog)
    dialog.run()
    dialog.destroy()

interface.show_notice_func = show_notice

# This variable holds the global state machine manager model as long as only one StateMachineMangerModel is allowed
state_machine_manager_model = StateMachineManagerModel(state_machine_manager)

global_variable_manager_model = GlobalVariableManagerModel(global_variable_manager)

main_window_controller = None
