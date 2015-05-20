"""
.. module:: singleton
   :platform: Unix, Windows
   :synopsis: A module to hold all singletons of the GTK GUI

.. moduleauthor:: Rico Belder, Franz Steinmetz


"""
from awesome_tool.statemachine.singleton import state_machine_manager
from awesome_tool.statemachine.singleton import global_variable_manager
from awesome_tool.mvc.models.state_machine_manager import StateMachineManagerModel
from awesome_tool.mvc.models.global_variable_manager import GlobalVariableManagerModel
from awesome_tool.statemachine import interface

global_focus = None


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

interface.open_folder_func = open_folder


def create_folder(query):
    import gtk
    dialog = gtk.FileChooserDialog(query,
                                   None,
                                   gtk.FILE_CHOOSER_ACTION_CREATE_FOLDER,
                                   (gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
                                    gtk.STOCK_SAVE, gtk.RESPONSE_OK))
    response = dialog.run()
    if response != gtk.RESPONSE_OK:
        dialog.destroy()
        return None

    path = dialog.get_filename()
    dialog.destroy()
    return path

interface.create_folder_func = create_folder

# This variable holds the global state machine manager model as long as only one StateMachineMangerModel is allowed
state_machine_manager_model = StateMachineManagerModel(state_machine_manager)

global_variable_manager_model = GlobalVariableManagerModel(global_variable_manager)