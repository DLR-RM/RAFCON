# Copyright (C) 2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Sebastian Brunner <sebastian.brunner@dlr.de>

from builtins import object
import os

from rafcon.gui.utils.shell_execution import execute_command_with_path_in_process
from rafcon.gui.utils.dialog import RAFCONInputDialog
from rafcon.gui.singleton import global_gui_config

from rafcon.utils import log
logger = log.get_logger(__name__)


class AbstractExternalEditor(object):
    """ A class which enables the use of an external editor. Data can be passed to an external editor and loaded back
    into RAFCON. This class expects the inheriting subclass being a gtkmvc3 model.
    This is currently used by the source editor and semantic data editor.

    """

    def __init__(self):
        super(AbstractExternalEditor, self).__init__()

    def execute_shell_command_with_path(self, command, path):
        """ Executes a specific command in the shell (in our case an editor).

        :param command: the command to be executed
        :param path: the path as first argument to the shell command
        :return: None
        """
        return execute_command_with_path_in_process(command, path, logger=logger)

    def get_file_name(self):
        """ The object base class specific file name, which can then be passed to e.g. the external editor shell command

        :return: None
        """
        raise NotImplementedError()

    def save_file_data(self, path):
        """ Saves object-internal data to a (temporary) file, which can then be accessed by an external editor.

        :param path: the path to where to save the data to
        :return: None
        """
        raise NotImplementedError()

    def set_editor_lock(self, lock):
        """ If the external editor is enabled, all internal data modification possibilities have to be disabled.
        On the other side, if the user finished using the external editor, the data can be loaded into RAFCON and the
        modification modalities can be used again.

        :param lock: a flag to either en- or disable the modification modalities
        :return: None
        """
        raise NotImplementedError()

    def load_and_set_file_content(self, file_system_path):
        """ Loads the file content of the specified path and saves it into the internal data structures.

        :param file_system_path: the path to load the data from
        :return: None
        """
        raise NotImplementedError()

    def open_externally_clicked(self, button):
        prefer_external_editor = global_gui_config.get_config_value("PREFER_EXTERNAL_EDITOR")
        file_system_path = self.model.state.file_system_path
        if file_system_path is None:
            file_system_path = self.model.state.get_temp_file_system_path()
            logger.info("External source editor uses temporary storage path {0}.".format(file_system_path))

        def set_editor_lock_inline(lock):
            if lock:
                button.set_label('Reload' if prefer_external_editor else 'Unlock')
            else:
                button.set_label('Open externally')
            button.set_active(lock)
            self.set_editor_lock(lock)

        if button.get_active():
            # Get the specified "Editor" as in shell command from the gui config yaml
            external_editor = global_gui_config.get_config_value('DEFAULT_EXTERNAL_EDITOR', None)

            def open_file_in_editor(cmd_to_open_editor, test_command=False):
                self.save_file_data(file_system_path)
                script_file_path = os.path.join(file_system_path, self.get_file_name())
                if not self.execute_shell_command_with_path(cmd_to_open_editor, script_file_path) and test_command:
                    # If a text field exists destroy it. Errors can occur with a specified editor as well
                    # e.g Permission changes or sth.
                    global_gui_config.set_config_value('DEFAULT_EXTERNAL_EDITOR', None)
                    global_gui_config.save_configuration()
                    set_editor_lock_inline(False)
                    return
                # Set the text on the button to 'Unlock' instead of 'Open externally'
                set_editor_lock_inline(True)

            def open_text_window():
                markup_text = "No external editor specified. Please specify a shell command to open scripts externally"
                # create a new RAFCONButtonInputDialog, add a checkbox and add the text 'remember' to it
                text_input = RAFCONInputDialog(markup_text, ["Apply", "Cancel"], checkbox_text='remember', parent=True)
                # Run the text_input Dialog until a response is emitted. The apply button and the 'activate' signal of
                # the text input send response 1
                if text_input.run() == 1:
                    # If the response emitted from the Dialog is 1 than handle the 'OK'
                    # If the checkbox is activated, also save the text input into the config
                    if text_input.get_checkbox_state():
                        global_gui_config.set_config_value('DEFAULT_EXTERNAL_EDITOR', text_input.get_entry_text())
                        global_gui_config.save_configuration()
                    open_file_in_editor(text_input.get_entry_text(), test_command=True)
                else:
                    # If Dialog is canceled either by the button or the cross, toggle back the button again
                    # and revert the lock, which is not implemented yet
                    set_editor_lock_inline(False)
                text_input.destroy()

            if not external_editor:
                # If no external editor is specified in the gui_config.yaml, open the Dialog and let the user choose
                # a shell command to apply to the file path
                open_text_window()
            else:
                # If an editor is specified, open the path with the specified command. Also text_field is None, there is
                # no active text field in the case of an already specified editor. Its needed for the SyntaxError catch
                open_file_in_editor(external_editor, test_command=True)
        else:
            # If button is clicked after one open a file in the external editor, unlock the internal editor to reload
            set_editor_lock_inline(False)
            # Load file contents after unlocking
            self.load_and_set_file_content(file_system_path)
            # After reload internal editor and external editor is preferred lock internal editor again
            set_editor_lock_inline(prefer_external_editor)

