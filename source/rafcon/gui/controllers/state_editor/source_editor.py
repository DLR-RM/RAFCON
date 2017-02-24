"""
.. module:: source_editor
   :synopsis: A module holds the controller to edit the state source script text.

"""

import os
import subprocess
import gtk
import shlex
import contextlib
from pylint import lint
from pylint.reporters.json import JSONReporter
from cStringIO import StringIO
from astroid import MANAGER

from rafcon.core.states.library_state import LibraryState

from rafcon.gui.controllers.utils.editor import EditorController
from rafcon.gui.singleton import state_machine_manager_model
from rafcon.gui.config import global_gui_config
from rafcon.gui.utils.dialog import RAFCONButtonDialog, ButtonDialog
from rafcon.utils import filesystem
from rafcon.utils.constants import RAFCON_TEMP_PATH_STORAGE
from rafcon.utils import log

logger = log.get_logger(__name__)


class SourceEditorController(EditorController):
    """Controller handling the source editor in Execution States.

    :param
    :param rafcon.gui.views.source_editor.SourceEditorView view: The GTK view showing the source editor.
    """
    # TODO Missing functions
    # - Code function-expander
    # - Code completion

    tmp_file = os.path.join(RAFCON_TEMP_PATH_STORAGE, 'file_to_get_pylinted.py')

    def __init__(self, model, view):
        """Constructor"""
        super(SourceEditorController, self).__init__(model, view, observed_method="script_text")

    def register_view(self, view):
        super(SourceEditorController, self).register_view(view)

        self.saved_initial = False

        view['open_external_button'].connect('clicked', self.open_external_clicked)
        view['apply_button'].connect('clicked', self.apply_clicked)
        view['cancel_button'].connect('clicked', self.cancel_clicked)

        view['pylint_check_button'].set_active(global_gui_config.get_config_value('CHECK_PYTHON_FILES_WITH_PYLINT', False))

        view['pylint_check_button'].set_tooltip_text("Change global default value in GUI config.")
        view['apply_button'].set_tooltip_text(global_gui_config.get_config_value('SHORTCUTS')['apply'][0])
        view['open_external_button'].set_tooltip_text("Open source in external editor. " +
                                                      global_gui_config.get_config_value('SHORTCUTS')['open_external_editor'][0])

        if isinstance(self.model.state, LibraryState):
            view['pylint_check_button'].set_sensitive(False)
            view.textview.set_sensitive(False)
            view['apply_button'].set_sensitive(False)
            view['cancel_button'].set_sensitive(False)


    @property
    def source_text(self):
        return self.model.state.script_text

    @source_text.setter
    def source_text(self, text):
        self.model.state.script_text = text

    # ===============================================================
    def append_shell_command_to_path(self, command, file_path):

        logger.debug("Opening path with command: {}".format(command))

        # This splits the command in a matter so that the editor gets called in a separate shell and thus
        # doesnt lock the window.
        args = shlex.split(command + ' "' + file_path + os.path.sep + 'script.py"')

        try:
            subprocess.Popen(args)
            return True
        except OSError as e:

            # This catches most of the errors being returned from the shell, destroys the old textfield and
            # opens the dialog again, so the user can specify a new command. Its a bit dirty...
            # The if is required to catch the case that a OSerror occurs due to other reasons than
            # the specified command
            logger.error('The operating system raised an error: {}'.format(e))
        return False

    def save_script(self):

        file_path = self.model.state.get_file_system_path()

        try:
            # Save the file before opening it to update the applied changes. Use option create_full_path=True
            # to assure that temporary state_machines' script files are saved to
            # (their path doesnt exist when not saved)
            filesystem.write_file(file_path + os.path.sep + 'script.py',
                                  self.view.get_text(), create_full_path=True)
        except IOError as e:
            # Only happens if the file doesnt exist yet and would be written to the temp folder.
            # The method write_file doesnt create the path
            logger.error('The operating system raised an error: {}'.format(e))
        return file_path

    def open_external_clicked(self, button):

        prefer_external_editor = global_gui_config.get_config_value("PREFER_EXTERNAL_EDITOR")

        def set_editor_lock(lock):
            if lock:
                button.set_label('Reload' if prefer_external_editor else 'Unlock')
            else:
                button.set_label('Open externally')
            button.set_active(lock)
            self.view.set_enabled(not lock)

        if button.get_active():
            # Get the specified "Editor" as in shell command from the gui config yaml
            external_editor = global_gui_config.get_config_value('DEFAULT_EXTERNAL_EDITOR')

            def open_file_in_editor(command, text_field):

                file_path = self.model.state.get_file_system_path()

                sm = state_machine_manager_model.state_machine_manager.get_active_state_machine()
                if sm.marked_dirty and not self.saved_initial:
                    self.save_script()
                    self.saved_initial = True

                if not self.append_shell_command_to_path(command, file_path) and text_field:
                    # If a text field exists destroy it. Errors can occur with a specified editor as well
                    # e.g Permission changes or sth.
                    text_field.destroy()
                    global_gui_config.set_config_value('DEFAULT_EXTERNAL_EDITOR', None)
                    global_gui_config.save_configuration()

                    set_editor_lock(False)
                    return

                # Set the text on the button to 'Unlock' instead of 'Open externally'
                set_editor_lock(True)

            def open_text_window():

                from rafcon.gui.utils.dialog import RAFCONButtonInputDialog
                markup_text = "No external editor specified. Please specify a shell command to open scripts externally"

                # create a new RAFCONButtonInputDialog, add a checkbox and add the text 'remember' to it
                text_input = RAFCONButtonInputDialog(markup_text, ["Apply", "Cancel"],
                                                     checkbox=True, checkbox_text='remember')

                # Run the text_input Dialog until a response is emitted. The apply button and the 'activate' signal of
                # the textinput send response 1
                if text_input.run() == 1:
                    # If the response emitted from the Dialog is 1 than handle the 'OK'

                    # If the checkbox is activated, also save the textinput the the config
                    if text_input.return_check():
                        global_gui_config.set_config_value('DEFAULT_EXTERNAL_EDITOR', text_input.return_text())
                        global_gui_config.save_configuration()
                    open_file_in_editor(text_input.return_text(), text_input)

                else:
                    # If Dialog is canceled either by the button or the cross, untoggle the button again and revert the
                    # lock, which is not implemented yet
                    set_editor_lock(False)

                text_input.destroy()

            if not external_editor:

                # If no external editor is specified in the gui_config.yaml, open the Dialog and let the user choose
                # a shell command to apply to the file path
                open_text_window()

            else:

                # If an editor is specified, open the path with the specified command. Also text_field is None, there is
                # no active text field in the case of an already specified editor. Its needed for the SyntaxError catch
                open_file_in_editor(external_editor, text_field=None)
        else:
            # If button is clicked after one open a file in the external editor, unlock the internal editor to reload
            set_editor_lock(False)

            # Load file contents after unlocking
            content = filesystem.read_file(self.model.state.get_file_system_path(), 'script.py')
            self.set_script_text(content)

            # After reload internal editor and external editor is preferred lock internal editor again
            set_editor_lock(prefer_external_editor)

    def apply_clicked(self, button):
        """Triggered when the Apply button in the source editor is clicked.

        """
        if isinstance(self.model.state, LibraryState):
            logger.warn("It is not allowed to modify libraries.")
            self.view.set_text("")
            return

        # Ugly workaround to give user at least some feedback about the parser
        # Without the loop, this function would block the GTK main loop and the log message would appear after the
        # function has finished
        # TODO: run parser in separate thread
        while gtk.events_pending():
            gtk.main_iteration_do()

        # get script
        current_text = self.view.get_text()

        # Directly apply script if linter was deactivated
        if not self.view['pylint_check_button'].get_active():
            self.set_script_text(current_text)
            return

        logger.debug("Parsing execute script...")
        with open(self.tmp_file, "w") as text_file:
            text_file.write(current_text)

        # clear astroid module cache, see http://stackoverflow.com/questions/22241435/pylint-discard-cached-file-state
        MANAGER.astroid_cache.clear()
        lint_config_file = os.path.join(os.environ['RAFCON_PATH'], "pylintrc")
        args = ["--rcfile={}".format(lint_config_file)]  # put your own here
        with contextlib.closing(StringIO()) as dummy_buffer:
            json_report = JSONReporter(dummy_buffer)
            try:
                lint.Run([self.tmp_file] + args, reporter=json_report, exit=False)
            except:
                logger.exception("Could not run linter to check script")
        os.remove(self.tmp_file)

        if json_report.messages:
            def on_message_dialog_response_signal(widget, response_id):
                if response_id == ButtonDialog.OPTION_1.value:
                    self.set_script_text(current_text)
                else:
                    logger.debug("The script was not saved")
                widget.destroy()

            message_string = "Are you sure that you want to save this file?\n\nThe following errors were found:"

            line = None
            for message in json_report.messages:
                (error_string, line) = self.format_error_string(message)
                message_string += "\n\n" + error_string

            # focus line of error
            if line:
                tbuffer = self.view.get_buffer()
                start_iter = tbuffer.get_start_iter()
                start_iter.set_line(int(line)-1)
                tbuffer.place_cursor(start_iter)
                message_string += "\n\nThe line was focused in the source editor."

            # select state to show source editor
            sm_m = state_machine_manager_model.get_state_machine_model(self.model)
            if sm_m.selection.get_selected_state() is not self.model:
                sm_m.selection.set(self.model)

            RAFCONButtonDialog(message_string, ["Save with errors", "Do not save"],
                               on_message_dialog_response_signal,
                               type=gtk.MESSAGE_WARNING, parent=self.get_root_window())
        else:
            self.set_script_text(current_text)

    @staticmethod
    def format_error_string(message):
        return "Line {}: {} ({})".format(message["line"], message["message"], message["symbol"]), message["line"]
