"""
.. module:: source_editor
   :platform: Unix, Windows
   :synopsis: A module holds the controller to edit the state source script text.

.. moduleauthor:: Rico Belder


"""

import os
import subprocess
import gtk
import shlex
from pylint import epylint as lint

from rafcon.statemachine.states.library_state import LibraryState

from rafcon.mvc.controllers.utils.editor import EditorController
from rafcon.mvc.singleton import state_machine_manager_model
from rafcon.mvc.config import global_gui_config
from rafcon.utils import filesystem
from rafcon.utils.constants import RAFCON_TEMP_PATH_STORAGE
from rafcon.utils import log

logger = log.get_logger(__name__)


class SourceEditorController(EditorController):
    """Controller handling the source editor in Execution States.

    :param
    :param rafcon.mvc.views.source_editor.SourceEditorView view: The GTK view showing the source editor.
    """
    # TODO Missing functions
    # - Code function-expander
    # - Code completion

    tmp_file = os.path.join(RAFCON_TEMP_PATH_STORAGE, 'file_to_get_pylinted.py')

    def __init__(self, model, view):
        """Constructor"""
        super(SourceEditorController, self).__init__(model, view, observed_method="script_text")
        self.not_pylint_compatible_modules = ["links_and_nodes"]

    def register_view(self, view):
        super(SourceEditorController, self).register_view(view)

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
    def code_changed(self, source):
        self.view.apply_tag('default')
        
    def open_external_clicked(self, button):

        if button.get_active():

            # Get the specified "Editor" as in shell command from the gui config yaml
            external_editor = global_gui_config.get_config_value('DEFAULT_EXTERNAL_EDITOR')

            def open_file_in_editor(command, text_field):

                file_path = self.model.state.get_file_system_path()
                logger.debug("File opened with command: {}".format(command))

                # This splits the command in a matter so that the editor gets called in a separate shell and thus
                # doesnt lock the window.
                args = shlex.split(command + ' "' + file_path + '/script.py"')

                try:
                    subprocess.Popen(args)
                except OSError:

                    # This catches most of the errors being returned from the shell, destroys the old textfield and
                    # opens the dialog again, so the user can specify a new command. Its a bit dirty...
                    # The if is required to catch the case that a OSerror occurs due to other reasons than
                    # the specified command
                    logger.error('Syntax Error occurred')
                    if text_field:
                        text_field.destroy()
                    open_text_window()

                # Set the text on the button to 'Unlock' instead of 'Open external'
                button.set_label('Unlock')
                self.view.textview.set_sensitive(False)

            def open_text_window():
                from rafcon.mvc.utils.text_input import RAFCONTextInput
                entry_sample_text = "<shell command>"

                text_input = RAFCONTextInput(content=entry_sample_text)

                text_input.set_size_request(550, 50)
                text_input.set_title("Please select external editor")
                text_input.add_button('Apply', 1)
                text_input.add_button('Discard', 0)

                # Run the text_input Dialog until a response is emitted
                response = text_input.run()

                if response:

                    # If the response emitted from the Dialog is 1 than handle the 'OK'
                    global_gui_config.set_config_value('DEFAULT_EXTERNAL_EDITOR', text_input.return_text())
                    open_file_in_editor(text_input.return_text(), text_input)
                else:

                    # If Dialog is canceled either by the button or the cross, untoggle the button again and revert the
                    # lock, which is not implemented yet
                    button.set_active(False)

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

            # Load file contents after unlocking
            content = filesystem.read_file(self.model.state.get_file_system_path(), 'script.py')
            self.set_script_text(content)

            # If button is clicked after one open a file in the external editor, unlock the internal editor
            button.set_label('Open external')
            self.view.textview.set_sensitive(True)

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
        tbuffer = self.view.get_buffer()
        current_text = tbuffer.get_text(tbuffer.get_start_iter(), tbuffer.get_end_iter())
        if not self.view['pylint_check_button'].get_active():
            self.set_script_text(current_text)
            return

        logger.debug("Parsing execute script...")

        # do syntax-check on script
        text_file = open(self.tmp_file, "w")
        text_file.write(current_text)
        text_file.close()

        (pylint_stdout, pylint_stderr) = lint.py_run(
            self.tmp_file + " --errors-only --disable=print-statement ",
            True, script="epylint")
        # the extension-pkg-whitelist= parameter does not work for the no-member errors of links_and_nodes
        os.remove(self.tmp_file)

        pylint_stdout_data = pylint_stdout.readlines()
        pylint_stderr_data = pylint_stderr.readlines()

        invalid_sytax = False
        for elem in pylint_stdout_data:
            if "error" in elem:
                if self.filter_out_not_compatible_modules(elem):
                    invalid_sytax = True

        if invalid_sytax:

            def on_message_dialog_response_signal(widget, response_id, current_text):
                if response_id == 42:
                    self.set_script_text(current_text)
                else:
                    logger.debug("The script was not saved")
                widget.destroy()

            from rafcon.mvc.utils.dialog import RAFCONDialog
            dialog = RAFCONDialog(type=gtk.MESSAGE_WARNING, parent=self.get_root_window())
            message_string = "Are you sure that you want to save this file?\n\nThe following errors were found:"

            line = None
            for elem in pylint_stdout_data:
                if "error" in elem:
                    if self.filter_out_not_compatible_modules(elem):
                        (error_string, line, error) = self.format_error_string(str(elem))
                        message_string += "\n\n" + error_string

            # focus line of error
            if line:
                tbuffer = self.view.get_buffer()
                start_iter = tbuffer.get_start_iter()
                start_iter.set_line(int(line)-1)
                tbuffer.place_cursor(start_iter)
                message_string += "\n\nThe line was focused in the source editor."

            # select state to show source editor
            sm_m = state_machine_manager_model.get_sm_m_for_state_model(self.model)
            if sm_m.selection.get_selected_state() is not self.model:
                sm_m.selection.set(self.model)

            dialog.set_markup(message_string)
            dialog.add_button("Save with errors", 42)
            dialog.add_button("Do not save", 43)
            dialog.finalize(on_message_dialog_response_signal, current_text)
        else:
            self.set_script_text(current_text)

    def filter_out_not_compatible_modules(self, pylint_msg):
        """This method filters out every pylint message that addresses an error of a module that is explicitly ignored
        and added to  self.not_pylint_compatible_modules.

        :param pylint_msg: the pylint message to be filtered
        :return:
        """
        for elem in self.not_pylint_compatible_modules:
            if elem in pylint_msg:
                return False
        return True

    def format_error_string(self, error_string):
        error_string = error_string.replace(self.tmp_file, '', 1)
        error_parts = error_string.split(':')
        line = error_parts[1]
        error_parts = error_parts[2].split(')')
        error = error_parts[1]
        return "Line " + line + ": " + error, line, error
