"""
.. module:: source_editor
   :platform: Unix, Windows
   :synopsis: A module holds the controller to edit the state source script text.

.. moduleauthor:: Rico Belder


"""

import os

import gtk
from pylint import epylint as lint

from rafcon.statemachine.states.library_state import LibraryState

from rafcon.mvc.controllers.utils.editor import EditorController
from rafcon.mvc.singleton import state_machine_manager_model
from rafcon.mvc.config import global_gui_config

from rafcon.utils.constants import RAFCON_TEMP_PATH_STORAGE
from rafcon.utils import log

from rafcon.statemachine import interface

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

        #JUST FOR DEBUG PURPOSES
        global_gui_config.set_config_value('DEFAULT_EXTERNAL_EDITOR', None)

        view['open_external_button'].connect('clicked', self.open_external_clicked)
        view['apply_button'].connect('clicked', self.apply_clicked)
        view['cancel_button'].connect('clicked', self.cancel_clicked)

        view['pylint_check_button'].set_active(global_gui_config.get_config_value('CHECK_PYTHON_FILES_WITH_PYLINT', False))

        view['pylint_check_button'].set_tooltip_text("Change global default value in GUI config.")
        view['apply_button'].set_tooltip_text(global_gui_config.get_config_value('SHORTCUTS')['apply'][0])
        view['open_external_button'].set_tooltip_text("Open source in external editor.")

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
        logger.debug("Open_external was clicked")

        def on_message_dialog_response_signal(widget, response_id, path="gedit"):
                if response_id is 42:
                    global_gui_config.set_config_value('DEFAULT_EXTERNAL_EDITOR', path)
                    global_gui_config.save_configuration()

                widget.destroy()

        file_path = self.model.state.get_file_system_path()

        # global_gui_config.set_config_value
        external_editor = global_gui_config.get_config_value('DEFAULT_EXTERNAL_EDITOR')

        if not external_editor:
            logger.debug("No external editor specified")

            # from rafcon.mvc.utils.dialog import RAFCONDialog
            from rafcon.mvc.utils.text_input import RAFCONTextInput
            # dialog = RAFCONDialog(parent=self.get_root_window())
            dia = RAFCONTextInput()
            message_string = "Please select external editor"
            entry_sample_text = "<editor shell command>"
            dia.setup(message_string, entry_sample_text)

            # dialog.set_markup(message_string)
            # dialog.add_button("Set as default external editor", 42)
            # logger.debug(dia)
            # dialog.add_button("Cancel", 44)
            # dialog.finalize(on_message_dialog_response_signal, "gedit")
        else:
            logger.debug("File opened with command: " + external_editor)
            os.system(external_editor + " " + file_path + "/script.py")

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
