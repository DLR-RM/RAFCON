import gtk
import os
from pylint import epylint as lint

from rafcon.utils.constants import RAFCON_TEMP_PATH_STORAGE

from rafcon.mvc.controllers.extended_controller import ExtendedController
from rafcon.statemachine.states.library_state import LibraryState

from rafcon.utils import log

logger = log.get_logger(__name__)


class SourceEditorController(ExtendedController):
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
        ExtendedController.__init__(self, model, view)
        self.not_pylint_compatible_modules = ["links_and_nodes"]
        # self.is_dirty = False

    def register_view(self, view):
        view.get_buffer().connect('changed', self.code_changed)
        view['apply_button'].connect('clicked', self.apply_clicked)
        view['cancel_button'].connect('clicked', self.cancel_clicked)
        view.get_buffer().begin_not_undoable_action()
        view.set_text(self.model.state.script.script)
        view.get_buffer().end_not_undoable_action()

        if isinstance(self.model.state, LibraryState):
            view.textview.set_sensitive(False)
            view['apply_button'].set_sensitive(False)
            view['cancel_button'].set_sensitive(False)

    def register_adapters(self):
        pass

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.mvc.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
            between shortcuts and actions.
        """
        shortcut_manager.add_callback_for_action("copy", self._copy)
        shortcut_manager.add_callback_for_action("paste", self._paste)
        shortcut_manager.add_callback_for_action("cut", self._cut)
        shortcut_manager.add_callback_for_action("undo", self._undo)
        shortcut_manager.add_callback_for_action("redo", self._redo)

    def _copy(self, *args):
        pass

    def _paste(self, *args):
        pass

    def _cut(self, *args):
        pass

    def _undo(self, *args):
        buffer = self.view.textview.get_buffer()
        if self.view.textview.is_focus() and buffer.can_undo():
            logger.debug('run Undo on script editor')
            return buffer.undo()
        else:
            return False

    def _redo(self, *args):
        buffer = self.view.textview.get_buffer()
        if self.view.textview.is_focus() and buffer.can_redo():
            logger.debug('run Redo on script editor')
            return buffer.redo()
        else:
            return False

    # ===============================================================
    def code_changed(self, source):
        # print "The text in the text_buffer changed"
        # tbuffer = self.view.get_buffer()
        # current_text = tbuffer.get_text(tbuffer.get_start_iter(), tbuffer.get_end_iter())
        # if self.model.state.script.script == current_text:
        #     self.is_dirty = False
        # else:
        #     self.is_dirty = True
        # print "script is dirty: {0} {1}".format(self.is_dirty, source)
        self.view.apply_tag('default')

    def apply_clicked(self, button):
        """Triggered when the Apply button in the source editor is clicked.

        """
        if isinstance(self.model.state, LibraryState):
            logger.warn("It is not allowed to modify libraries.")
            self.view.set_text("")
            return

        logger.debug("Apply button pressed!")

        ###############
        # get script
        tbuffer = self.view.get_buffer()
        current_text = tbuffer.get_text(tbuffer.get_start_iter(), tbuffer.get_end_iter())

        ###############
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

        logger.debug("pylint_stdout_data: %s" % pylint_stdout_data)
        logger.debug("pylint_stderr: %s" % pylint_stderr_data)

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
                    logger.debug("Source script is not stored to memory")
                widget.destroy()

            from rafcon.mvc.utils.dialog import RAFCONDialog
            dialog = RAFCONDialog(type=gtk.MESSAGE_WARNING)
            message_string = "Are you sure that you want to save this file?\n\nThe following errors were found:"
            for elem in pylint_stdout_data:
                if "error" in elem:
                    if self.filter_out_not_compatible_modules(elem):
                        error_string = self.format_error_string(str(elem))
                        message_string += "\n\n" + error_string
            dialog.set_markup(message_string)
            dialog.add_button("Save with errors", 42)
            dialog.add_button("Do not save", 43)
            dialog.finalize(on_message_dialog_response_signal, current_text)
        else:
            self.set_script_text(current_text)

    def set_script_text(self, text):
        try:
            if self.model.state.set_script_text(text):
                logger.debug("Source script is stored to memory")
                return True
            else:
                logger.debug("Source script is not stored to memory")
        except TypeError as e:
            logger.warning("Source script is not stored to memory: {0}".format(e))

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
        return "Line " + line + ": " + error

    def cancel_clicked(self, button):
        """Triggered when the Cancel button in the source editor is clicked

        Resets the code in the editor to the last-saved code.
        """
        self.view.set_text(self.model.state.script.script)

    @ExtendedController.observe("state", after=True)
    def after_notification_of_script_text_was_changed(self, model, prop_name, info):

        if hasattr(info, "method_name") and "set_script_text" == info.method_name:
            # logger.debug('after_notification_of_script_text_was_changed' + str(info) + "\n" +
            # self.model.state.script.script)
            self.view.set_text(self.model.state.script.script)
        if hasattr(info, "method_name") and "script" == info.method_name:
            # logger.debug('after_notification_of_script_was_exchanged' + str(info) + "\n" +
            # self.model.state.script.script)
            self.view.set_text(self.model.state.script.script)
