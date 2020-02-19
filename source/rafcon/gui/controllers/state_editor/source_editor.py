# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: source_editor
   :synopsis: A module holds the controller to edit the state source script text.

"""

from future import standard_library
standard_library.install_aliases()
import os
from gi.repository import Gtk
import contextlib
from pylint import lint
try:
    from pylint.reporters.json import JSONReporter
except ModuleNotFoundError:
    from pylint.reporters.json_reporter import JSONReporter
from io import StringIO
from astroid import MANAGER
from pkg_resources import resource_filename

import rafcon
from rafcon.core.states.library_state import LibraryState
from rafcon.core.storage import storage

from rafcon.gui.controllers.utils.editor import EditorController
from rafcon.gui.singleton import state_machine_manager_model
from rafcon.gui.config import global_gui_config
from rafcon.gui.utils.dialog import RAFCONButtonDialog, RAFCONInputDialog
from rafcon.gui.models import AbstractStateModel, LibraryStateModel
from rafcon.gui.views.state_editor.source_editor import SourceEditorView
from rafcon.gui.utils.external_editor import AbstractExternalEditor

from rafcon.utils import filesystem
from rafcon.utils.constants import RAFCON_TEMP_PATH_STORAGE
from rafcon.utils import log

logger = log.get_logger(__name__)


class SourceEditorController(EditorController, AbstractExternalEditor):
    """Controller handling the source editor in Execution States.

    :param
    :param rafcon.gui.views.source_editor.SourceEditorView view: The GTK view showing the source editor.
    """
    # TODO Missing functions
    # - Code function-expander
    # - Code completion

    tmp_file = os.path.join(RAFCON_TEMP_PATH_STORAGE, 'file_to_get_pylinted.py')

    def __init__(self, model, view):
        assert isinstance(model, AbstractStateModel)
        assert isinstance(view, SourceEditorView)
        lib_with_show_content = isinstance(model, LibraryStateModel) and not model.show_content()
        model = model.state_copy if lib_with_show_content else model

        # unfortunately this cannot be down with super, as gtkmvc3 does not use super() consistently
        EditorController.__init__(self, model, view, observed_method="script_text")
        AbstractExternalEditor.__init__(self)
        self.saved_initial = False

    def register_view(self, view):
        super(SourceEditorController, self).register_view(view)

        view['open_external_button'].connect('clicked', self.open_externally_clicked)
        view['apply_button'].connect('clicked', self.apply_clicked)
        view['cancel_button'].connect('clicked', self.cancel_clicked)

        view['pylint_check_button'].set_active(global_gui_config.get_config_value('CHECK_PYTHON_FILES_WITH_PYLINT', False))

        view['pylint_check_button'].set_tooltip_text("Change global default value in GUI config.")
        view['apply_button'].set_tooltip_text(global_gui_config.get_config_value('SHORTCUTS')['apply'][0])
        view['open_external_button'].set_tooltip_text("Open source in external editor. " +
                                                      global_gui_config.get_config_value('SHORTCUTS')['open_external_editor'][0])

        if isinstance(self.model.state, LibraryState) or self.model.state.get_next_upper_library_root_state():
            view['pylint_check_button'].set_sensitive(False)
            view.textview.set_editable(False)
            view['apply_button'].set_sensitive(False)
            view['cancel_button'].set_sensitive(False)
            view['open_external_button'].set_sensitive(False)

    @property
    def source_text(self):
        return self.model.state.script_text

    @source_text.setter
    def source_text(self, text):
        try:
            self.model.state.script_text = text
        except Exception as e:
            logger.error("The script was saved, but cannot be compiled: {}: {}".format(e.__class__.__name__, e))

    # ==================================== External Editor functions start ====================================

    def get_file_name(self):
        """ Implements the abstract method of the ExternalEditor class.
        """
        return storage.SCRIPT_FILE

    def save_file_data(self, path):
        """ Implements the abstract method of the ExternalEditor class.
        """
        sm = self.model.state.get_state_machine()
        if sm.marked_dirty and not self.saved_initial:
            try:
                # Save the file before opening it to update the applied changes. Use option create_full_path=True
                # to assure that temporary state_machines' script files are saved to
                # (their path doesnt exist when not saved)
                filesystem.write_file(os.path.join(path, storage.SCRIPT_FILE),
                                      self.view.get_text(), create_full_path=True)
            except IOError as e:
                # Only happens if the file doesnt exist yet and would be written to the temp folder.
                # The method write_file doesnt create the path
                logger.error('The operating system raised an error: {}'.format(e))
            self.saved_initial = True

    def set_editor_lock(self, lock):
        """ Implements the abstract method of the ExternalEditor class.
        """
        self.view.set_enabled(not lock)

    def load_and_set_file_content(self, file_system_path):
        """ Implements the abstract method of the ExternalEditor class.
        """
        content = filesystem.read_file(file_system_path, storage.SCRIPT_FILE)
        if content is not None:
            self.set_script_text(content)

    # ==================================== External Editor functions end ====================================

    def apply_clicked(self, button):
        """Triggered when the Apply button in the source editor is clicked.

        """
        if isinstance(self.model.state, LibraryState):
            logger.warning("It is not allowed to modify libraries.")
            self.view.set_text("")
            return

        # Ugly workaround to give user at least some feedback about the parser
        # Without the loop, this function would block the GTK main loop and the log message would appear after the
        # function has finished
        # TODO: run parser in separate thread
        while Gtk.events_pending():
            Gtk.main_iteration_do(False)

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
        lint_config_file = resource_filename(rafcon.__name__, "pylintrc")
        args = ["--rcfile={}".format(lint_config_file)]  # put your own here
        with contextlib.closing(StringIO()) as dummy_buffer:
            json_report = JSONReporter(dummy_buffer.getvalue())
            try:
                lint.Run([self.tmp_file] + args, reporter=json_report, exit=False)
            except:
                logger.exception("Could not run linter to check script")
        os.remove(self.tmp_file)

        if json_report.messages:
            def on_message_dialog_response_signal(widget, response_id):
                if response_id == 1:
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
                self.view.scroll_to_cursor_onscreen()

            # select state to show source editor
            sm_m = state_machine_manager_model.get_state_machine_model(self.model)
            if sm_m.selection.get_selected_state() is not self.model:
                sm_m.selection.set(self.model)

            dialog = RAFCONButtonDialog(message_string, ["Save with errors", "Do not save"],
                               on_message_dialog_response_signal,
                               message_type=Gtk.MessageType.WARNING, parent=self.get_root_window())
            result = dialog.run()
        else:
            self.set_script_text(current_text)

    @staticmethod
    def format_error_string(message):
        return "Line {}: {} ({})".format(message["line"], message["message"], message["symbol"]), message["line"]
