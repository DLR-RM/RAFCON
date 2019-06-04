# Copyright (C) 2017-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>

from gi.repository import Gtk
import threading

from rafcon.gui.utils import wait_for_gui, constants
from rafcon.gui.helpers.label import create_menu_item
from rafcon.gui.models.config_model import ConfigModel
from rafcon.gui.views.logging_console import LoggingConsoleView
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.utils import log, log_helpers
logger = log.get_logger(__name__)


class LoggingConsoleController(ExtendedController):
    """Controller handling the updates and modifications of the logging console.

    :param rafcon.gui.models.config_model.ConfigModel: Gui config model holding and observing the global gui config.
    :param rafcon.gui.views.logging_console.LoggingConsoleView view: The GTK view showing the logging messages.
    """

    def __init__(self, model, view):
        assert isinstance(model, ConfigModel)
        assert isinstance(view, LoggingConsoleView)
        super(LoggingConsoleController, self).__init__(model, view)

        self._lock = threading.Lock()
        self._log_entries = []
        self._enables = self._get_config_enables()
        log_helpers.LoggingViewHandler.add_logging_view('main', self)

    def register_view(self, view):
        super(LoggingConsoleController, self).register_view(view)
        view.text_view.connect('populate_popup', self.add_clear_menu_item)
        self.view.set_enables(self._enables)
        self.update_filtered_buffer()

    def destroy(self):
        self.view.quit_flag = True
        log_helpers.LoggingViewHandler.remove_logging_view('main')
        super(LoggingConsoleController, self).destroy()

    def print_message(self, message, log_level, new=True):
        if self.view is None:
            return
        # Store all new log entries
        if new:
            with self._lock:
                self._log_entries.append((log_level, message))
        self.view.print_message(message, log_level)

    def print_filtered_buffer(self):
        # remember cursor position
        self.view.store_cursor_position()

        # update text buffer
        self.view.clean_buffer()

        for entry in self._log_entries:
            level = entry[0]
            message = entry[1]
            self.print_message(message, level, new=False)

        # restore cursor position
        wait_for_gui()
        self.view.restore_cursor_position()

        self.view.scroll_to_cursor_onscreen()

    def update_filtered_buffer(self):
        if self.view is None:
            return
        self.print_filtered_buffer()

    def _clear_buffer(self, widget, data=None):
        self._log_entries = []
        self.print_filtered_buffer()

    def add_clear_menu_item(self, widget, menu):
        clear_item = create_menu_item("Clear Logging View", constants.BUTTON_DEL, callback=self._clear_buffer)
        menu.append(Gtk.SeparatorMenuItem())
        menu.append(clear_item)
        menu.show_all()

    def _get_config_enables(self):
        keys = ['VERBOSE', 'DEBUG', 'INFO', 'WARNING', 'ERROR']
        result = {key: self.model.config.get_config_value('LOGGING_SHOW_' + key, True) for key in keys}
        result['CONSOLE_FOLLOW_LOGGING'] = self.model.config.get_config_value('CONSOLE_FOLLOW_LOGGING', True)
        return result

    @ExtendedController.observe("config", after=True)
    def model_changed(self, model, prop_name, info):
        """ React to configuration changes

        Update internal hold enable state, propagates it to view and refresh the text buffer."""
        current_enables = self._get_config_enables()
        if not self._enables == current_enables:
            # check if filtered buffer update needed
            filtered_buffer_update_needed = True
            if all(self._enables[key] == current_enables[key] for key in ['VERBOSE', 'DEBUG', 'INFO', 'WARNING', 'ERROR']):
                follow_mode_key = 'CONSOLE_FOLLOW_LOGGING'
                only_follow_mode_changed = self._enables[follow_mode_key] != current_enables[follow_mode_key]
                filtered_buffer_update_needed = not only_follow_mode_changed

            self._enables = current_enables
            self.view.set_enables(self._enables)
            if filtered_buffer_update_needed:
                self.update_filtered_buffer()
            else:
                self.view.scroll_to_cursor_onscreen()
