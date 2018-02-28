# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import gtk
import threading

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

    def print_message(self, message, log_level, new=True):
        if self.view is None:
            return
        # Store all new log entries
        if new:
            self._lock.acquire()
            self._log_entries.append((log_level, message))
            self._lock.release()
        self.view.print_message(message, log_level)

    def print_filtered_buffer(self):
        self.view.clean_buffer()

        for entry in self._log_entries:
            level = entry[0]
            message = entry[1]
            self.print_message(message, level, new=False)

    def update_filtered_buffer(self):
        if self.view is None:
            return
        self.print_filtered_buffer()

        self.view.text_view.scroll_mark_onscreen(self.view.text_view.get_buffer().get_insert())

    def _clear_buffer(self, widget, data=None):
        self._log_entries = []
        self.print_filtered_buffer()
        self.view.text_view.scroll_mark_onscreen(self.view.text_view.get_buffer().get_insert())

    def add_clear_menu_item(self, widget, menu):
        clear_item = gtk.MenuItem("Clear Logging View")
        clear_item.connect('activate', self._clear_buffer)
        menu.append(gtk.SeparatorMenuItem())
        menu.append(clear_item)
        menu.show_all()

    def _get_config_enables(self):
        keys = ['VERBOSE', 'DEBUG', 'INFO', 'WARNING', 'ERROR']
        return {key: self.model.config.get_config_value('LOGGING_SHOW_' + key, True) for key in keys}

    @ExtendedController.observe("config", after=True)
    def model_changed(self, model, prop_name, info):
        """ React to configuration changes """
        current_enables = self._get_config_enables()
        if not self._enables == current_enables:
            self._enables = current_enables
            self.view.set_enables(self._enables)
            self.update_filtered_buffer()
