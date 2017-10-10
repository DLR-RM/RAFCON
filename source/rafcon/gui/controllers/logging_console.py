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

from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.utils import log, log_helpers
logger = log.get_logger(__name__)


class LoggingConsoleController(ExtendedController):
    """Controller handling the updates and modifications of the logging console.

    :param
    :param rafcon.mvc.views.logging.LoggingView view: The GTK view showing the source editor.
    """

    def __init__(self, model, view):
        ExtendedController.__init__(self, model, view)

        self._lock = threading.Lock()
        self._log_entries = []

        log_helpers.LoggingViewHandler.add_logging_view('main', self)

    def register_view(self, view):
        view.text_view.connect('populate_popup', self.add_clear_menu_item)

        self.update_filtered_buffer()

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.mvc.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
            between shortcuts and actions.
        """
        # shortcut_manager.add_callback_for_action("copy", self._copy)
        # shortcut_manager.add_callback_for_action("paste", self._paste)
        pass

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
        self.view.read_enables()

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
