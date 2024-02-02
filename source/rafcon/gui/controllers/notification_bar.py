#  Copyright (C) 2019 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>

import time

from gi.repository import Gtk

from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.utils import log, log_helpers
from rafcon.gui.config import global_gui_config
logger = log.get_logger(__name__)


class NotificationBarController(ExtendedController):
    """Controller showing notifications for important log messages"""

    def __init__(self, model, view):
        super(NotificationBarController, self).__init__(model, view)
        log_helpers.LoggingViewHandler.add_logging_view(self.__class__.__name__, self)
        self.last_notification_timestamp = None

    def register_view(self, view):
        super(NotificationBarController, self).register_view(view)
        self.connect_signal(view.info_bar, "response", self._handle_response)

    def destroy(self):
        log_helpers.LoggingViewHandler.remove_logging_view(self.__class__.__name__)
        if self.view.timer:
            self.view.timer.cancel()
            self.view.timer = None
        super(NotificationBarController, self).destroy()

    def print_message(self, message, log_level):
       if global_gui_config.get_config_value('LOGGING_SHOW_POPUP', True):
            if self.view is None:
                return
            if not self._handle_log_level(log_level):
                return
            message_text = ": ".join(message.split(": ")[2:])  # remove time, log level and source
            now = time.time() * 1000
            if self.last_notification_timestamp is None or self.last_notification_timestamp + 100 < now:
                self.view.show_notification(message_text, log_level)
            self.last_notification_timestamp = now

    def _handle_log_level(self, log_level):
        minimum_low_level = self.model.config.get_config_value("NOTIFICATIONS_MINIMUM_LOG_LEVEL", 30)
        return log_level >= minimum_low_level

    def _handle_response(self, widget, response_id):
        if response_id == Gtk.ResponseType.CLOSE:
            if self.view.timer:
                self.view.timer.cancel()
            self.view.hide_bar()