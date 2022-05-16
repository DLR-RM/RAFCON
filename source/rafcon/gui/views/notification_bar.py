#  Copyright (C) 2019 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>

import logging
from threading import Timer

from rafcon.design_patterns.mvc.view import View

from gi.repository import Gtk
from gi.repository import GLib

from rafcon.gui.config import global_gui_config as gui_config
from rafcon.gui.helpers import label


class NotificationBarView(View):

    timer = None

    def __init__(self):
        super().__init__(parent='notification_bar')

        self['notification_bar'] = self.notification_bar = Gtk.Revealer()
        self.notification_bar.set_transition_type(Gtk.RevealerTransitionType.SLIDE_UP)
        self.notification_bar.set_transition_duration(400)

        self.info_bar = Gtk.InfoBar()
        self.info_bar.set_show_close_button(True)
        self.notification_bar.add(self.info_bar)

        self._message_label = Gtk.Label(label="test")
        content_area = self.info_bar.get_content_area()
        content_area.add(self._message_label)
        label.ellipsize_labels_recursively(content_area)

    def show_bar(self):
        if not self.notification_bar.get_reveal_child():
            self.notification_bar.set_reveal_child(True)

    def hide_bar(self):
        self.notification_bar.set_reveal_child(False)

    def show_notification(self, message, log_level):
        GLib.idle_add(self._show_notification, message, log_level, priority=GLib.PRIORITY_LOW)

    def _show_notification(self, message, log_level):
        self._set_corresponding_message_type(log_level)
        self._message_label.set_label(message)
        duration = gui_config.get_config_value("NOTIFICATIONS_DURATION", 3.)
        self._show_temporarily(duration)

    def _set_corresponding_message_type(self, log_level):
        if log_level >= logging.ERROR:
            self.info_bar.set_message_type(Gtk.MessageType.ERROR)
        elif log_level >= logging.WARNING:
            self.info_bar.set_message_type(Gtk.MessageType.WARNING)
        elif log_level >= logging.INFO:
            self.info_bar.set_message_type(Gtk.MessageType.INFO)
        else:
            self.info_bar.set_message_type(Gtk.MessageType.OTHER)

    def _show_temporarily(self, duration=3.):
        def hide_me():
            self.hide_bar()
        if self.timer:
            self.timer.cancel()
        self.show_bar()
        if duration > 0:
            self.timer = Timer(duration, hide_me)
            self.timer.start()
