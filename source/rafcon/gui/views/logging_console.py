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
# Sebastian Brunner <sebastian.brunner@dlr.de>

import gtk
import threading
from gtkmvc import View
import glib
from rafcon.utils import log


class LoggingConsoleView(View):

    def __init__(self):
        View.__init__(self)

        self._lock = threading.Lock()

        self.text_view = gtk.TextView()
        self.text_view.set_property('editable', False)

        self.filtered_buffer = self.create_text_buffer()

        self.text_view.set_buffer(self.filtered_buffer)

        self.text_view.set_border_width(10)

        self._enables = {}

        scrollable = gtk.ScrolledWindow()
        scrollable.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        scrollable.set_name('console_scroller')
        scrollable.add(self.text_view)
        self.text_view.show()

        self['scrollable'] = scrollable
        self.top = 'scrollable'
        self.quit_flag = False

    def clean_buffer(self):
        self.text_view.set_buffer(self.filtered_buffer)

        start, end = self.filtered_buffer.get_bounds()
        self.filtered_buffer.delete(start, end)

    def print_message(self, message, log_level):
        self._lock.acquire()
        if log_level <= log.logging.DEBUG and self._enables.get('DEBUG', True):
            glib.idle_add(self.print_to_text_view, message, self.filtered_buffer, "set_debug_color",
                          priority=glib.PRIORITY_LOW)
        elif log.logging.DEBUG < log_level <= log.logging.INFO and self._enables.get('INFO', True):
            glib.idle_add(self.print_to_text_view, message, self.filtered_buffer, "set_info_color",
                          priority=glib.PRIORITY_LOW)
        elif log.logging.INFO < log_level <= log.logging.WARNING and self._enables.get('WARNING', True):
            glib.idle_add(self.print_to_text_view, message, self.filtered_buffer, "set_warning_color",
                          priority=glib.PRIORITY_LOW)
        elif log.logging.WARNING < log_level and self._enables.get('ERROR', True):
            glib.idle_add(self.print_to_text_view, message, self.filtered_buffer, "set_error_color",
                          priority=glib.PRIORITY_LOW)
        self._lock.release()

    def print_to_text_view(self, text, text_buf, use_tag=None):
        time, source, message = self.split_text(text)
        text_buf.insert_with_tags_by_name(text_buf.get_end_iter(), time + " ", "set_gray_text")
        text_buf.insert_with_tags_by_name(text_buf.get_end_iter(), source + ": ", "set_white_text")
        if use_tag:
            if self.text_view.get_buffer().get_tag_table().lookup(use_tag) is not None:
                text_buf.insert_with_tags_by_name(text_buf.get_end_iter(), message + "\n", use_tag)
            else:
                text_buf.insert(text_buf.get_end_iter(), message + "\n")
        else:
            text_buf.insert(text_buf.get_end_iter(), message + "\n")

        if not self.quit_flag:
            self.text_view.scroll_mark_onscreen(self.text_view.get_buffer().get_insert())

    @staticmethod
    def split_text(text_to_split):
        """Split text

        Splits the debug text into its different parts: 'Time', 'LogLevel + Module Name', 'Debug message'

        :param text_to_split: Text to split
        :return: List containing the content of text_to_split split up
        """
        assert isinstance(text_to_split, (str, unicode))
        try:
            time, rest = text_to_split.split(': ', 1)
            source, message = rest.split(':', 1)
        except ValueError:
            time = source = ""
            message = text_to_split
        return time.strip(), source.strip(), message.strip()

    @staticmethod
    def create_text_buffer():
        text_buffer = gtk.TextBuffer()
        text_buffer.create_tag("default", font="Monospace 10")
        text_buffer.create_tag("set_warning_color", foreground="orange")
        text_buffer.create_tag("set_error_color", foreground="red")
        text_buffer.create_tag("set_debug_color", foreground="#00baf8")
        text_buffer.create_tag("set_info_color", foreground="#39af57")
        text_buffer.create_tag("set_gray_text", foreground="#93959a")
        text_buffer.create_tag("set_white_text", foreground="#ffffff")
        return text_buffer

    def set_enables(self, enables):
        self._enables = enables
