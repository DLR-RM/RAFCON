# Copyright (C) 2015-2018 DLR
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
from gtkmvc import View
import glib
from rafcon.utils import log
logger = log.get_logger(__name__)


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
        self._auto_scroll_handler_id = None

        scrollable = gtk.ScrolledWindow()
        scrollable.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        scrollable.set_name('console_scroller')
        scrollable.add(self.text_view)
        self.text_view.show()

        self['scrollable'] = scrollable
        self.top = 'scrollable'
        self.quit_flag = False

        from rafcon.gui.config import global_gui_config
        self.logging_priority = global_gui_config.get_config_value("LOGGING_CONSOLE_GTK_PRIORITY", glib.PRIORITY_LOW)

        self._stored_line_number = None
        self._stored_line_offset = None
        self._stored_text_of_line = None
        self._stored_relative_lines = None

    def clean_buffer(self):
        self.text_view.set_buffer(self.filtered_buffer)

        start, end = self.filtered_buffer.get_bounds()
        self.filtered_buffer.delete(start, end)

    def print_message(self, message, log_level):
        self._lock.acquire()
        if log_level <= log.logging.VERBOSE and self._enables.get('VERBOSE', False):
            glib.idle_add(self.print_to_text_view, message, self.filtered_buffer, "set_debug_color",
                          priority=glib.PRIORITY_LOW)
        if log.logging.VERBOSE < log_level <= log.logging.DEBUG and self._enables.get('DEBUG', True):
            glib.idle_add(self.print_to_text_view, message, self.filtered_buffer, "set_debug_color",
                          priority=self.logging_priority)
        elif log.logging.DEBUG < log_level <= log.logging.INFO and self._enables.get('INFO', True):
            glib.idle_add(self.print_to_text_view, message, self.filtered_buffer, "set_info_color",
                          priority=self.logging_priority)
        elif log.logging.INFO < log_level <= log.logging.WARNING and self._enables.get('WARNING', True):
            glib.idle_add(self.print_to_text_view, message, self.filtered_buffer, "set_warning_color",
                          priority=self.logging_priority)
        elif log.logging.WARNING < log_level and self._enables.get('ERROR', True):
            glib.idle_add(self.print_to_text_view, message, self.filtered_buffer, "set_error_color",
                          priority=self.logging_priority)
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

        if not self.quit_flag and self._enables['CONSOLE_FOLLOW_LOGGING']:
            self.scroll_to_cursor_onscreen()

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
        self.update_auto_scroll_mode()

    def update_auto_scroll_mode(self):
        """ Register or un-register signals for follow mode """
        if self._enables['CONSOLE_FOLLOW_LOGGING']:
            if self._auto_scroll_handler_id is None:
                self._auto_scroll_handler_id = self.text_view.connect("size-allocate", self._auto_scroll)
        else:
            if self._auto_scroll_handler_id is not None:
                self.text_view.disconnect(self._auto_scroll_handler_id)
                self._auto_scroll_handler_id = None

    def _auto_scroll(self, *args):
        """ Scroll to the end of the text view """
        adj = self['scrollable'].get_vadjustment()
        adj.set_value(adj.get_upper() - adj.get_page_size())

    def scroll_to_cursor_onscreen(self):
        self.text_view.scroll_mark_onscreen(self.text_view.get_buffer().get_insert())

    def get_cursor_position(self):
        text_buffer = self.text_view.get_buffer()
        p_iter = text_buffer.get_iter_at_offset(text_buffer.props.cursor_position)
        return p_iter.get_line(), p_iter.get_line_offset()

    def set_cursor_position(self, line_number, line_offset):
        text_buffer = self.text_view.get_buffer()
        new_p_iter = text_buffer.get_iter_at_line(line_number)
        if new_p_iter.get_chars_in_line() >= line_offset:
            new_p_iter = text_buffer.get_iter_at_line_offset(line_number, line_offset)
        else:
            logger.debug("Line has not enough chars {0} {1}".format((line_number, line_offset), new_p_iter.get_chars_in_line()))
        if new_p_iter.is_cursor_position():
            result = text_buffer.place_cursor(new_p_iter)
        else:
            if not (line_offset == 0 and new_p_iter.get_chars_in_line() == 0):
                logger.debug("Line and offset is no cursor position line: {0} offset: {1} line length: {2}"
                             "".format(line_number, line_offset, new_p_iter.get_chars_in_line()))
            result = False

        self.text_view.scroll_mark_onscreen(self.text_view.get_buffer().get_insert())
        return result

    def len(self):
        text_buffer = self.text_view.get_buffer()
        return text_buffer.get_line_count()

    def get_text_of_line(self, line_number_or_iter):
        text_buffer = self.text_view.get_buffer()
        if isinstance(line_number_or_iter, gtk.TextIter):
            line_iter = line_number_or_iter
            line_end_iter = text_buffer.get_iter_at_line(line_iter.get_line())
        else:
            line_number = line_number_or_iter
            line_iter = text_buffer.get_iter_at_line(line_number)
            line_end_iter = text_buffer.get_iter_at_line(line_number)
        line_end_iter.forward_to_line_end()
        text = text_buffer.get_text(line_iter, line_end_iter)
        return text

    def set_cursor_on_line_with_string(self, s, line_offset=0):
        text_buffer = self.text_view.get_buffer()
        line_iter = text_buffer.get_iter_at_line(0)
        current_text_line = self.get_text_of_line(line_iter)
        while not s == current_text_line:
            if not line_iter.forward_line():
                return False
            current_text_line = self.get_text_of_line(line_iter)

        return self.set_cursor_position(line_iter.get_line(), line_offset)

    def get_line_number_next_to_cursor_with_string_within(self, s):
        """ Find the closest occurrence of a string with respect to the cursor position in the text view """
        line_number, _ = self.get_cursor_position()
        text_buffer = self.text_view.get_buffer()
        line_iter = text_buffer.get_iter_at_line(line_number)

        # find closest before line with string within
        before_line_number = None
        while line_iter.backward_line():
            if s in self.get_text_of_line(line_iter):
                before_line_number = line_iter.get_line()
                break

        # find closest after line with string within
        after_line_number = None
        while line_iter.forward_line():
            if s in self.get_text_of_line(line_iter):
                after_line_number = line_iter.get_line()
                break

        # take closest one to current position
        if after_line_number is not None and before_line_number is None:
            return after_line_number, after_line_number - line_number
        elif before_line_number is not None and after_line_number is None:
            return before_line_number, line_number - before_line_number
        elif after_line_number is not None and before_line_number is not None:
            after_distance = after_line_number - line_number
            before_distance = line_number - before_line_number
            if after_distance < before_distance:
                return after_line_number, after_distance
            else:
                return before_line_number, before_distance
        else:
            return None, None

    def store_cursor_position(self):
        self._stored_line_number, self._stored_line_offset = self.get_cursor_position()
        self._stored_text_of_line = self.get_text_of_line(self._stored_line_number)
        self._stored_relative_lines = []
        for key in self._enables.keys():
            if self._enables[key]:
                checked_line_number, distance = self.get_line_number_next_to_cursor_with_string_within(key)
                if checked_line_number is not None:
                    text_of_found_line = self.get_text_of_line(checked_line_number)
                    self._stored_relative_lines.append((distance, text_of_found_line))

    def restore_cursor_position(self):
        if self.get_text_of_line(self._stored_line_number) == self._stored_text_of_line:
            return self.set_cursor_position(self._stored_line_number, self._stored_line_offset)
        else:
            done = self.set_cursor_on_line_with_string(self._stored_text_of_line, self._stored_line_offset)
            if not done and self._stored_relative_lines:
                next_relative_lines = sorted(self._stored_relative_lines, key=lambda type_tuple: type_tuple[0])
                text_of_line = next_relative_lines[0][1]
                done = self.set_cursor_on_line_with_string(text_of_line, self._stored_line_offset)
            return done
