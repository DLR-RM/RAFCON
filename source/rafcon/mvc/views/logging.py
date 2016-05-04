import gtk
import threading
from gtkmvc import View
import glib
from rafcon.mvc.utils import constants
from rafcon.utils import log
from rafcon.mvc.config import global_gui_config


class LoggingView(View):
    top = 'main_frame'

    def __init__(self):
        View.__init__(self)

        self._lock = threading.Lock()
        self._log_entries = []

        self.text_view = gtk.TextView()
        self.text_view.set_property('editable', False)

        self.filtered_buffer = self.create_text_buffer()

        self.text_view.set_buffer(self.filtered_buffer)

        self.text_view.set_border_width(10)

        scrollable = gtk.ScrolledWindow()
        scrollable.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        scrollable.set_name('console_scroller')
        scrollable.add(self.text_view)
        self.text_view.show()

        self.info = global_gui_config.get_config_value('LOGGING_SHOW_INFO', True)
        self.debug = global_gui_config.get_config_value('LOGGING_SHOW_DEBUG', True)
        self.warning = global_gui_config.get_config_value('LOGGING_SHOW_WARNING', True)
        self.error = global_gui_config.get_config_value('LOGGING_SHOW_ERROR', True)

        self['scrollable'] = scrollable
        self.quit_flag = False

        self.text_view.connect('populate_popup', self.add_clear_menu_item)

        log.register_logging_view('main', self)

    def add_clear_menu_item(self, widget, menu):
        clear_item = gtk.MenuItem("Clear Logging View")
        separator_item = gtk.SeparatorMenuItem()
        menu.append(separator_item)
        menu.append(clear_item)
        clear_item.connect('activate', self._clear_buffer)
        separator_item.show()
        clear_item.show()

    def _clear_buffer(self, widget, data=None):
        self._log_entries = []
        self.print_filtered_buffer()
        self.text_view.scroll_mark_onscreen(self.text_view.get_buffer().get_insert())

    def print_message(self, message, log_level, new=True):
        # return
        self._lock.acquire()
        # Store all new log entries
        if new:
            self._log_entries.append((log_level, message))
        if log_level <= 10 and self.debug:
            glib.idle_add(self.print_to_text_view, message, self.filtered_buffer, "set_debug_color",
                          priority=glib.PRIORITY_LOW)
        elif 10 < log_level <= 20 and self.info:
            glib.idle_add(self.print_to_text_view, message, self.filtered_buffer, "set_info_color",
                          priority=glib.PRIORITY_LOW)
        elif 20 < log_level <= 30 and self.warning:
            glib.idle_add(self.print_to_text_view, message, self.filtered_buffer, "set_warning_color",
                          priority=glib.PRIORITY_LOW)
        elif 30 < log_level and self.error:
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
        time, rest = text_to_split.split(': ', 1)
        source, message = rest.split(':', 1)
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

    def update_filtered_buffer(self):
        self.info = global_gui_config.get_config_value('LOGGING_SHOW_INFO', True)
        self.debug = global_gui_config.get_config_value('LOGGING_SHOW_DEBUG', True)
        self.warning = global_gui_config.get_config_value('LOGGING_SHOW_WARNING', True)
        self.error = global_gui_config.get_config_value('LOGGING_SHOW_ERROR', True)

        self.print_filtered_buffer()

        self.text_view.scroll_mark_onscreen(self.text_view.get_buffer().get_insert())

    def print_filtered_buffer(self):
        self.text_view.set_buffer(self.filtered_buffer)

        start, end = self.filtered_buffer.get_bounds()
        self.filtered_buffer.delete(start, end)

        for entry in self._log_entries:
            level = entry[0]
            message = entry[1]
            self.print_message(message, level, new=False)
