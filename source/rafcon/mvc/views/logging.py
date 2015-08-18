import gtk
from gtkmvc import View
import glib
from rafcon.utils import constants

from rafcon.mvc.config import global_gui_config


class LoggingView(View):
    top = 'main_frame'

    def __init__(self):
        View.__init__(self)

        # create textview
        self.textview = gtk.TextView()
        self.textview.set_property('editable', False)

        self.complete_buffer = self.create_text_buffer()
        self.filtered_buffer = self.create_text_buffer()

        self.textview.set_buffer(self.complete_buffer)

        self.textview.set_border_width(constants.BORDER_WIDTH_TEXTVIEW)

        scrollable = gtk.ScrolledWindow()
        scrollable.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        scrollable.add(self.textview)
        self.textview.show()

        self.info = global_gui_config.get_config_value('LOGGING_SHOW_INFO', True)
        self.debug = global_gui_config.get_config_value('LOGGING_SHOW_DEBUG', True)
        self.warning = global_gui_config.get_config_value('LOGGING_SHOW_WARNING', True)
        self.error = global_gui_config.get_config_value('LOGGING_SHOW_ERROR', True)

        self['scrollable'] = scrollable
        self.quit_flag = False

        self.textview.connect('populate_popup', self.add_clear_menu_item)

    def add_clear_menu_item(self, widget, menu):
        clear_item = gtk.MenuItem("Clear Logging View")
        separator_item = gtk.SeparatorMenuItem()
        menu.append(separator_item)
        menu.append(clear_item)
        clear_item.connect('activate', self._clear_buffer)
        separator_item.show()
        clear_item.show()

    def _clear_buffer(self, widget, data=None):
        self.complete_buffer.set_text("")
        self.update_filtered_buffer()

    def apply_tag(self, name):
        self.textview.get_buffer().apply_tag_by_name(name,
                                                     self.textview.get_buffer().get_start_iter(),
                                                     self.textview.get_buffer().get_end_iter())

    # LOOK OUT: This will be called from several threads => make it thread safe
    def print_debug(self, text):
        glib.idle_add(self.print_add, text, self.complete_buffer, "set_debug_color")
        if self.debug:
            glib.idle_add(self.print_add, text, self.filtered_buffer, "set_debug_color")

    def print_error(self, text):
        glib.idle_add(self.print_add, text, self.complete_buffer, "set_error_color")
        if self.error:
            glib.idle_add(self.print_add, text, self.filtered_buffer, "set_error_color")

    def print_info(self, text):
        glib.idle_add(self.print_add, text, self.complete_buffer, "set_info_color")
        if self.info:
            glib.idle_add(self.print_add, text, self.filtered_buffer, "set_info_color")

    def print_warning(self, text):
        glib.idle_add(self.print_add, text, self.complete_buffer, "set_warning_color")
        if self.warning:
            glib.idle_add(self.print_add, text, self.filtered_buffer, "set_warning_color")

    # def get_buffer(self):
    #     return self.textview.get_buffer()

    # def set_text(self, text):
    #     self.textview.get_buffer().set_text(text)

    def print_add(self, text_to_add, text_buf, use_tag=None):
        text_to_add += "\n"
        self.print_push(text_to_add, text_buf, use_tag)

    def print_push(self, text_to_push, text_buf, use_tag=None):
        text_to_push = self.split_text(text_to_push)
        text_buf.insert_with_tags_by_name(text_buf.get_end_iter(), text_to_push[0], "set_gray_text")
        text_buf.insert_with_tags_by_name(text_buf.get_end_iter(), text_to_push[1], "set_white_text")
        if use_tag:
            if self.textview.get_buffer().get_tag_table().lookup(use_tag) is not None:
                text_buf.insert_with_tags_by_name(text_buf.get_end_iter(), text_to_push[2], use_tag)
            else:
                text_buf.insert(text_buf.get_end_iter(), text_to_push[2])
        else:
            text_buf.insert(text_buf.get_end_iter(), text_to_push[2])

        if not self.quit_flag:
            # print self.quit_flag
            # self.textview.scroll_to_iter(text_buf.get_end_iter(), 0.0, use_align=True, yalign=0.0)
            self.textview.scroll_mark_onscreen(self.textview.get_buffer().get_insert())

    def split_text(self, text_to_split):
        """
        Splits the debug text into its different parts: 'Time', 'LogLevel + Module Name', 'Debug message'
        :param text_to_split: Text to split
        :return: Array containing the content of text_to_split split up
        """
        assert isinstance(text_to_split, (str, unicode))
        first_separation = text_to_split.find(": ") + 1
        splitt = [text_to_split[:first_separation]]
        second_separation = text_to_split.find(":", first_separation) + 1
        splitt.append(text_to_split[first_separation:second_separation])
        splitt.append(text_to_split[second_separation:])
        return splitt

    def create_text_buffer(self):
        buffer = gtk.TextBuffer()
        buffer.create_tag("default", font="Monospace 10")
        buffer.create_tag("set_warning_color", foreground="orange")
        buffer.create_tag("set_error_color", foreground="red")
        buffer.create_tag("set_debug_color", foreground="#00baf8")
        buffer.create_tag("set_info_color", foreground="#39af57")
        buffer.create_tag("set_gray_text", foreground="#93959a")
        buffer.create_tag("set_white_text", foreground="#ffffff")
        return buffer

    def update_filtered_buffer(self):
        self.info = global_gui_config.get_config_value('LOGGING_SHOW_INFO', True)
        self.debug = global_gui_config.get_config_value('LOGGING_SHOW_DEBUG', True)
        self.warning = global_gui_config.get_config_value('LOGGING_SHOW_WARNING', True)
        self.error = global_gui_config.get_config_value('LOGGING_SHOW_ERROR', True)

        if self.info and self.debug and self.warning and self.error:
            self.textview.set_buffer(self.complete_buffer)
        else:
            self.print_filtered_buffer()

        self.textview.scroll_mark_onscreen(self.textview.get_buffer().get_insert())

    def print_filtered_buffer(self):
        self.textview.set_buffer(self.filtered_buffer)

        start, end = self.filtered_buffer.get_bounds()
        self.filtered_buffer.delete(start, end)

        has_line = True
        iter1 = self.complete_buffer.get_start_iter()
        iter2 = self.complete_buffer.get_start_iter()
        while has_line:
            iter2.forward_line()
            line = self.complete_buffer.get_text(iter1, iter2)
            has_line = iter1.forward_line()
            if line.find("INFO") != -1 and self.info:
                self.print_push(line, self.filtered_buffer, "set_info_color")
            elif line.find("DEBUG") != -1 and self.debug:
                self.print_push(line, self.filtered_buffer, "set_debug_color")
            elif line.find("WARNING") != -1 and self.warning:
                self.print_push(line, self.filtered_buffer, "set_warning_color")
            elif line.find("ERROR") != -1 and self.error:
                self.print_push(line, self.filtered_buffer, "set_error_color")

    # def print_clean(self, keep_lines=0):
    #     text_buf = self.textview.get_buffer()
    #     text = ""
    #     self.textview.get_buffer().set_text(text)