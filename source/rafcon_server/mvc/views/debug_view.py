from gtkmvc import View
import gtk
import glib
import threading


class DebugView(View):
    builder = './glade/basic_server_debug.glade'
    top = "window"

    def __init__(self):
        View.__init__(self)

        self._lock = threading.Lock()

        liststore = gtk.ListStore(str, str, int)
        combobox = gtk.ComboBox(liststore)
        cell = gtk.CellRendererText()
        combobox.pack_start(cell, True)
        combobox.add_attribute(cell, 'text', 0)
        self["liststore"] = liststore
        self["combobox"] = combobox
        combobox.show()

        self["hbox1"].remove(self["combobox_placeholder"])
        self["hbox1"].pack_start(combobox, False, True, 0)
        self["hbox1"].reorder_child(combobox, 0)

        self.textview = self["textview"]

        self.quit_flag = False

    def apply_tag(self, name):
        self.textview.get_buffer().apply_tag_by_name(name,
                                                     self.textview.get_buffer().get_start_iter(),
                                                     self.textview.get_buffer().get_end_iter())

    # LOOK OUT: This will be called from several threads => make it thread safe
    def print_debug(self, text):
        glib.idle_add(self.print_add, text, self.textview.get_buffer())

    def print_error(self, text):
        glib.idle_add(self.print_add, text, self.textview.get_buffer())

    def print_info(self, text):
        glib.idle_add(self.print_add, text, self.textview.get_buffer())

    def print_warning(self, text):
        glib.idle_add(self.print_add, text, self.textview.get_buffer())

    def print_add(self, text_to_add, text_buf):
        text_to_add += "\n"
        self.print_push(text_to_add, text_buf)

    def print_push(self, text_to_push, text_buf):
        text_buf.insert(text_buf.get_end_iter(), text_to_push)

        if not self.quit_flag:
            self.textview.scroll_mark_onscreen(self.textview.get_buffer().get_insert())

    def print_message(self, message, log_level, new=True):
        self._lock.acquire()
        # Store all new log entries
        if log_level <= 10:
            self.print_debug(message)
        elif 10 < log_level <= 20:
            self.print_info(message)
        elif 20 < log_level <= 30:
            self.print_warning(message)
        elif 30 < log_level:
            self.print_error(message)
        self._lock.release()