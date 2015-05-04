from gtkmvc import View
import gtk
import glib

from twisted.internet import reactor


class DebugView(View):
    top = "window"

    def __init__(self):
        View.__init__(self)

        window = gtk.Window(gtk.WINDOW_TOPLEVEL)

        window.set_size_request(800, 600)

        textview = gtk.TextView()
        textview.set_wrap_mode(gtk.WRAP_WORD)
        textview.set_editable(False)
        textview.set_cursor_visible(False)
        textview.show()

        scroller = gtk.ScrolledWindow()
        scroller.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        scroller.add(textview)
        scroller.show()

        vbox = gtk.VBox()
        hbox = gtk.HBox()
        add_tcp_button = gtk.Button("Add TCP client")
        self["add_tcp_button"] = add_tcp_button
        add_tcp_button.show()
        add_udp_button = gtk.Button("Add UDP client")
        self["add_udp_button"] = add_udp_button
        add_udp_button.show()
        hbox.pack_start(add_tcp_button, True, True, 0)
        hbox.pack_start(add_udp_button, True, True, 0)
        # hbox.show()

        send_box = gtk.HBox()
        liststore = gtk.ListStore(str, str, int)
        combobox = gtk.ComboBox(liststore)
        cell = gtk.CellRendererText()
        combobox.pack_start(cell, True)
        combobox.add_attribute(cell, 'text', 0)
        self["liststore"] = liststore
        self["combobox"] = combobox
        combobox.show()
        message_label = gtk.Label("Message:")
        message_label.show()
        entry = gtk.Entry()
        self["entry"] = entry
        entry.show()
        send_button = gtk.Button("Send")
        self["send_button"] = send_button
        send_button.show()
        send_box.pack_start(combobox, False, True, 0)
        send_box.pack_start(message_label, False, True, 0)
        send_box.pack_start(entry, True, True, 0)
        send_box.pack_start(send_button, False, True, 0)
        send_box.show()

        vbox.pack_start(scroller, True, True, 0)
        vbox.pack_start(hbox, False, True, 0)
        vbox.pack_start(send_box, False, True, 0)
        vbox.show()

        window.add(vbox)

        window.connect("delete_event", self.delete_event)

        window.show()

        self.textview = textview
        self["window"] = window
        self["textview"] = textview

        self.quit_flag = False

    def delete_event(self, widget, event=None):
        reactor.stop()
        gtk.main_quit()

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