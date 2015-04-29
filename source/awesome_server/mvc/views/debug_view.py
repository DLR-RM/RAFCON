from gtkmvc import View
import gtk

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
        hbox.show()

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

        self["window"] = window
        self["textview"] = textview

    def delete_event(self, widget, event=None):
        reactor.stop()
        gtk.main_quit()