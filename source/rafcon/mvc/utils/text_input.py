import gtk
import os
from rafcon.mvc.utils import constants
from rafcon.utils import log
from rafcon.mvc.config import global_gui_config

logger = log.get_logger(__name__)

class RAFCONTextInput(gtk.Window):

    def __init__(self):
        super(RAFCONTextInput, self).__init__()

    def enter_callback(self, widget, entry):
        entry_text = entry.get_text()

    def set_path(self, widget, entry, text="gedit"):
        logger.debug(entry.get_text() + " was set as external editor")
        global_gui_config.set_config_value('DEFAULT_EXTERNAL_EDITOR', entry.get_text())
        self.destroy()
        return

    def setup(self, title, entry_sample_text):
        window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.set_size_request(400,100)
        self.set_title(title)

        vbox = gtk.VBox(spacing=constants.GRID_SIZE, homogeneous=False)
        self.add(vbox)

        vbox.show()

        entry = gtk.Entry()

        entry.set_max_length(40)
        entry.connect("activate", self.enter_callback, entry)
        entry.set_text(entry_sample_text)
        entry.set_editable(1)
        entry.select_region(0, len(entry.get_text()))

        vbox.pack_start(entry, True, True, 0)

        entry.show()

        hbox = gtk.HBox(spacing=constants.GRID_SIZE, homogeneous=False)
        vbox.add(hbox)

        hbox.show()

        button = gtk.Button(stock=gtk.STOCK_OK)
        button.connect("clicked", self.set_path, entry)
        hbox.pack_start(button, True, True, 0)
        button.set_flags(gtk.CAN_DEFAULT)
        button.grab_default()
        button.show()

        button2 = gtk.Button(stock=gtk.STOCK_CANCEL)
        button2.connect("clicked", lambda w: self.destroy())
        hbox.pack_start(button2, True, True, 0)
        button2.set_flags(gtk.CAN_DEFAULT)
        button2.grab_default()
        button2.show()

        self.show()


