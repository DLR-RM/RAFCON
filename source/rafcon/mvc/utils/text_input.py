import gtk

from rafcon.mvc.utils import constants
from rafcon.utils import log
from rafcon.mvc.config import global_gui_config

logger = log.get_logger(__name__)


class RAFCONTextInput(gtk.Window):

    def __init__(self, content='', window_title=''):
        super(RAFCONTextInput, self).__init__()
        self.content = content
        self.window_title = window_title

    def return_text(self):
        return self.content

    def update(self, widget, entry, text="gedit"):
        logger.info("'{}' was set as external editor".format(entry.get_text()))
        self.content = entry.get_text
        global_gui_config.set_config_value('DEFAULT_EXTERNAL_EDITOR', entry.get_text())
        self.destroy()
        return

    def setup(self):

        self.set_size_request(400, 100)
        self.set_title(self.window_title)

        vbox = gtk.VBox(spacing=constants.GRID_SIZE, homogeneous=False)
        self.add(vbox)

        entry = gtk.Entry()

        entry.set_max_length(40)
        entry.connect("activate", self.update, entry)
        entry.set_text(self.content)
        entry.set_editable(1)
        entry.select_region(0, len(entry.get_text()))

        vbox.pack_start(entry, True, True, 0)

        entry.show()

        hbox = gtk.HBox(spacing=constants.GRID_SIZE, homogeneous=False)
        vbox.add(hbox)

        button = gtk.Button(stock=gtk.STOCK_OK)
        button.connect("clicked", self.update, entry)
        hbox.pack_start(button, True, True, 0)
        button.show()

        button_cancel = gtk.Button(stock=gtk.STOCK_CANCEL)
        button_cancel.connect("clicked", lambda w: self.destroy())
        hbox.pack_start(button_cancel, True, True, 0)
        button_cancel.show()

        vbox.show()
        hbox.show()
        hbox.show()
        self.show()
        #return entry.get_text()

