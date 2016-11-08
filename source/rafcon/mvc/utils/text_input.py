import gtk


from rafcon.utils import log

logger = log.get_logger(__name__)


class RAFCONTextInput(gtk.Dialog):

    def __init__(self, content=''):
        super(RAFCONTextInput, self).__init__()
        self.content = content
        self.entry = None
        self.setup()

    def return_text(self):
        return self.entry.get_text()

    def setup(self):

        self.entry = gtk.Entry()

        self.entry.set_max_length(60)
        self.entry.set_text(self.content)
        self.entry.set_editable(1)
        self.entry.select_region(0, len(self.entry.get_text()))

        self.add_action_widget(self.entry, 1)
        self.entry.show()

        self.show()
        return

