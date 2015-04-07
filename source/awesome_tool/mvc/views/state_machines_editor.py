import gtk
from gtkmvc import View
import gobject


class StateMachinesEditorView(View):
    top = 'notebook'

    def __init__(self):
        View.__init__(self)
        self.notebook = PlusAddNotebook()
        #self.notebook = gtk.Notebook()
        self.notebook.set_scrollable(True)
        self.notebook.show()
        self['notebook'] = self.notebook


gobject.signal_new("add_state_machine", gtk.Notebook, gobject.SIGNAL_RUN_FIRST, None,
                   (gtk.Notebook, ))


class PlusAddNotebook (gtk.Notebook):

    pixbuf_data = [
        "13 13 2 1",
        "  c None",
        "x c #787b7e",
        "     xxx     ",
        "     xxx     ",
        "     xxx     ",
        "     xxx     ",
        "     xxx     ",
        "xxxxxxxxxxxxx",
        "xxxxxxxxxxxxx",
        "xxxxxxxxxxxxx",
        "     xxx     ",
        "     xxx     ",
        "     xxx     ",
        "     xxx     ",
        "     xxx     "
    ]

    def __init__(self):
        gtk.Notebook.__init__(self)

        self.connect("button_release_event", self.on_button_release)
        self.pixbuf = gtk.gdk.pixbuf_new_from_xpm_data(self.pixbuf_data)

        self.add_visible = True

    def on_button_release(self, widget, event):
        x, y = self.get_pixbuf_xy_root()

        pb_width = self.pixbuf.get_width()
        pb_height = self.pixbuf.get_height()

        if event.x_root >= x and event.x_root <= x + pb_width and event.y_root >= y and event.y_root <= y + pb_height\
                and self.add_visible:
            self.emit("add_state_machine", self)

    def do_expose_event(self, event):
        gtk.Notebook.do_expose_event(self, event)

        x, y = self.get_pixbuf_xy()

        self.add_visible = x < self.get_allocation().x + self.get_allocation().width - self.pixbuf.get_width()

        if self.add_visible:
            self.window.draw_pixbuf(None, self.pixbuf, 0, 0, x, y, -1, -1, gtk.gdk.RGB_DITHER_NONE, 0, 0)

        return True

    def get_pixbuf_xy(self):
        pb_width = self.pixbuf.get_width()
        pb_height = self.pixbuf.get_height()

        allocation = self.get_tab_label(self.get_nth_page(self.get_n_pages() - 1)).get_allocation()
        x = allocation.x + allocation.width + pb_width
        y = allocation.y + (allocation.height - pb_height) / 2

        return x, y

    def get_pixbuf_xy_root(self):
        root_x, root_y = self.window.get_root_origin()
        x, y = self.get_pixbuf_xy()

        x += root_x
        y += root_y + self.pixbuf.get_height() / 2

        return x, y