import gtk
from gtkmvc import View
import gobject
from awesome_tool.utils import constants


class StateMachinesEditorView(View):
    top = 'notebook'

    def __init__(self):
        View.__init__(self)
        self.notebook = PlusAddNotebook()
        # self.notebook = gtk.Notebook()
        self.notebook.set_scrollable(True)
        self.notebook.show()
        self['notebook'] = self.notebook


gobject.signal_new("add_state_machine", gtk.VBox, gobject.SIGNAL_RUN_FIRST, None,
                   (gtk.VBox, ))
gobject.signal_new("add_state_machine", gtk.Notebook, gobject.SIGNAL_RUN_FIRST, None,
                   (gtk.Notebook, ))
gobject.signal_new("switch-page", gtk.VBox, gobject.SIGNAL_RUN_LAST, None, (gtk.Notebook, gobject.GPointer, gobject.TYPE_UINT))


class PlusAddNotebook2 (gtk.VBox):

    def __init__(self):
        gtk.VBox.__init__(self)

        hbox = gtk.HBox()

        self.nb1 = gtk.Notebook()
        self.nb1.set_show_border(False)
        self.nb1.set_border_width(5)
        self.nb1.set_scrollable(True)

        self.nb1.set_can_focus(False)
        self.nb1.set_tab_border(6)

        add = gtk.Button()
        add.set_border_width(5)
        add_label = gtk.Label()
        add_label.set_markup('<span font_desc="%s %s"> &#x%s; </span>' % (constants.DEFAULT_FONT,
                                                                          constants.FONT_SIZE_NORMAL,
                                                                          constants.BUTTON_ADD))
        add_label.show()
        add.set_image(add_label)
        add.set_focus_on_click(False)

        self.nb2 = gtk.Notebook()
        self.nb2.set_show_border(False)
        self.nb2.set_border_width(5)

        add.connect("clicked", self.add_page)

        self.nb1.connect("switch-page", self.sel_page)
        self.nb1.connect("page-reordered", self.reorder_tabs)
        self.nb1.set_tab_pos(gtk.POS_TOP)

        self.nb2.set_show_tabs(False)
        self.nb2.set_tab_pos(gtk.POS_TOP)

        self.pack_start(hbox, False, True, 0)
        self.pack_start(self.nb2, True, True, 0)

        hbox.pack_start(self.nb1, True, True, 0)
        hbox.pack_end(add, False, True, 0)

        self.add_page(None)

        add.show()
        self.nb1.show()
        self.nb2.show()
        hbox.show()

    def add_page(self, widget, data=None):
        self.emit("add_state_machine", self)

    def append_page(self, widget, label):
        alignment = gtk.Alignment()
        alignment.show()
        self.nb1.append_page(alignment, label)
        self.nb2.append_page(widget, gtk.Label())

        return self.nb1.get_n_pages() - 1

    def remove_page(self, page_num):
        self.nb1.remove_page(page_num)
        self.nb2.remove_page(page_num)

    def sel_page(self, widget, page, page_num):
        self.emit("switch-page", widget, page, page_num)
        self.nb2.set_current_page(page_num)

    def get_nth_page(self, page_num):
        return self.nb2.get_nth_page(page_num)

    def set_tab_reorderable(self, page, reorderable):
        nb1_child = self.nb1.get_nth_page(self.page_num(page))
        self.nb1.set_tab_reorderable(nb1_child, reorderable)
        pass

    def reorder_tabs(self, widget, dir, arg2, data=None):
        child = self.nb2.get_nth_page(self.nb2.get_current_page())
        self.nb2.reorder_child(child, widget.get_current_page())

    def page_num(self, widget):
        return self.nb2.page_num(widget)

    def get_current_page(self):
        return self.nb1.get_current_page()

    def set_current_page(self, page_num):
        self.nb1.set_current_page(page_num)
        self.nb2.set_current_page(page_num)

    def set_tab_label(self, child, tab_label):
        nb1_child = self.nb1.get_nth_page(self.page_num(child))
        self.nb1.set_tab_label(nb1_child, tab_label)


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

        # check if number of pages is greater zero, else the drawing will raise errors
        if self.get_n_pages() > 0:
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