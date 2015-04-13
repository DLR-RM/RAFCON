import gtk
from gtkmvc import View
from awesome_tool.utils import constants
import gobject


gobject.signal_new("close_state_tab", gtk.Notebook, gobject.SIGNAL_RUN_FIRST, None, (int, ))


class StatesEditorView(View):
    top = 'notebook'

    def __init__(self):
        View.__init__(self)
        self.notebook = gtk.Notebook()
        self.notebook.set_scrollable(True)
        self.notebook.show()
        self.notebook.set_tab_hborder(constants.BORDER_WIDTH * 2)
        self.notebook.set_tab_vborder(constants.BORDER_WIDTH * 3)

        self.notebook.connect("button_press_event", self.button_released)

        self['notebook'] = self.notebook

    def button_released(self, widget, event=None):
        x, y = event.x, event.y
        for i in range(0, self.notebook.get_n_pages()):
            alloc = self.notebook.get_tab_label(self.notebook.get_nth_page(i)).get_allocation()
            widget_position = widget.get_allocation()
            mouse_x = widget_position.x + x
            mouse_y = widget_position.y + y
            if mouse_x > alloc.x and mouse_x < alloc.x + alloc.width and mouse_y > alloc.y\
                    and mouse_y < alloc.y + alloc.height and event.button == 2:
                self.notebook.emit("close_state_tab", i)
                return