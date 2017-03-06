# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import gtk
import gobject
from gtkmvc import View
from rafcon.gui.utils import constants
from rafcon.gui.config import global_gui_config as gui_config


class StateMachinesEditorView(View):
    top = 'notebook'

    def __init__(self):
        View.__init__(self)
        self.notebook = PlusAddNotebook()
        self.notebook.set_tab_hborder(constants.BORDER_WIDTH)
        self.notebook.set_tab_vborder(constants.BORDER_WIDTH)
        self.notebook.set_scrollable(True)
        self.notebook.set_name("state_machines_notebook")
        self.notebook.show()
        self['notebook'] = self.notebook


# gobject.signal_new("add_state_machine", gtk.VBox, gobject.SIGNAL_RUN_FIRST, None, (gtk.VBox,))
# gobject.signal_new("switch-page", gtk.VBox, gobject.SIGNAL_RUN_LAST, None,
#                    (gtk.Notebook, gobject.GPointer, gobject.TYPE_UINT))


gobject.signal_new("add_state_machine", gtk.Notebook, gobject.SIGNAL_RUN_FIRST, None, ())
gobject.signal_new("close_state_machine", gtk.Notebook, gobject.SIGNAL_RUN_FIRST, None, (int,))


class PlusAddNotebook(gtk.Notebook):
    pixbuf_data = [
        "13 13 2 1",
        "  c None",
        "x c %s" % gui_config.colors['TEXT'],
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

        self.set_tab_hborder(constants.BORDER_WIDTH * 2)
        self.set_tab_vborder(constants.BORDER_WIDTH * 2)

        self.add_visible = True

    def on_button_release(self, widget, event):
        x, y = event.x, event.y
        pb_x, pb_y = self.get_pixbuf_xy()

        pb_width = self.pixbuf.get_width()
        pb_height = self.pixbuf.get_height()

        if pb_x - constants.ICON_MARGIN <= event.x <= pb_x + pb_width + constants.ICON_MARGIN and \
                pb_y - constants.ICON_MARGIN <= event.y <= pb_y + pb_height + constants.ICON_MARGIN \
                and self.add_visible and event.state & gtk.gdk.BUTTON1_MASK:
            self.emit("add_state_machine")
            return True

        for i in range(0, self.get_n_pages()):
            alloc = self.get_tab_label(self.get_nth_page(i)).get_allocation()
            widget_position = widget.get_allocation()
            mouse_x = widget_position.x + x
            mouse_y = widget_position.y + y
            if alloc.x < mouse_x < alloc.x + alloc.width and alloc.y < mouse_y < alloc.y + alloc.height and \
                            event.state & gtk.gdk.BUTTON2_MASK:
                self.emit("close_state_machine", i)
                return True

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
        x = allocation.x + allocation.width + pb_width + constants.ICON_MARGIN
        y = allocation.y + (allocation.height - pb_height) / 2

        return x, y

    def get_pixbuf_xy_root(self):
        root_x, root_y = self.window.get_root_origin()
        x, y = self.get_pixbuf_xy()

        x += root_x
        y += root_y + self.pixbuf.get_height() / 2

        return x, y
