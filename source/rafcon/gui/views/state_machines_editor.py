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

    def __init__(self):
        View.__init__(self)
        self.notebook = PlusAddNotebook()
        self.notebook.set_tab_hborder(constants.TAB_BORDER_WIDTH)
        self.notebook.set_tab_vborder(constants.TAB_BORDER_WIDTH)
        self.notebook.set_scrollable(True)
        self.notebook.set_name("state_machines_notebook")
        self.notebook.show()

        self['notebook'] = self.notebook
        self.top = 'notebook'


gobject.signal_new("add_clicked", gtk.Notebook, gobject.SIGNAL_RUN_FIRST, None, ())


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
        self.connect('button_press_event', self.on_button_press)
        self.connect('expose_event', self.on_expose_event)
        self.pixbuf = gtk.gdk.pixbuf_new_from_xpm_data(self.pixbuf_data)

        self.enable_add_button = True
        self._add_button_drawn = False

    def on_button_press(self, widget, event):
        """ Emit an add_state_machine signal if a left double click is performed right of open state machine tabs and 
        generate a popup menu to switch pages and create new or close state machines.
        """
        pb_x, pb_y = self.get_pixbuf_xy()
        pb_height = self.pixbuf.get_height()

        there_is_an_arrow = self.style_get_property("has-forward-stepper") and not self.is_there_space_for_a_button()
        arrow_icon_width = constants.ICON_SIZE_IN_PIXEL if there_is_an_arrow else 0
        right_row_end_x = self.get_allocation().x + self.get_allocation().width - arrow_icon_width
        if pb_x - constants.ICON_MARGIN <= event.x <= right_row_end_x and \
                pb_y - constants.ICON_MARGIN <= event.y <= pb_y + pb_height + constants.ICON_MARGIN and \
                event.type == gtk.gdk._2BUTTON_PRESS and event.button == 1:
            self.emit("add_clicked")
            return True

    def do_emit(self, *args):
        self.emit(*args[1:])

    def on_button_release(self, widget, event):
        """ Emit an add_state_machine signal if a left click is performed on the drawn pix-buffer add button area and 
        emit a close_state_machine signal if a middle click is performed on a state machine tab-label.
        """
        pb_x, pb_y = self.get_pixbuf_xy()

        pb_width = self.pixbuf.get_width()
        pb_height = self.pixbuf.get_height()

        if pb_x - constants.ICON_MARGIN <= event.x <= pb_x + pb_width + constants.ICON_MARGIN and \
                pb_y - constants.ICON_MARGIN <= event.y <= pb_y + pb_height + constants.ICON_MARGIN \
                and self._add_button_drawn and event.state & gtk.gdk.BUTTON1_MASK:
            self.emit("add_clicked")
            return True

    def on_expose_event(self, widget, event):
        if self.get_n_pages() > 0 and self.enable_add_button:
            self.update_add_button()

    def is_there_space_for_a_button(self):
        x, y = self.get_pixbuf_xy()
        return x < self.get_allocation().x + self.get_allocation().width - self.pixbuf.get_width()

    def update_add_button(self):
        x, y = self.get_pixbuf_xy()

        self._add_button_drawn = self.is_there_space_for_a_button()
        if self._add_button_drawn:
            self.window.draw_pixbuf(None, self.pixbuf, 0, 0, x, y, -1, -1, gtk.gdk.RGB_DITHER_NONE, 0, 0)

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
