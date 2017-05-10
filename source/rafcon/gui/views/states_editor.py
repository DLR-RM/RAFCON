# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import gtk
import gobject
from gtkmvc import View
from rafcon.gui.utils import constants

gobject.signal_new("tab_close_event", gtk.Notebook, gobject.SIGNAL_RUN_FIRST, None, (int,))


class StatesEditorView(View):

    def __init__(self):
        View.__init__(self)
        self.notebook = gtk.Notebook()
        self.notebook.set_scrollable(True)
        self.notebook.set_name('states_editor_notebook')
        self.notebook.set_tab_hborder(constants.TAB_BORDER_WIDTH)
        self.notebook.set_tab_vborder(constants.TAB_BORDER_WIDTH)
        self.notebook.show()

        self.notebook.connect("button_press_event", self.button_released)

        self['notebook'] = self.notebook
        self.top = 'notebook'

    def button_released(self, widget, event=None):
        x, y = event.x, event.y
        for i in range(0, self.notebook.get_n_pages()):
            alloc = self.notebook.get_tab_label(self.notebook.get_nth_page(i)).get_allocation()
            widget_position = widget.get_allocation()
            mouse_x = widget_position.x + x
            mouse_y = widget_position.y + y
            if alloc.x < mouse_x < alloc.x + alloc.width and alloc.y < mouse_y < alloc.y + alloc.height and \
                    event.button == 2:
                self.notebook.emit("tab_close_event", i)
                return
