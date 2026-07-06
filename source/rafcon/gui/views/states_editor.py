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

from gi.repository import Gtk
from gi.repository import GObject
from rafcon.design_patterns.mvc.view import View

GObject.signal_new("tab_close_event", Gtk.Notebook, GObject.SignalFlags.RUN_FIRST, None, (int,))


class StatesEditorView(View):

    def __init__(self):
        super().__init__(parent='notebook')
        self.notebook = Gtk.Notebook()
        self.notebook.set_scrollable(True)
        self.notebook.set_name('states_editor_notebook')
        self.notebook.add_css_class("secondary")
        self.notebook.show()
        # GTK4: widget event signals are gone; a middle-click gesture closes the tab under the pointer
        middle_click_gesture = Gtk.GestureClick()
        middle_click_gesture.set_button(2)
        middle_click_gesture.connect("pressed", self.on_middle_click)
        self.notebook.add_controller(middle_click_gesture)
        self['notebook'] = self.notebook

    def on_middle_click(self, gesture, n_press, x, y):
        for i in range(0, self.notebook.get_n_pages()):
            tab_label = self.notebook.get_tab_label(self.notebook.get_nth_page(i))
            success, bounds = tab_label.compute_bounds(self.notebook)
            if success and bounds.origin.x < x < bounds.origin.x + bounds.size.width and \
                    bounds.origin.y < y < bounds.origin.y + bounds.size.height:
                self.notebook.emit("tab_close_event", i)
                return
