# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gi.repository import Gtk
from rafcon.design_patterns.mvc.view import View

from rafcon.gui import glade
from rafcon.gui.views.utils.tree import TreeView
import rafcon.gui.helpers.label as gui_helper_label
from rafcon.gui.utils import constants


class StateOutcomesTreeView(TreeView):
    def __init__(self):
        super().__init__(builder_filename=glade.get_glade_path('outcome_list_widget.glade'), parent='tree_view')


class StateOutcomesEditorView(View):
    def __init__(self):
        super().__init__(parent='main_frame')

        self.vbox = Gtk.Box.new(Gtk.Orientation.VERTICAL, 0)
        self.treeView = StateOutcomesTreeView()

        add_button = Gtk.Button(label='Add')
        Gtk.Widget.set_focus_on_click(add_button, True)
        add_button.set_border_width(constants.BUTTON_BORDER_WIDTH)
        add_button.set_size_request(constants.BUTTON_MIN_WIDTH, -1)

        remove_button = Gtk.Button(label='Remove')
        Gtk.Widget.set_focus_on_click(remove_button, True)
        remove_button.set_border_width(constants.BUTTON_BORDER_WIDTH)
        remove_button.set_size_request(constants.BUTTON_MIN_WIDTH, -1)

        self['add_button'] = add_button
        self['remove_button'] = remove_button

        self.Hbox = Gtk.Box.new(Gtk.Orientation.HORIZONTAL, 0)
        self.Hbox.get_style_context().add_class("widget-toolbar")
        self.Hbox.pack_end(self['remove_button'], False, True, 0)
        self.Hbox.pack_end(self['add_button'], False, True, 0)

        scrollable = Gtk.ScrolledWindow()
        scrollable.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
        scrollable.add(self.treeView.get_parent_widget())
        self.treeView.scrollbar_widget = scrollable

        outcomes_widget_title = gui_helper_label.create_widget_title("OUTCOMES")

        self.vbox.pack_start(outcomes_widget_title, False, True, 0)
        self.vbox.pack_start(scrollable, True, True, 0)
        self.vbox.pack_start(self.Hbox, expand=False, fill=True, padding=0)
        self.vbox.show_all()

        self['main_frame'] = self.vbox
