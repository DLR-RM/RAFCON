# Copyright (C) 2015-2018 DLR
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

from gi.repository import Gtk
from gi.repository import GObject
from gtkmvc3.view import View
from rafcon.gui.utils import constants
from rafcon.gui.helpers import label


class ExecutionHistoryTreeView(View, Gtk.TreeView):

    def __init__(self):
        View.__init__(self)
        Gtk.TreeView.__init__(self)
        self.set_name("history_tree")

        tvcolumn = Gtk.TreeViewColumn('History', Gtk.CellRendererText(), text=0)
        tvcolumn.set_property("sizing", Gtk.TreeViewColumnSizing.AUTOSIZE)
        self.append_column(tvcolumn)

        self['history_treeview'] = self
        self.top = 'history_treeview'


class ExecutionHistoryView(View, Gtk.ScrolledWindow):

    def __init__(self):
        View.__init__(self)
        Gtk.ScrolledWindow.__init__(self)

        history_tree = ExecutionHistoryTreeView()

        reload_button = Gtk.Button.new_with_label("Reload")
        reload_button.set_border_width(constants.BUTTON_BORDER_WIDTH)
        clean_button = Gtk.Button.new_with_label("Clean")
        clean_button.set_border_width(constants.BUTTON_BORDER_WIDTH)
        open_separately_button = Gtk.Button.new_with_label("Open externally")
        open_separately_button.set_border_width(constants.BUTTON_BORDER_WIDTH)

        button_box = Gtk.Box.new(Gtk.Orientation.HORIZONTAL, 0)
        button_box.get_style_context().add_class("widget-toolbar")
        button_box.pack_end(reload_button, False, True, 0)
        button_box.pack_end(clean_button, False, True, 0)
        button_box.pack_end(open_separately_button, False, True, 0)

        label.ellipsize_labels_recursively(button_box)

        history_vbox = Gtk.Box.new(Gtk.Orientation.VERTICAL, 0)
        history_vbox.pack_end(button_box, False, True, 0)
        history_vbox.pack_end(self, True, True, 0)

        self.add(history_tree)
        self.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
        self.show_all()

        self['history_vbox'] = history_vbox
        self['history_view'] = self
        self['history_tree'] = history_tree
        self['reload_button'] = reload_button
        self['clean_button'] = clean_button
        self['open_separately_button'] = open_separately_button
        self.top = 'history_vbox'
