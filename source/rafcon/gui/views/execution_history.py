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
from rafcon.design_patterns.mvc.view import View
from rafcon.gui.utils import constants
from rafcon.gui.utils.gtk_utils import set_all_margins
from rafcon.gui.helpers import label


class ExecutionHistoryTreeView(View, Gtk.TreeView):
    def __init__(self):
        View.__init__(self, parent='history_treeview')
        Gtk.TreeView.__init__(self)
        self.set_name("history_tree")

        tvcolumn = Gtk.TreeViewColumn('History', Gtk.CellRendererText(), text=0)
        tvcolumn.set_property("sizing", Gtk.TreeViewColumnSizing.AUTOSIZE)
        self.append_column(tvcolumn)

        self['history_treeview'] = self


class ExecutionHistoryView(View, Gtk.ScrolledWindow):
    def __init__(self):
        View.__init__(self, parent='history_vbox')
        Gtk.ScrolledWindow.__init__(self)

        history_tree = ExecutionHistoryTreeView()

        reload_button = Gtk.Button.new_with_label("Reload")
        set_all_margins(reload_button, constants.BUTTON_BORDER_WIDTH)
        clean_button = Gtk.Button.new_with_label("Clean")
        set_all_margins(clean_button, constants.BUTTON_BORDER_WIDTH)
        open_separately_button = Gtk.Button.new_with_label("Open externally")
        set_all_margins(open_separately_button, constants.BUTTON_BORDER_WIDTH)
        lock_checkbox = Gtk.CheckButton.new_with_label("Lock")
        lock_checkbox.set_tooltip_text('Locks the execution history')
        set_all_margins(lock_checkbox, constants.BUTTON_BORDER_WIDTH)
        lock_checkbox.add_css_class("secondary")

        button_box = Gtk.Box.new(Gtk.Orientation.HORIZONTAL, 0)
        button_box.add_css_class("widget-toolbar")
        button_box.set_halign(Gtk.Align.END)
        button_box.append(lock_checkbox)
        button_box.append(open_separately_button)
        button_box.append(clean_button)
        button_box.append(reload_button)

        label.ellipsize_labels_recursively(button_box)

        history_vbox = Gtk.Box.new(Gtk.Orientation.VERTICAL, 0)
        self.set_vexpand(True)
        history_vbox.append(self)
        history_vbox.append(button_box)

        self.set_child(history_tree)
        self.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)

        self['history_vbox'] = history_vbox
        self['history_view'] = self
        self['history_tree'] = history_tree
        self['reload_button'] = reload_button
        self['clean_button'] = clean_button
        self['open_separately_button'] = open_separately_button
        self['lock_checkbox'] = lock_checkbox
