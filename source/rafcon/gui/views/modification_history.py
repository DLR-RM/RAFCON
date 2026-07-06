# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from gi.repository import Gtk
from rafcon.design_patterns.mvc.view import View
from rafcon.gui.utils import constants
from rafcon.gui.utils.gtk_utils import set_all_margins
from rafcon.gui.helpers import label


class HistoryTreeView(View, Gtk.TreeView):

    def __init__(self):
        View.__init__(self, parent='history_treeview')
        Gtk.TreeView.__init__(self)

        foreground = 5

        tvcolumn = Gtk.TreeViewColumn('Nr', Gtk.CellRendererText(), text=1, foreground=foreground)
        tvcolumn.set_property("sizing", Gtk.TreeViewColumnSizing.AUTOSIZE)
        self.append_column(tvcolumn)

        tvcolumn = Gtk.TreeViewColumn('Action', Gtk.CellRendererText(), text=2, foreground=foreground)
        tvcolumn.set_property("sizing", Gtk.TreeViewColumnSizing.AUTOSIZE)
        self.append_column(tvcolumn)

        tvcolumn = Gtk.TreeViewColumn('Parameters', Gtk.CellRendererText(), text=7, foreground=foreground)
        tvcolumn.set_property("sizing", Gtk.TreeViewColumnSizing.AUTOSIZE)
        self.append_column(tvcolumn)

        tvcolumn = Gtk.TreeViewColumn('Affects', Gtk.CellRendererText(), text=3, foreground=foreground)
        tvcolumn.set_property("sizing", Gtk.TreeViewColumnSizing.AUTOSIZE)
        # tvcolumn.set_min_width(150)
        self.append_column(tvcolumn)

        self['history_treeview'] = self


class ModificationHistoryView(View, Gtk.ScrolledWindow):
    def __init__(self):
        View.__init__(self, parent='history_vbox')
        Gtk.ScrolledWindow.__init__(self)

        history_tree = HistoryTreeView()
        history_tree.set_name('history_tree')

        undo_button = Gtk.Button.new_with_label("Undo")
        set_all_margins(undo_button, constants.BUTTON_BORDER_WIDTH)
        redo_button = Gtk.Button.new_with_label("Redo")
        set_all_margins(redo_button, constants.BUTTON_BORDER_WIDTH)
        reset_button = Gtk.Button.new_with_label("Reset")
        set_all_margins(reset_button, constants.BUTTON_BORDER_WIDTH)
        branch_checkbox = Gtk.CheckButton.new_with_label("Branches")
        branch_checkbox.set_tooltip_text('Show branches')
        set_all_margins(branch_checkbox, constants.BUTTON_BORDER_WIDTH)
        branch_checkbox.add_css_class("secondary")
        folded_checkbox = Gtk.CheckButton.new_with_label("Fold")
        folded_checkbox.set_tooltip_text('Fold branches')
        set_all_margins(folded_checkbox, constants.BUTTON_BORDER_WIDTH)
        folded_checkbox.add_css_class("secondary")

        button_hbox = Gtk.Box.new(Gtk.Orientation.HORIZONTAL, 0)
        button_hbox.add_css_class("widget-toolbar")
        button_hbox.set_halign(Gtk.Align.END)
        button_hbox.append(undo_button)
        button_hbox.append(redo_button)
        button_hbox.append(reset_button)
        button_hbox.append(branch_checkbox)
        button_hbox.append(folded_checkbox)

        label.ellipsize_labels_recursively(button_hbox)

        history_vbox = Gtk.Box.new(Gtk.Orientation.VERTICAL, 0)
        self.set_vexpand(True)
        history_vbox.append(self)
        history_vbox.append(button_hbox)

        self.set_child(history_tree)
        self.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)

        self['history_vbox'] = history_vbox
        self['history_view'] = self
        self['history_tree'] = history_tree
        self['undo_button'] = undo_button
        self['redo_button'] = redo_button
        self['reset_button'] = reset_button
        self['branch_checkbox'] = branch_checkbox
        self['folded_checkbox'] = folded_checkbox
