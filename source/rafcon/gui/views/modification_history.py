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

import gtk
from gtkmvc import View
from rafcon.gui.utils import constants


class HistoryTreeView(View, gtk.TreeView):

    def __init__(self):
        View.__init__(self)
        gtk.TreeView.__init__(self)

        foreground = 5

        tvcolumn = gtk.TreeViewColumn('Nr', gtk.CellRendererText(), text=1, foreground=foreground)
        tvcolumn.set_property("sizing", "autosize")
        self.append_column(tvcolumn)

        tvcolumn = gtk.TreeViewColumn('Action', gtk.CellRendererText(), text=2, foreground=foreground)
        tvcolumn.set_property("sizing", "autosize")
        self.append_column(tvcolumn)

        tvcolumn = gtk.TreeViewColumn('Parameters', gtk.CellRendererText(), text=7, foreground=foreground)
        tvcolumn.set_property("sizing", "autosize")
        self.append_column(tvcolumn)

        tvcolumn = gtk.TreeViewColumn('Affects', gtk.CellRendererText(), text=3, foreground=foreground)
        tvcolumn.set_property("sizing", "autosize")
        # tvcolumn.set_min_width(150)
        self.append_column(tvcolumn)

        self['history_treeview'] = self
        self.top = 'history_treeview'


class ModificationHistoryView(View, gtk.ScrolledWindow):

    def __init__(self):
        View.__init__(self)
        gtk.ScrolledWindow.__init__(self)

        history_tree = HistoryTreeView()
        history_tree.set_name('history_tree')

        undo_button = gtk.Button("Undo")
        undo_button.set_border_width(constants.BUTTON_BORDER_WIDTH)
        redo_button = gtk.Button("Redo")
        redo_button.set_border_width(constants.BUTTON_BORDER_WIDTH)
        reset_button = gtk.Button("Reset")
        reset_button.set_border_width(constants.BUTTON_BORDER_WIDTH)
        branch_checkbox = gtk.CheckButton("B")
        branch_checkbox.set_tooltip_text('Show branches')
        branch_checkbox.set_border_width(constants.BUTTON_BORDER_WIDTH)
        folded_checkbox = gtk.CheckButton("F")
        folded_checkbox.set_tooltip_text('Fold branches')
        folded_checkbox.set_border_width(constants.BUTTON_BORDER_WIDTH)

        button_hbox = gtk.HBox()
        button_hbox.pack_end(folded_checkbox, False, True, 0)
        button_hbox.pack_end(branch_checkbox, False, True, 0)
        button_hbox.pack_end(reset_button, False, True, 0)
        button_hbox.pack_end(redo_button, False, True, 0)
        button_hbox.pack_end(undo_button, False, True, 0)

        history_vbox = gtk.VBox()
        history_vbox.pack_start(self, True, True, 0)
        history_vbox.pack_start(button_hbox, False, True, 0)

        self.add(history_tree)
        self.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        self.show_all()

        self['history_vbox'] = history_vbox
        self['history_view'] = self
        self['history_tree'] = history_tree
        self['undo_button'] = undo_button
        self['redo_button'] = redo_button
        self['reset_button'] = reset_button
        self['branch_checkbox'] = branch_checkbox
        self['folded_checkbox'] = folded_checkbox
        self.top = 'history_view'
