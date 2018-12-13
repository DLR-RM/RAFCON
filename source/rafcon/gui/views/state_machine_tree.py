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
from gtkmvc3.view import View


class StateMachineTreeView(View, Gtk.TreeView):

    def __init__(self):
        View.__init__(self)
        Gtk.TreeView.__init__(self)
        self.set_name('state_machine_tree')

        tvcolumn = Gtk.TreeViewColumn('Name')
        tvcolumn.set_property("sizing", Gtk.TreeViewColumnSizing.AUTOSIZE)
        # tvcolumn.set_min_width(150)
        self.append_column(tvcolumn)
        cell = Gtk.CellRendererText()
        tvcolumn.pack_start(cell, True)
        tvcolumn.add_attribute(cell, 'text', 0)
        tvcolumn.set_sort_column_id(0)

        tvcolumn = Gtk.TreeViewColumn('ID')
        tvcolumn.set_property("sizing", Gtk.TreeViewColumnSizing.AUTOSIZE)
        # tvcolumn.set_min_width(150)
        self.append_column(tvcolumn)
        cell = Gtk.CellRendererText()
        tvcolumn.pack_start(cell, True)
        tvcolumn.add_attribute(cell, 'text', 1)
        tvcolumn.set_sort_column_id(1)

        tvcolumn = Gtk.TreeViewColumn('Type')
        tvcolumn.set_property("sizing", Gtk.TreeViewColumnSizing.AUTOSIZE)
        # tvcolumn.set_min_width(150)
        self.append_column(tvcolumn)
        cell = Gtk.CellRendererText()
        tvcolumn.pack_start(cell, True)
        tvcolumn.add_attribute(cell, 'text', 2)
        tvcolumn.set_sort_column_id(2)

        self['state_machine_tree_view'] = self
        self.top = 'state_machine_tree_view'
