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
from gi.repository import GObject

from gtkmvc3.view import View


class LibraryTreeView(View, Gtk.TreeView):

    def __init__(self):
        View.__init__(self)
        Gtk.TreeView.__init__(self)
        self.set_name('library_tree')
        # self.set_grid_lines(Gtk.TREE_VIEW_GRID_LINES_HORIZONTAL)

        tvcolumn_name = Gtk.TreeViewColumn('Library Name')
        tvcolumn_name.set_property("sizing", Gtk.TreeViewColumnSizing.AUTOSIZE)
        self.append_column(tvcolumn_name)
        cell_renderer_name = Gtk.CellRendererText()
        tvcolumn_name.pack_start(cell_renderer_name, True)
        tvcolumn_name.add_attribute(cell_renderer_name, 'text', 0)

        self['library_tree_view'] = self
        self.top = 'library_tree_view'
