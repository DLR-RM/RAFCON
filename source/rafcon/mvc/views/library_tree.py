import gtk
from gtkmvc import View


class LibraryTreeView(View, gtk.TreeView):
    top = 'library_tree_view'

    def __init__(self):
        View.__init__(self)
        gtk.TreeView.__init__(self)
        self.set_name('library_tree')
        # self.set_grid_lines(gtk.TREE_VIEW_GRID_LINES_HORIZONTAL)

        tvcolumn_name = gtk.TreeViewColumn('Library Name')
        self.append_column(tvcolumn_name)
        cell_renderer_name = gtk.CellRendererText()
        tvcolumn_name.pack_start(cell_renderer_name, True)
        tvcolumn_name.add_attribute(cell_renderer_name, 'text', 0)


        self['library_tree_view'] = self
