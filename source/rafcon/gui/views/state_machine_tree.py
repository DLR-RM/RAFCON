import gtk
from gtkmvc import View


class StateMachineTreeView(View, gtk.TreeView):
    top = 'state_machine_tree_view'

    def __init__(self):
        View.__init__(self)
        gtk.TreeView.__init__(self)
        self.set_name('state_machine_tree')

        tvcolumn = gtk.TreeViewColumn('Name')
        # tvcolumn.set_min_width(150)
        self.append_column(tvcolumn)
        cell = gtk.CellRendererText()
        tvcolumn.pack_start(cell, True)
        tvcolumn.add_attribute(cell, 'text', 0)

        tvcolumn = gtk.TreeViewColumn('ID')
        # tvcolumn.set_min_width(150)
        self.append_column(tvcolumn)
        cell = gtk.CellRendererText()
        tvcolumn.pack_start(cell, True)
        tvcolumn.add_attribute(cell, 'text', 1)

        tvcolumn = gtk.TreeViewColumn('Type')
        # tvcolumn.set_min_width(150)
        self.append_column(tvcolumn)
        cell = gtk.CellRendererText()
        tvcolumn.pack_start(cell, True)
        tvcolumn.add_attribute(cell, 'text', 2)

        self['state_machine_tree_view'] = self
