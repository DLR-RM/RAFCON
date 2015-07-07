import gtk
from gtkmvc import View


class HistoryTreeView(View, gtk.TreeView):
    top = 'history_treeview'

    def __init__(self):
        View.__init__(self)
        gtk.TreeView.__init__(self)

        foreground = 5

        cell = gtk.CellRendererText()
        tvcolumn = gtk.TreeViewColumn('Nr', cell, text=0, foreground=foreground)
        self.append_column(tvcolumn)

        cell = gtk.CellRendererText()
        tvcolumn = gtk.TreeViewColumn('VNr', cell, text=1, foreground=foreground)
        self.append_column(tvcolumn)

        cell = gtk.CellRendererText()
        tvcolumn = gtk.TreeViewColumn('Method', cell, text=2, foreground=foreground)
        self.append_column(tvcolumn)

        cell = gtk.CellRendererText()
        tvcolumn = gtk.TreeViewColumn('Instance', cell, text=3, foreground=foreground)
        self.append_column(tvcolumn)

        cell = gtk.CellRendererText()
        tvcolumn = gtk.TreeViewColumn('Details', cell, text=4, foreground=foreground)
        #tvcolumn.set_min_width(150)
        self.append_column(tvcolumn)

        self['history_treeview'] = self


class StateMachineHistoryView(View, gtk.ScrolledWindow):
    top = 'state_machine_history_view'

    def __init__(self):
        View.__init__(self)
        gtk.ScrolledWindow.__init__(self)

        tree_view = HistoryTreeView()

        self.add(tree_view)
        self.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        self['state_machine_history_view'] = self
        self['state_machine_history_tree'] = tree_view
