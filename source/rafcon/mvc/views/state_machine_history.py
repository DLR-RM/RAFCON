import gtk
from gtkmvc import View
from rafcon.mvc.utils import constants


class HistoryTreeView(View, gtk.TreeView):
    top = 'history_treeview'

    def __init__(self):
        View.__init__(self)
        gtk.TreeView.__init__(self)

        foreground = 5

        # cell = gtk.CellRendererText()
        # tvcolumn = gtk.TreeViewColumn('Nr', cell, text=0, foreground=foreground)
        # self.append_column(tvcolumn)

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
        # tvcolumn.set_min_width(150)
        self.append_column(tvcolumn)

        self['history_treeview'] = self


class StateMachineHistoryView(View, gtk.ScrolledWindow):
    top = 'history_view'

    def __init__(self):
        View.__init__(self)
        gtk.ScrolledWindow.__init__(self)

        history_tree = HistoryTreeView()
        history_tree.show()

        button_hbox = gtk.HBox()
        undo_button = gtk.Button("Undo")
        undo_button.set_border_width(constants.BORDER_WIDTH)
        undo_button.show()
        redo_button = gtk.Button("Redo")
        redo_button.set_border_width(constants.BORDER_WIDTH)
        redo_button.show()
        reset_button = gtk.Button("Reset")
        reset_button.set_border_width(constants.BORDER_WIDTH)
        reset_button.show()
        branch_checkbox = gtk.CheckButton("B")
        branch_checkbox.set_border_width(constants.BORDER_WIDTH)
        branch_checkbox.show()
        tree_checkbox = gtk.CheckButton("T")
        tree_checkbox.set_border_width(constants.BORDER_WIDTH)
        tree_checkbox.show()
        folded_checkbox = gtk.CheckButton("F")
        folded_checkbox.set_border_width(constants.BORDER_WIDTH)
        folded_checkbox.show()
        button_hbox.pack_start(undo_button, False, True, 0)
        button_hbox.pack_start(redo_button, False, True, 0)
        button_hbox.pack_start(reset_button, False, True, 0)
        button_hbox.pack_start(branch_checkbox, False, True, 0)
        button_hbox.pack_start(tree_checkbox, False, True, 0)
        button_hbox.pack_start(folded_checkbox, False, True, 0)
        button_hbox.show()
        history_vbox = gtk.VBox()
        history_vbox.pack_start(button_hbox, False, True, 0)
        history_vbox.pack_start(self, True, True, 0)
        history_vbox.show()

        self.add(history_tree)
        self.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        self.show()

        self['history_vbox'] = history_vbox
        self['history_view'] = self
        self['history_tree'] = history_tree
        self['undo_button'] = undo_button
        self['redo_button'] = redo_button
        self['reset_button'] = reset_button
        self['branch_checkbox'] = branch_checkbox
        self['tree_checkbox'] = tree_checkbox
        self['folded_checkbox'] = folded_checkbox
