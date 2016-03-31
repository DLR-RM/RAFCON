import gtk
from gtkmvc import View
from rafcon.mvc.utils import constants


class ExecutionHistoryTreeView(View, gtk.TreeView):
    top = 'history_treeview'

    def __init__(self):
        View.__init__(self)
        gtk.TreeView.__init__(self)

        tvcolumn = gtk.TreeViewColumn('History', gtk.CellRendererText(), text=0)
        self.append_column(tvcolumn)

        self['history_treeview'] = self


class ExecutionHistoryView(View, gtk.ScrolledWindow):
    top = 'history_view'

    def __init__(self):
        View.__init__(self)
        gtk.ScrolledWindow.__init__(self)

        history_tree = ExecutionHistoryTreeView()

        reload_button = gtk.Button("Reload history")
        reload_button.set_border_width(constants.BORDER_WIDTH)

        reload_button_box = gtk.HBox()
        reload_button_box.pack_end(reload_button, False, True, 0)

        history_vbox = gtk.VBox()
        history_vbox.pack_end(reload_button_box, False, True, 0)
        history_vbox.pack_end(self, True, True, 0)


        self.add(history_tree)
        self.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        self.show_all()

        self['history_vbox'] = history_vbox
        self['history_view'] = self
        self['history_tree'] = history_tree
        self['reload_button'] = reload_button

