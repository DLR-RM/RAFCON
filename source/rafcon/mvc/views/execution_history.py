import gtk
from gtkmvc import View
from rafcon.utils import constants


class ExecutionHistoryTreeView(View, gtk.TreeView):
    top = 'history_treeview'

    def __init__(self):
        View.__init__(self)
        gtk.TreeView.__init__(self)

        column_name = gtk.TreeViewColumn('History')
        self.append_column(column_name)
        cell_renderer_name = gtk.CellRendererText()
        column_name.pack_start(cell_renderer_name, True)
        column_name.add_attribute(cell_renderer_name, 'text', 0)

        self['history_treeview'] = self


class ExecutionHistoryView(View):
    top = 'history_vbox'

    def __init__(self):
        View.__init__(self)

        history_vbox = gtk.VBox()
        reload_button = gtk.Button("Reload history")
        reload_button.set_border_width(constants.BORDER_WIDTH)
        reload_button.show()
        history_tree = ExecutionHistoryTreeView()
        history_tree.show()
        scroller = gtk.ScrolledWindow()
        scroller.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        scroller.add(history_tree)
        scroller.show()
        history_vbox.pack_start(reload_button, False, True, 0)
        history_vbox.pack_start(scroller, True, True, 0)
        history_vbox.show()

        self['history_vbox'] = history_vbox
        self['reload_button'] = reload_button
        self['history_tree'] = history_tree