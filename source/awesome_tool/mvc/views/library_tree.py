import gtk
from gtkmvc import View


class LibraryTreeView(View, gtk.TreeView):
    top = 'top_tree'

    def __init__(self):
        View.__init__(self)
        gtk.TreeView.__init__(self)

        tvcolumn = gtk.TreeViewColumn('Name')
        #tvcolumn.set_min_width(150)
        self.append_column(tvcolumn)
        cell = gtk.CellRendererText()
        tvcolumn.pack_start(cell, True)
        tvcolumn.add_attribute(cell, 'text', 0)

        # tvcolumn = gtk.TreeViewColumn('ID')
        # #tvcolumn.set_min_width(150)
        # self.append_column(tvcolumn)
        # cell = gtk.CellRendererText()
        # tvcolumn.pack_start(cell, True)
        # tvcolumn.add_attribute(cell, 'text', 1)
        #
        # tvcolumn = gtk.TreeViewColumn('Type')
        # #tvcolumn.set_min_width(150)
        # self.append_column(tvcolumn)
        # cell = gtk.CellRendererText()
        # tvcolumn.pack_start(cell, True)
        # tvcolumn.add_attribute(cell, 'text', 2)

        self['top_tree'] = self


if __name__ == '__main__':
    from mvc.views import SingleWidgetWindowView
    from mvc.controllers import LibraryTreeController, SingleWidgetWindowController

    import mvc.main as main

    main.setup_path()
    main.check_requirements()
    [ctr_model, logger, ctr_state] = main.main()

    v = SingleWidgetWindowView(LibraryTreeView, width=50, height=100)
    c = SingleWidgetWindowController(ctr_model, v, LibraryTreeController)

    gtk.main()
