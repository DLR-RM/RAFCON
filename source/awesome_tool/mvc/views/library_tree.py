import gtk
from gtkmvc import View


class LibraryTreeView(View, gtk.TreeView):
    top = 'library_tree_view'

    def __init__(self):
        View.__init__(self)
        gtk.TreeView.__init__(self)

        tvcolumn_name = gtk.TreeViewColumn('Name')
        self.append_column(tvcolumn_name)
        cell_renderer_name = gtk.CellRendererText()
        tvcolumn_name.pack_start(cell_renderer_name, True)
        tvcolumn_name.add_attribute(cell_renderer_name, 'text', 0)

        # tvcolumn_path = gtk.TreeViewColumn('Path')
        # self.append_column(tvcolumn_path)
        # cell_renderer_path = gtk.CellRendererText()
        # tvcolumn_path.pack_start(cell_renderer_path, True)
        # tvcolumn_path.add_attribute(cell_renderer_path, 'text', 1)

        self['library_tree_view'] = self


if __name__ == '__main__':
    from mvc.views import SingleWidgetWindowView
    from mvc.controllers import LibraryTreeController, SingleWidgetWindowController

    import mvc.main as main

    main.setup_path()
    main.check_requirements()
    [ctr_model, logger, ctr_state, gvm_model, emm_model] = main.create_models()

    v = SingleWidgetWindowView(LibraryTreeView, width=50, height=100)
    c = SingleWidgetWindowController(ctr_model, v, LibraryTreeController)

    gtk.main()
