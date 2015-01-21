import gtk
from gtkmvc import View


class StateMachineTreeView(View, gtk.TreeView):
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

        tvcolumn = gtk.TreeViewColumn('ID')
        #tvcolumn.set_min_width(150)
        self.append_column(tvcolumn)
        cell = gtk.CellRendererText()
        tvcolumn.pack_start(cell, True)
        tvcolumn.add_attribute(cell, 'text', 1)

        tvcolumn = gtk.TreeViewColumn('Type')
        #tvcolumn.set_min_width(150)
        self.append_column(tvcolumn)
        cell = gtk.CellRendererText()
        tvcolumn.pack_start(cell, True)
        tvcolumn.add_attribute(cell, 'text', 2)

        self['top_tree'] = self


if __name__ == '__main__':
    from mvc.controllers import StateMachineTreeController, SingleWidgetWindowController
    from mvc.views import SingleWidgetWindowView

    import mvc.main as main

    main.setup_path()
    main.check_requirements()
    [ctr_model, logger, ctr_state, gvm_model, emm_model] = main.create_models()

    v = SingleWidgetWindowView(StateMachineTreeView, width=500, height=200, title='State Machine Tree')
    c = SingleWidgetWindowController(ctr_model, v, StateMachineTreeController)

    gtk.main()
