import gtk
import gobject
from gtkmvc import View


class StateOutcomesTreeView(View):
    top = 'tree_view'

    def __init__(self):
        View.__init__(self)
        self.tree_view = gtk.TreeView()

        # id, name, to-state, to-outcome, name-color, to-state-color,
        #                               outcome_model, state_model
        self.tree_store = gtk.TreeStore(str, str, str, str, str, str,
                                        gobject.TYPE_PYOBJECT, gobject.TYPE_PYOBJECT)
        self.tree_view.set_model(self.tree_store)

        # Variable id
        self.id_cell = gtk.CellRendererText()
        self['id_cell'] = self.id_cell
        self.id_cell.set_property("width-chars", 5)
        self.id_cell.set_property("editable", True)
        self['id_col'] = gtk.TreeViewColumn('ID', self.id_cell, text=0, background=4)
        self.tree_view.append_column(self['id_col'])

        # Variable name
        self.name_cell = gtk.CellRendererText()
        self['name_cell'] = self.name_cell
        #self.name_cell.set_property("width-chars", 30)
        self.name_cell.set_property("editable", True)
        self['name_col'] = gtk.TreeViewColumn('Name', self.name_cell, text=1, background=4)
        self.tree_view.append_column(self['name_col'])

        # Variable to-state
        self.to_state_cell = gtk.CellRendererCombo()
        self['to_state_combo'] = self.to_state_cell
        self.to_state_cell.set_property("text-column", 0)
        #self.to_state_cell.set_property("width", 30)
        self['to_state_col'] = gtk.TreeViewColumn('To-State', self.to_state_cell, text=2, background=5)
        self.tree_view.append_column(self['to_state_col'])

        # Variable to-outcome
        self.to_outcome_cell = gtk.CellRendererCombo()
        self['to_outcome_combo'] = self.to_outcome_cell
        self.to_outcome_cell.set_property("text-column", 0)
        #self.to_outcome_cell.set_property("width", 30)
        self['to_outcome_col'] = gtk.TreeViewColumn('To-Outcome', self.to_outcome_cell, text=3, background=5)
        self.tree_view.append_column(self['to_outcome_col'])
        self.tree_view.show_all()

        self['tree_view'] = self.tree_view


class StateOutcomesEditorView(View):
    top = 'main_frame'

    def __init__(self):
        View.__init__(self)

        #w = gtk.Window()

        self.vbox = gtk.VBox()
        self.treeView = StateOutcomesTreeView()
        self.tree = self.treeView['tree_view']

        self['add_button'] = gtk.Button('Add')

        self['remove_button'] = gtk.Button('Remove')

        self.Hbox = gtk.HButtonBox()
        self.Hbox.pack_start(self['add_button'], expand=False, fill=True)
        self.Hbox.pack_end(self['remove_button'], expand=False, fill=True)

        self.vbox.pack_start(self.Hbox, expand=False, fill=True)

        scrollable = gtk.ScrolledWindow()
        scrollable.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        scrollable.add(self.treeView['tree_view'])

        self.vbox.add(scrollable)
        self.vbox.show_all()
        #w.resize(width=550, height=400)
        #w.add(vbox)
        #w.show_all()

        #self['window'] = w
        self['main_frame'] = self.vbox
        self['tree'] = self.tree

    def get_top_widget(self):
        return self.vbox


if __name__ == '__main__':
    from mvc.controllers import SingleWidgetWindowController, StateOutcomesEditorController
    from mvc.views import SingleWidgetWindowView

    import mvc.main as main
    import gtk

    main.setup_path()
    main.check_requirements()
    [ctr_model, logger, ctr_state, gvm_model, emm_model] = main.create_models()

    v = SingleWidgetWindowView(StateOutcomesEditorView, width=500, height=200, title='Outcomes Editor')
    c = SingleWidgetWindowController(ctr_model, v, StateOutcomesEditorController)

    gtk.main()
