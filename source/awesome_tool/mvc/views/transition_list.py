import gtk, gobject
from gtkmvc import View

class TransitionListView(View):
    #builder = './glade/TransitionListWidget.glade'
    #top = 'transition_list_view'
    top = 'tree_view'

    def __init__(self):
        View.__init__(self)
        #self.tree_view = self.get_top_widget()

        self.tree_view = gtk.TreeView()

        # # id, name, to-state, to-outcome, name-color, to-state-color,
        # #                               outcome_model, state_model
        # self.tree_store = gtk.TreeStore(str, str, str, str, str, str,
        #                                 gobject.TYPE_PYOBJECT, gobject.TYPE_PYOBJECT)
        # self.tree_view.set_model(self.tree_store)

        # Variable from-state
        self.from_state_cell = gtk.CellRendererCombo()
        self['from_state_combo'] = self.from_state_cell
        #self.to_state_cell.set_property("width", 30)
        self.from_state_cell.set_property("text-column", 0)
        self['from_state_col'] = gtk.TreeViewColumn('From-State', self.from_state_cell, text=1, background=6, editable=10)
        self.tree_view.append_column(self['from_state_col'])

        # Variable from-outcome
        self.from_outcome_cell = gtk.CellRendererCombo()
        self['from_outcome_combo'] = self.from_outcome_cell
        #self.to_state_cell.set_property("width", 30)
        self.from_outcome_cell.set_property("text-column", 0)
        self['from_outcome_col'] = gtk.TreeViewColumn('From-Outcome', self.from_outcome_cell, text=2, background=6, editable=10)
        self.tree_view.append_column(self['from_outcome_col'])

        # Variable to-state
        self.to_state_cell = gtk.CellRendererCombo()
        self['to_state_combo'] = self.to_state_cell
        self.to_state_cell.set_property("text-column", 0)
        #self.to_state_cell.set_property("width", 30)
        self['to_state_col'] = gtk.TreeViewColumn('To-State', self.to_state_cell, text=3, background=7, editable=10)
        self.tree_view.append_column(self['to_state_col'])

        # Variable to-outcome
        self.to_outcome_cell = gtk.CellRendererCombo()
        self['to_outcome_combo'] = self.to_outcome_cell
        self.to_outcome_cell.set_property("text-column", 0)
        #self.to_outcome_cell.set_property("width", 30)
        self['to_outcome_col'] = gtk.TreeViewColumn('To-Outcome', self.to_outcome_cell, text=4, background=7, editable=10)
        self.tree_view.append_column(self['to_outcome_col'])

        # Variable is_external
        self.external_cell = gtk.CellRendererToggle()
        self['external_toggle'] = self.external_cell
        #self.to_outcome_cell.set_property("width", 30)
        self['external_col'] = gtk.TreeViewColumn('External', self.external_cell, active=5, cell_background=7)
        self.tree_view.append_column(self['external_col'])

        self.tree_view.show_all()

        self['tree_view'] = self.tree_view

    def get_top_widget(self):
        return self.tree_view