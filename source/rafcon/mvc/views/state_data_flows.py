from gtkmvc import View
from rafcon.utils import constants


class StateDataFlowsListView(View):
    builder = './glade/data_flow_list_widget.glade'
    # top = 'data_flow_list_view'
    top = 'tree_view'

    def __init__(self):
        View.__init__(self)
        # self.tree_view = self.get_top_widget()
        self.tree_view = self['tree_view']

        # self.tree_view = gtk.TreeView()
        #
        # # id, name, to-state, to-outcome, name-color, to-state-color,
        # #                               outcome_model, state_model
        # # self.tree_store = gtk.TreeStore(str, str, str, str, str, str,
        # #                                 gobject.TYPE_PYOBJECT, gobject.TYPE_PYOBJECT)
        # # self.tree_view.set_model(self.tree_store)
        #
        # # Variable from-state
        # self.from_state_cell = gtk.CellRendererCombo()
        # self['from_state_combo'] = self.from_state_cell
        # #self.from_state_cell.set_property("width", 30)
        # self.from_state_cell.set_property("text-column", 0)
        # self['from_state_col'] = gtk.TreeViewColumn('From-State', self.from_state_cell, text=1, editable=10)  #,
        # background=6)
        # self.tree_view.append_column(self['from_state_col'])
        #
        # # Variable from-key
        # self.from_key_cell = gtk.CellRendererCombo()
        # self['from_key_combo'] = self.from_key_cell
        # #self.from_key_cell.set_property("width", 30)
        # self.from_key_cell.set_property("text-column", 0)
        # self['from_key_col'] = gtk.TreeViewColumn('From-Key', self.from_key_cell, text=2, editable=10) # ,
        # background=6)
        # self.tree_view.append_column(self['from_key_col'])
        #
        # # Variable to-state
        # self.to_state_cell = gtk.CellRendererCombo()
        # self['to_state_combo'] = self.to_state_cell
        # self.to_state_cell.set_property("text-column", 0)
        # #self.to_state_cell.set_property("width", 30)
        # self['to_state_col'] = gtk.TreeViewColumn('To-State', self.to_state_cell, text=3, editable=10) # ,
        # background=7)
        # self.tree_view.append_column(self['to_state_col'])
        #
        # # Variable to-key
        # self.to_key_cell = gtk.CellRendererCombo()
        # self['to_key_combo'] = self.to_key_cell
        # self.to_key_cell.set_property("text-column", 0)
        # #self.to_outcome_cell.set_property("width", 30)
        # self['to_key_col'] = gtk.TreeViewColumn('To-Key', self.to_key_cell, text=4, editable=10)  # , background=7)
        # self.tree_view.append_column(self['to_key_col'])
        #
        # # Variable is_external
        # self.external_cell = gtk.CellRendererToggle()
        # self['external_toggle'] = self.external_cell
        # #self.to_outcome_cell.set_property("width", 30)
        # self['external_col'] = gtk.TreeViewColumn('External', self.external_cell, active=5)  # , cell_background=7)
        # self.tree_view.append_column(self['external_col'])
        #
        # self.tree_view.show_all()
        #
        # self['tree_view'] = self.tree_view

    def get_top_widget(self):
        return self.tree_view


class StateDataFlowsEditorView(View):
    builder = './glade/state_data_flows_widget.glade'
    top = 'vbox1'

    def __init__(self):
        View.__init__(self)
        self.data_flows_listView = StateDataFlowsListView()
        self['dataflows_scroller'].add(self.data_flows_listView.get_top_widget())

        self['internal_d_checkbutton'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['connected_to_d_checkbutton'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['add_d_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
        self['remove_d_button'].set_border_width(constants.BUTTON_BORDER_WIDTH)
