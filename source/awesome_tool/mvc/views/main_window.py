import gtk
from gtkmvc import View
from mvc.views import StateEditorView, GraphicalEditorView, LibraryTreeView, StateMachineTreeView


class FullMainWindowView(View):
    builder = './glade/main_window.glade'
    top = 'MainWindow'

    def __init__(self):
        View.__init__(self)

        # insert library-tree
        self.library_tree = LibraryTreeView()
        self['library_vbox'].add(self.library_tree.get_top_widget())

        # insert state-machine-tree
        self.state_machine_tree = StateMachineTreeView()
        self['smtree_vbox'].add(self.state_machine_tree.get_top_widget())

        # insert one graphical-statemachine-editor
        self.graphical_editors = GraphicalEditorView()
        self.state_machines_editors = self.graphical_editors
        self['state_machines_vbox'].add(self.state_machines_editors.get_top_widget())

        # insert one state-editor
        self.state_editors = StateEditorView()
        self["state_editors_vbox"].add(self.state_editors.get_top_widget())

        # insert environment-manager
        #self['external_module_manager_vbox'].add(self['external_module_manager'].get_top_widget())

        # insert global-variable-manager
        #self['global_variable_manager_vbox'].add(self.global_variable_manager.get_top_widget())

        # insert history-overview
        #self['history_vbox'].add(self['history'].get_top_widget())

        # insert logger-console
        #self['console_scroller'].add(self['logger_console'].get_top_widget())

        # insert status-bar
        #self['vbox1'].reparent(self['lower_statusbar'])

