import gtk
from gtkmvc import View
from mvc.views import StatePropertiesView, ContainerStateView, GraphicalEditorView, StateDataportEditorView,\
     GlobalVariableEditorView, ExternalModuleManagerView,  SourceEditorView, SingleWidgetWindowView, StateEditorView, \
     LoggingView, StateMachineTreeView, LibraryTreeView


class MainWindowView(View):
    builder = './glade/main_window.glade'
    top = 'main_window'

    def __init__(self, logging_view):
        View.__init__(self)

        self.logging_view = logging_view

        # insert library-tree
        self.library_tree = LibraryTreeView()
        self.library_tree.show()

        self.state_machine_tree = StateMachineTreeView()
        self.state_machine_tree.show()

        # self.graphical_editor_window = gtk.Window()
        # self.graphical_editor_window.set_title("Graphical Editor")
        # self.graphical_editor_window.set_position(1)
        # graphical_editor_view = GraphicalEditorView()
        # self.graphical_editor_window.add(graphical_editor_view.get_top_widget())
        # self.graphical_editor_window.show_all()

        #self.graphical_editor_window = SingleWidgetWindowView(GraphicalEditorView, title="Graphical-Editor", pos=1)

        self.graphical_editor_view = GraphicalEditorView()
        self.graphical_editor_view.show()

        self.external_module_manager_view = ExternalModuleManagerView()
        self.external_module_manager_view.show()

        self.global_var_manager_view = GlobalVariableEditorView()
        self.global_var_manager_view.show()

        # # insert one graphical-statemachine-editor
        # self.graphical_editors = GraphicalEditorView()
        # self.state_machines_editors = self.graphical_editors
        # self['state_machines_vbox'].add(self.state_machines_editors.get_top_widget())
        #
        # # insert one state-editor
        # self.state_editors = StateEditorView()
        # self["state_editors_vbox"].add(self.state_editors.get_top_widget())

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

