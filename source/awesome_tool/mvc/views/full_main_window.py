import gtk
from gtkmvc import View


class FullMainWindowView(View):
    builder = './glade/full_main_window.glade'
    top = 'MainWindow'

    def __init__(self):
        View.__init__()

        # insert one state-editor
        self["state_editor_notebook_vbox"].add(self['state_editor_notebook'])

        # insert one graphical-statemachine-editor
        self['drawing_notebook'].append(self.open_statemachine_list[0])

        # insert state-machine-tree
        self['sm_vbox'].add(self['library_tree'].get_top_widget())

        # insert library-tree
        self['library_vbox'].add(self['library_tree'].get_top_widget())

        # insert environment-manager
        #self['external_module_manager_vbox'].add(self['external_module_manager'].get_top_widget())

        # insert history-overview
        #self['history_vbox'].add(self['history'].get_top_widget())

        # insert logger-console
        #self['console_scroller'].add(self['logger_console'].get_top_widget())

        # insert status-bar
        #self['vbox1'].reparent(self['lower_statusbar'])