
import gtk
from gtkmvc import View
from awesome_tool.mvc.views import StatePropertiesView, ContainerStateView, GraphicalEditorView, StateDataportEditorView,\
     GlobalVariableEditorView, SourceEditorView, SingleWidgetWindowView, StateEditorView, \
     LoggingView, StateMachineTreeView, LibraryTreeView, MenuBarView, ToolBarView, TopToolBarView
from awesome_tool.mvc.views.states_editor import StatesEditorView
from awesome_tool.mvc.views.state_machines_editor import StateMachinesEditorView
from awesome_tool.utils import constants


class MainWindowView(View):
    builder = './glade/main_window.glade'
    top = 'main_window'

    def __init__(self, logging_view):
        View.__init__(self)

        # self['main_window'].set_decorated(False)

        self.logging_view = logging_view

        # insert library-tree
        self.library_tree = LibraryTreeView()
        self.library_tree.show()

        self.state_machine_tree = StateMachineTreeView()
        self.state_machine_tree.show()

        self.global_var_manager_view = GlobalVariableEditorView()
        self.global_var_manager_view.show()

        # self.graphical_editor_window = gtk.Window()
        # self.graphical_editor_window.set_title("Graphical Editor")
        # self.graphical_editor_window.set_position(1)
        # graphical_editor_view = GraphicalEditorView()
        # self.graphical_editor_window.add(graphical_editor_view.get_top_widget())
        # self.graphical_editor_window.show_all()

        #self.graphical_editor_window = SingleWidgetWindowView(GraphicalEditorView, title="Graphical-Editor", pos=1)

        ##################################################
        # insert one graphical-statemachine-editor
        ##################################################
        # self.graphical_editor_view = GraphicalEditorView()
        # self.graphical_editor_view.show()
        # self['graphical_editor_frame'].add(self.graphical_editor_view['main_frame'])

        ##################################################
        # insert state-machines-editor (graphical)
        ##################################################
        self.state_machines_editor = StateMachinesEditorView()
        self.state_machines_editor.show()
        self['graphical_editor_vbox'].pack_start(self.state_machines_editor.get_top_widget(), True, True, 0)
        self['graphical_editor_vbox'].reorder_child(self.state_machines_editor.get_top_widget(), 1)
        # self['state_machines_vbox'].add(self.state_machines_editor.get_top_widget())

        ##################################################
        # insert states-editor
        ##################################################
        self.states_editor = StatesEditorView()
        self["state_editor"].add(self.states_editor.get_top_widget())
        self.states_editor.show()

        ##################################################
        # menu bar view
        ##################################################
        self.top_tool_bar = TopToolBarView()
        self.top_tool_bar.show()
        self["top_menu_hbox"].remove(self["top_tool_bar_placeholder"])
        self["top_menu_hbox"].pack_end(self.top_tool_bar.get_top_widget(), expand=True, fill=True, padding=0)
        self["top_menu_hbox"].reorder_child(self.top_tool_bar.get_top_widget(), 1)

        self.menu_bar = MenuBarView(self)
        self.menu_bar.show()
        self["top_menu_hbox"].remove(self["menu_bar_placeholder"])
        self["top_menu_hbox"].pack_start(self.menu_bar.get_top_widget(), expand=False, fill=True, padding=0)
        self["top_menu_hbox"].reorder_child(self.menu_bar.get_top_widget(), 0)

        self.tool_bar = ToolBarView()
        self.tool_bar.show()
        self["top_level_vbox"].remove(self["tool_bar_placeholder"])
        self["top_level_vbox"].pack_start(self.tool_bar.get_top_widget(), expand=False, fill=True, padding=0)
        self["top_level_vbox"].reorder_child(self.tool_bar.get_top_widget(), 1)

        # insert global-variable-manager
        #self['global_variable_manager_vbox'].add(self.global_variable_manager.get_top_widget())

        # insert history-overview
        #self['history_vbox'].add(self['history'].get_top_widget())

        # insert logger-console
        #self['console_scroller'].add(self['logger_console'].get_top_widget())

        # insert status-bar
        #self['vbox1'].reparent(self['lower_statusbar'])

        # --------------------------------------------------------------------------
        # Edit graphical_editor_shortcuts
        # --------------------------------------------------------------------------

        button_start_shortcut = self['button_start_shortcut']
        button_pause_shortcut = self['button_pause_shortcut']
        button_stop_shortcut = self['button_stop_shortcut']
        button_step_mode_shortcut = self['button_step_mode_shortcut']
        button_step_shortcut = self['button_step_shortcut']
        button_step_backward_shortcut = self['button_step_backward_shortcut']

        button_start_shortcut.set_label_widget(self.create_button_label(constants.BUTTON_START))
        button_pause_shortcut.set_label_widget(self.create_button_label(constants.BUTTON_PAUSE))
        button_stop_shortcut.set_label_widget(self.create_button_label(constants.BUTTON_STOP))
        button_step_mode_shortcut.set_label_widget(self.create_button_label(constants.BUTTON_STEPM))
        button_step_shortcut.set_label_widget(self.create_button_label(constants.BUTTON_STEP))
        button_step_backward_shortcut.set_label_widget(self.create_button_label(constants.BUTTON_BACKW))

        # --------------------------------------------------------------------------

        self.get_top_widget().set_decorated(False)

        self['main_window'].set_border_width(constants.MAIN_WINDOW_BORDER_WIDTH)

        self['top_menu_hbox'].set_border_width(constants.BORDER_WIDTH)
        self['tree_notebook_1'].set_border_width(constants.BORDER_WIDTH)
        self['tree_notebook_2'].set_border_width(constants.BORDER_WIDTH)
        self['library_event_box'].set_border_width(constants.BORDER_WIDTH)
        self['graphical_editor_vbox'].set_border_width(constants.BORDER_WIDTH)
        self['debug_eventbox'].set_border_width(constants.BORDER_WIDTH)
        self['debug_label_eventbox'].set_border_width(constants.BORDER_WIDTH_TEXTVIEW)
        self['state_editor'].set_border_width(constants.BORDER_WIDTH)

    def create_button_label(self, icon):
        label = gtk.Label()
        label.set_markup('<span font_desc="%s %s">&#x%s;</span>' % (constants.DEFAULT_FONT,
                                                                    constants.FONT_SIZE_NORMAL,
                                                                    icon))
        label.show()
        return label