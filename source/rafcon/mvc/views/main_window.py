import gtk
from gtkmvc import View
from rafcon.mvc.views.logging import LoggingView
from rafcon.mvc.views.library_tree import LibraryTreeView
from rafcon.mvc.views.state_icons import StateIconView
from rafcon.mvc.views.state_machine_tree import StateMachineTreeView
from rafcon.mvc.views.global_variable_editor import GlobalVariableEditorView
from rafcon.mvc.views.state_machine_history import StateMachineHistoryView
from rafcon.mvc.views.execution_history import ExecutionHistoryView
from rafcon.mvc.views.state_machines_editor import StateMachinesEditorView
from rafcon.mvc.views.states_editor import StatesEditorView
from rafcon.mvc.views.top_tool_bar import TopToolBarView
from rafcon.mvc.views.menu_bar import MenuBarView
from rafcon.mvc.views.tool_bar import ToolBarView
from rafcon.mvc.utils import constants
from rafcon.mvc.config import global_gui_config
from rafcon.mvc import gui_helper


class MainWindowView(View):
    builder = './glade/main_window.glade'
    top = 'main_window'

    def __init__(self):
        View.__init__(self)
        # Add gui components by removing their corresponding placeholders defined in the glade file first and then
        # adding the widgets.

        ######################################################
        # Logging
        ######################################################
        self.logging_view = LoggingView()
        self['console'].remove(self['console_scroller'])
        self.logging_view.get_top_widget().show()
        self['console'].pack_start(self.logging_view.get_top_widget(), True, True, 0)

        ######################################################
        # Library Tree
        ######################################################
        self.library_tree = LibraryTreeView()
        self.library_tree.show()
        self.replace_notebook_placeholder_with_widget('libraries', 'tree_notebook_up', self.library_tree)

        ######################################################
        # State Icons
        ######################################################
        self.state_icons = StateIconView()
        self.state_icons.show()
        self["state_icons_box"].pack_start(self.state_icons.get_top_widget())

        ######################################################
        # State Machine Tree
        ######################################################
        self.state_machine_tree = StateMachineTreeView()
        self.state_machine_tree.show()
        self.replace_notebook_placeholder_with_widget('state_tree', 'tree_notebook_up', self.state_machine_tree)
        # TODO: this is not always the active state machine

        ######################################################
        # Global Variable Manager
        ######################################################
        self.global_var_editor = GlobalVariableEditorView()
        self.global_var_editor.show()
        self.replace_notebook_placeholder_with_widget('global_variables', 'tree_notebook_up',
                                                      self.global_var_editor.get_top_widget())

        self['tree_notebook_up'].set_current_page(0)

        ######################################################
        # State Machine History
        ######################################################
        self.state_machine_history = StateMachineHistoryView()
        self.state_machine_history.show()
        self.replace_notebook_placeholder_with_widget('history', 'tree_notebook_down',
                                                      self.state_machine_history.get_top_widget())
                                                      
        ######################################################
        # State Machine Execution History
        ######################################################
        self.execution_history = ExecutionHistoryView()
        self.execution_history.show()
        self.replace_notebook_placeholder_with_widget('execution_history', 'tree_notebook_down',
                                                      self.execution_history.get_top_widget())

        ######################################################
        # rotate all tab labels by 90 degrees and make detachable
        ######################################################
        self.rotate_and_detach_tab_labels(self['tree_notebook_up'])
        self.rotate_and_detach_tab_labels(self['tree_notebook_down'])

        ######################################################
        # State-machines-editor (graphical)
        ######################################################
        self.state_machines_editor = StateMachinesEditorView()
        self.state_machines_editor.show()
        self['graphical_editor_vbox'].pack_start(self.state_machines_editor.get_top_widget(), True, True, 0)
        self['graphical_editor_vbox'].reorder_child(self.state_machines_editor.get_top_widget(), 0)

        self['graphical_editor_label_event_box'].remove(self['graphical_editor_label'])
        self['graphical_editor_label_event_box'].set_border_width(constants.GRID_SIZE)
        graphical_editor_label = gui_helper.create_label_with_text_and_spacing('GRAPHICAL EDITOR',
                                                                               font_size=constants.FONT_SIZE_BIG,
                                                                               letter_spacing=constants.
                                                                               LETTER_SPACING_1PT)
        graphical_editor_label.set_alignment(0., .5)
        self['graphical_editor_label_event_box'].add(graphical_editor_label)

        ######################################################
        # States-editor
        ######################################################
        self.states_editor = StatesEditorView()
        self['state_editor_eventbox'].add(self.states_editor.get_top_widget())
        self.states_editor.show()

        self['state_editor_label_hbox'].remove(self['state_editor_label'])
        self['state_editor_label_hbox'].set_border_width(constants.GRID_SIZE)
        state_editor_label = gui_helper.create_label_with_text_and_spacing('STATE EDITOR',
                                                                           font_size=constants.FONT_SIZE_BIG,
                                                                           letter_spacing=constants.LETTER_SPACING_1PT)
        state_editor_label.set_alignment(0., .5)
        self['state_editor_label_hbox'].add(state_editor_label)
        #self['state_editor_label_hbox'].reorder_child(state_editor_label, 0)

        ##################################################
        # menu bar view
        ##################################################
        self.top_tool_bar = TopToolBarView()
        self.top_tool_bar.show()
        self['top_menu_hbox'].remove(self['top_tool_bar_placeholder'])
        self['top_menu_hbox'].pack_end(self.top_tool_bar.get_top_widget(), expand=True, fill=True, padding=0)
        self['top_menu_hbox'].reorder_child(self.top_tool_bar.get_top_widget(), 1)

        self.menu_bar = MenuBarView(self)
        self.menu_bar.show()
        self['top_menu_hbox'].remove(self['menu_bar_placeholder'])
        self['top_menu_hbox'].pack_start(self.menu_bar.get_top_widget(), expand=False, fill=True, padding=0)
        self['top_menu_hbox'].reorder_child(self.menu_bar.get_top_widget(), 0)

        self.tool_bar = ToolBarView()
        self.tool_bar.show()
        self['top_level_vbox'].remove(self['tool_bar_placeholder'])
        self['top_level_vbox'].pack_start(self.tool_bar.get_top_widget(), expand=False, fill=True, padding=0)
        self['top_level_vbox'].reorder_child(self.tool_bar.get_top_widget(), 1)

        ################################################
        # Hide Buttons
        ################################################
        self['left_bar_hide_button'].set_image(self.create_arrow_label(constants.BUTTON_LEFTA))
        self['right_bar_hide_button'].set_image(self.create_arrow_label(constants.BUTTON_RIGHTA))
        self['console_hide_button'].set_image(self.create_arrow_label(constants.BUTTON_DOWNA))

        ################################################
        # Return Buttons
        ################################################
        self['left_bar_return_button'].set_image(self.create_arrow_label(constants.BUTTON_RIGHTA))
        self['right_bar_return_button'].set_image(self.create_arrow_label(constants.BUTTON_LEFTA))
        self['console_return_button'].set_image(self.create_arrow_label(constants.BUTTON_UPA))

        ################################################
        # Undock Buttons
        ################################################
        self['undock_right_bar_button'].set_image(self.create_arrow_label(constants.BUTTON_UNDOCK))
        self['undock_left_bar_button'].set_image(self.create_arrow_label(constants.BUTTON_UNDOCK))
        self['undock_console_button'].set_image(self.create_arrow_label(constants.BUTTON_UNDOCK))

        # --------------------------------------------------------------------------
        # Edit graphical_editor_shortcuts
        # --------------------------------------------------------------------------

        button_start_shortcut = self['button_start_shortcut']
        button_pause_shortcut = self['button_pause_shortcut']
        button_stop_shortcut = self['button_stop_shortcut']
        button_step_mode_shortcut = self['button_step_mode_shortcut']
        button_step_in_shortcut = self['button_step_in_shortcut']
        button_step_over_shortcut = self['button_step_over_shortcut']
        button_step_out_shortcut = self['button_step_out_shortcut']
        button_step_backward_shortcut = self['button_step_backward_shortcut']

        button_start_shortcut.set_label_widget(self.create_button_label(constants.BUTTON_START))
        button_pause_shortcut.set_label_widget(self.create_button_label(constants.BUTTON_PAUSE))
        button_stop_shortcut.set_label_widget(self.create_button_label(constants.BUTTON_STOP))
        button_step_mode_shortcut.set_label_widget(self.create_button_label(constants.BUTTON_STEPM))
        button_step_in_shortcut.set_label_widget(self.create_button_label(constants.BUTTON_STEP_INTO))
        button_step_over_shortcut.set_label_widget(self.create_button_label(constants.BUTTON_STEP_OVER))
        button_step_out_shortcut.set_label_widget(self.create_button_label(constants.BUTTON_STEP_OUT))
        button_step_backward_shortcut.set_label_widget(self.create_button_label(constants.BUTTON_BACKW))

        # --------------------------------------------------------------------------

        self.get_top_widget().set_decorated(False)

        # self['main_window'].set_border_width(constants.MAIN_WINDOW_BORDER_WIDTH)

        # self['top_menu_hbox'].set_border_width(constants.BORDER_WIDTH)

        # self['tree_notebook_up'].set_border_width(constants.BORDER_WIDTH)
        self['tree_notebook_up'].set_tab_hborder(constants.BORDER_WIDTH * 2)
        self['tree_notebook_up'].set_tab_vborder(constants.BORDER_WIDTH * 3)

        # self['tree_notebook_down'].set_border_width(constants.BORDER_WIDTH)
        self['tree_notebook_down'].set_tab_hborder(constants.BORDER_WIDTH * 2)
        self['tree_notebook_down'].set_tab_vborder(constants.BORDER_WIDTH * 3)

        #self['library_event_box'].set_border_width(constants.BORDER_WIDTH)
        # self['debug_eventbox'].set_border_width(constants.BORDER_WIDTH)
        # self['debug_label_hbox'].set_border_width(constants.BORDER_WIDTH_TEXTVIEW)

        # self['right_bar'].set_border_width(constants.BORDER_WIDTH)

        self['button_show_info'].set_active(global_gui_config.get_config_value('LOGGING_SHOW_INFO', True))
        self['button_show_debug'].set_active(global_gui_config.get_config_value('LOGGING_SHOW_DEBUG', True))
        self['button_show_warning'].set_active(global_gui_config.get_config_value('LOGGING_SHOW_WARNING', True))
        self['button_show_error'].set_active(global_gui_config.get_config_value('LOGGING_SHOW_ERROR', True))

        self.logging_view.update_filtered_buffer()

        ######################################################
        # setup correct sizes
        ######################################################
        self['top_level_h_pane'].set_position(300)
        self['right_h_pane'].set_position(1000)
        self['left_bar'].set_position(400)
        self['central_v_pane'].set_position(600)

        self.left_bar_window = gtk.Window()
        self.right_bar_window = gtk.Window()
        self.console_window = gtk.Window()
        
        self.top_window_width = self['main_window'].get_size()[0]

    @staticmethod
    def create_button_label(icon):
        label = gtk.Label()
        label.set_markup('<span font_desc="%s %s">&#x%s;</span>' % (constants.ICON_FONT, constants.FONT_SIZE_NORMAL,
                                                                    icon))
        label.show()
        return label

    def create_notebook_widget(self, title, widget, use_scroller=True, border=constants.GRID_SIZE):
        title_label = self.create_label_box(title)
        event_box = gtk.EventBox()
        vbox = gtk.VBox()
        vbox.pack_start(title_label, False, True, 0)
        if use_scroller:
            scroller = gtk.ScrolledWindow()
            scroller.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
            alig = gtk.Alignment(0., 0., 1., 1.)
            alig.set_padding(0, 0, 0, 0)
            alig.add(widget)
            alig.show()
            scroller.add_with_viewport(alig)
            vbox.pack_start(scroller, True, True, 0)
        else:
            vbox.pack_start(widget, True, True, 0)
        event_box.add(vbox)
        event_box.show_all()
        return event_box

    def create_label_box(self, text):
        """Creates a horizontal box containing a label that shows a title.

        :param text: The title to be shown
        :return: The horizontal box
        """
        hbox = gtk.HBox()
        label = gui_helper.create_label_with_text_and_spacing(text, font_size=constants.FONT_SIZE_BIG,
                                                              letter_spacing=constants.LETTER_SPACING_1PT)
        label.set_alignment(0.0, 0.5)
        hbox.pack_start(label, True, True, 0)
        hbox.set_border_width(constants.BORDER_WIDTH_TEXTVIEW)
        hbox.set_name('widget_title')
        return hbox

    @staticmethod
    def rotate_and_detach_tab_labels(notebook):
        """Rotates tab labels of a given notebook by 90 degrees and makes them detachable.

        :param notebook: GTK Notebook container, whose tab labels are to be rotated and made detachable
        """
        icons = {'libraries': constants.SIGN_LIB, 'state_tree': constants.ICON_TREE,
                 'global_variables': constants.ICON_GLOB, 'history': constants.ICON_HIST,
                 'execution_history': constants.ICON_EHIST, 'network': constants.ICON_NET}
        for i in range(notebook.get_n_pages()):
            child = notebook.get_nth_page(i)
            tab_label = notebook.get_tab_label(child)
            if global_gui_config.get_config_value('USE_ICONS_AS_TAB_LABELS', True):
                tab_label_text = tab_label.get_text()
                notebook.set_tab_label(child, gui_helper.create_tab_header_label(tab_label_text, icons))
            else:
                tab_label.set_angle(90)
            notebook.set_tab_reorderable(child, True)
            notebook.set_tab_detachable(child, True)

    @staticmethod
    def create_arrow_label(icon):
        label = gtk.Label()
        label.set_markup('<span font_desc="%s %s">&#x%s;</span>' % (constants.ICON_FONT, constants.FONT_SIZE_BIG, icon))
        return label

    def replace_notebook_placeholder_with_widget(self, tab_label, notebook, view):
        """Replace notebook's placeholder with widget by removing the placeholder, creating a widget, and assigning the
        widget to the corresponding tab in the notebook.

        :param tab_label: String containing the label of the tab containing the placeholder to be removed
        :param notebook: Name of the notebook
        :param view: The widget to replace the placeholder
        """
        placeholder = tab_label + '_placeholder'
        page_num = self[notebook].page_num(self[placeholder])
        self[notebook].remove_page(page_num)
        notebook_widget = self.create_notebook_widget(get_widget_title(tab_label), view,
                                                      border=constants.BORDER_WIDTH_TEXTVIEW)
        self[notebook].insert_page(notebook_widget, gtk.Label(tab_label), page_num)


def get_widget_title(tab_label):
    """Transform tab label to title by replacing underscores with white spaces and capitalizing the first letter of each
    word.

    :param tab_label: The string of the tab label to be transformed
    :return: The transformed title as a string
    """
    title = ''
    title_list = tab_label.split('_')
    for word in title_list:
        title += word.upper() + ' '
    title.strip()
    return title
