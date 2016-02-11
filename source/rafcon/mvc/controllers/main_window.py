import gtk
import threading

from rafcon.mvc.controllers.global_variable_manager import GlobalVariableManagerController
from rafcon.mvc.controllers.state_icons import StateIconController
from rafcon.mvc.controllers.state_machine_tree import StateMachineTreeController
from rafcon.mvc.controllers.state_machine_history import StateMachineHistoryController
from rafcon.mvc.controllers.library_tree import LibraryTreeController

from rafcon.mvc.models.state_machine_manager import StateMachineManagerModel
from rafcon.mvc.models.library_manager import LibraryManagerModel
from rafcon.mvc.shortcut_manager import ShortcutManager

from rafcon.mvc.controllers.extended_controller import ExtendedController
from rafcon.mvc.controllers.states_editor import StatesEditorController
from rafcon.mvc.controllers.state_machines_editor import StateMachinesEditorController
from rafcon.mvc.controllers.menu_bar import MenuBarController
from rafcon.mvc.controllers.tool_bar import ToolBarController
from rafcon.mvc.controllers.top_tool_bar import TopToolBarMainWindowController
from rafcon.mvc.controllers.execution_history import ExecutionHistoryTreeController
from rafcon.mvc.controllers.undocked_window import UndockedWindowController

from rafcon.statemachine.enums import StateMachineExecutionStatus

from rafcon.mvc.singleton import global_variable_manager_model as gvm_model
import rafcon.statemachine.singleton
import rafcon.statemachine.config
from rafcon.mvc.config import global_gui_config as gui_config
from rafcon.network.network_config import global_net_config

from rafcon.mvc.utils import constants
from rafcon.mvc import gui_helper
from rafcon.utils import log

logger = log.get_logger(__name__)
try:
    # run if not defined or variable True
    if global_net_config.get_config_value("NETWORK_CONNECTIONS") is None or global_net_config.get_config_value(
            "NETWORK_CONNECTIONS"):
        from rafcon.mvc.controllers.network_connections import NetworkController
        from rafcon.network.singleton import network_connections
except ImportError as e:
    logger.warn(
        "{1} Only local use of RAFCON will be possible due to missing network communication libraries -> {0}".format(
            e.message, global_net_config.get_config_value("NETWORK_CONNECTIONS") is None))
    # logger.error("%s, %s" % (e.message, traceback.format_exc()))
    global_net_config.set_config_value('NETWORK_CONNECTIONS', False)


class MainWindowController(ExtendedController):
    """Controller handling the main window.

    :param rafcon.mvc.models.state_machine_manager.StateMachineManagerModel state_machine_manager_model: The state
        machine manager model, holding data regarding state machines. Should be exchangeable.
    :param rafcon.mvc.views.main_window.MainWindowView view: The GTK View showing the main window.
    :ivar docked: Dict holding mappings between bars/console and their current docking-status.
    """

    def __init__(self, state_machine_manager_model, view, editor_type='PortConnectionGrouped'):
        ExtendedController.__init__(self, state_machine_manager_model, view)

        rafcon.mvc.singleton.main_window_controller = self
        self.state_machine_manager_model = state_machine_manager_model
        self.editor_type = editor_type
        self.shortcut_manager = None

        # state machine manager
        assert isinstance(state_machine_manager_model, StateMachineManagerModel)
        state_machine_manager = state_machine_manager_model.state_machine_manager

        # execution engine
        self.state_machine_execution_engine = rafcon.statemachine.singleton.state_machine_execution_engine
        self.observe_model(self.state_machine_execution_engine)
        self.state_machine_execution_engine.register_observer(self)

        # shortcut manager
        self.shortcut_manager = ShortcutManager(view['main_window'])

        # library tree
        library_manager_model = LibraryManagerModel(rafcon.statemachine.singleton.library_manager)
        library_controller = LibraryTreeController(library_manager_model, view.library_tree,
                                                   state_machine_manager_model)
        self.add_controller('library_controller', library_controller)
        # view['main_window'].add_events(
        #    gtk.gdk.BUTTON_PRESS_MASK | gtk.gdk.BUTTON_RELEASE_MASK | gtk.gdk.BUTTON_MOTION_MASK |
        #    gtk.gdk.KEY_PRESS_MASK | gtk.gdk.KEY_RELEASE_MASK | gtk.gdk.POINTER_MOTION_MASK)

        ######################################################
        # state icons
        ######################################################
        state_icon_controller = StateIconController(state_machine_manager_model, view.state_icons,
                                                    self.shortcut_manager)
        self.add_controller('state_icon_controller', state_icon_controller)

        state_machine_tree_controller = StateMachineTreeController(state_machine_manager_model, view.state_machine_tree)
        self.add_controller('state_machine_tree_controller', state_machine_tree_controller)

        # state editor
        states_editor_ctrl = StatesEditorController(state_machine_manager_model, view.states_editor, editor_type)
        self.add_controller('states_editor_ctrl', states_editor_ctrl)

        # state machines editor
        state_machines_editor_ctrl = StateMachinesEditorController(state_machine_manager_model,
                                                                   view.state_machines_editor)
        self.add_controller('state_machines_editor_ctrl', state_machines_editor_ctrl)

        # global variable editor
        global_variable_manager_ctrl = GlobalVariableManagerController(gvm_model, view.global_var_editor)
        self.add_controller('global_variable_manager_ctrl', global_variable_manager_ctrl)

        ######################################################
        # state machine edition history
        ######################################################
        state_machine_history_controller = StateMachineHistoryController(state_machine_manager_model,
                                                                         view.state_machine_history)
        self.add_controller('state_machine_history_controller', state_machine_history_controller)

        ######################################################
        # network controller
        ######################################################
        if global_net_config.get_config_value('NETWORK_CONNECTIONS', False):
            from rafcon.mvc.controllers.network_connections import NetworkController
            from rafcon.network.singleton import network_connections
            from rafcon.mvc.views.network_connections import NetworkConnectionsView
            network_connections_view = NetworkConnectionsView()
            network_connections_ctrl = NetworkController(state_machine_manager_model, network_connections_view)
            network_connections.initialize()
            self.add_controller('network_connections_ctrl', network_connections_ctrl)

            network_tab = view['network_placeholder']
            page_num = view['lower_notebook'].page_num(network_tab)
            view['lower_notebook'].remove_page(page_num)

            network_label = gtk.Label('Network')

            network_notebook_widget = view.create_notebook_widget('NETWORK',  network_connections_view.get_top_widget(),
                                                                  use_scroller=False,
                                                                  border=constants.BORDER_WIDTH_TEXTVIEW)

            view['lower_notebook'].insert_page(network_notebook_widget, network_label, page_num)
        else:
            network_tab = view['network_tab']
            page_num = view['lower_notebook'].page_num(network_tab)
            view['lower_notebook'].remove_page(page_num)

        ######################################################
        # state machine execution history
        ######################################################
        execution_history_ctrl = ExecutionHistoryTreeController(state_machine_manager_model, view.execution_history,
                                                                state_machine_manager)
        self.add_controller('execution_history_ctrl', execution_history_ctrl)

        ######################################################
        # menu bar
        ######################################################
        menu_bar_controller = MenuBarController(state_machine_manager_model,
                                                view,
                                                state_machines_editor_ctrl,
                                                states_editor_ctrl,
                                                view.logging_view,
                                                view.get_top_widget(),
                                                self.shortcut_manager)
        self.add_controller('menu_bar_controller', menu_bar_controller)

        ######################################################
        # tool bar
        ######################################################
        tool_bar_controller = ToolBarController(state_machine_manager_model, view.tool_bar, menu_bar_controller)
        self.add_controller('tool_bar_controller', tool_bar_controller)

        ######################################################
        # top tool bar
        ######################################################
        top_tool_bar_controller = TopToolBarMainWindowController(state_machine_manager_model, view.top_tool_bar,
                                                                 view['main_window'])
        self.add_controller('top_tool_bar_controller', top_tool_bar_controller)

        ######################################################
        # Undocked Windows Controllers
        ######################################################
        left_undocked_window_controller = UndockedWindowController(state_machine_manager_model, view.left_bar_window)
        self.add_controller('left_window_controller', left_undocked_window_controller)

        right_undocked_window_controller = UndockedWindowController(state_machine_manager_model, view.right_bar_window)
        self.add_controller('right_window_controller', right_undocked_window_controller)

        console_undocked_window_controller = UndockedWindowController(state_machine_manager_model, view.console_window)
        self.add_controller('console_window_controller', console_undocked_window_controller)

        self.left_bar_child = view['top_level_h_pane'].get_child1()
        self.right_bar_child = view['right_h_pane'].get_child2()
        self.console_child = view['central_v_pane'].get_child2()

        self.docked = {'left_bar': True, 'right_bar': True, 'console': True}

        view['debug_console_button_hbox'].reorder_child(view['button_show_error'], 0)
        view['debug_console_button_hbox'].reorder_child(view['button_show_warning'], 1)
        view['debug_console_button_hbox'].reorder_child(view['button_show_info'], 2)
        view['debug_console_button_hbox'].reorder_child(view['button_show_debug'], 3)

        # Initialize the Left-Bar Notebooks' titles according to initially-selected tabs
        upper_title = gui_helper.set_notebook_title(view['upper_notebook'], view['upper_notebook'].get_current_page(),
                                                    view['upper_notebook_title'])
        lower_title = gui_helper.set_notebook_title(view['lower_notebook'], view['lower_notebook'].get_current_page(),
                                                    view['lower_notebook_title'])

        # Initialize the Left-Bar un-docked window title
        view.left_bar_window.initialize_title(gui_helper.create_left_bar_window_title(upper_title, lower_title))
        view.right_bar_window.initialize_title('STATE EDITOR')
        view.console_window.initialize_title('CONSOLE')

    def register_view(self, view):
        self.register_actions(self.shortcut_manager)
        view['main_window'].connect('delete_event', self.get_controller('menu_bar_controller').on_delete_event)
        view['main_window'].connect('destroy', self.get_controller('menu_bar_controller').on_destroy)

        # connect left bar, right bar and console hide buttons' signals to their corresponding methods
        view['left_bar_hide_button'].connect('clicked', self.on_left_bar_hide_clicked)
        view['right_bar_hide_button'].connect('clicked', self.on_right_bar_hide_clicked)
        view['console_hide_button'].connect('clicked', self.on_console_hide_clicked)

        # Connect left bar, right bar and console return buttons' signals to their corresponding methods
        view['left_bar_return_button'].connect('clicked', self.on_left_bar_return_clicked)
        view['right_bar_return_button'].connect('clicked', self.on_right_bar_return_clicked)
        view['console_return_button'].connect('clicked', self.on_console_return_clicked)

        # Connect undock buttons' signals
        view['undock_left_bar_button'].connect('clicked', self.on_left_bar_undock_clicked)
        view['undock_right_bar_button'].connect('clicked', self.on_right_bar_undock_clicked)
        view['undock_console_button'].connect('clicked', self.on_console_undock_clicked)

        # Connect Shortcut buttons' signals to their corresponding methods
        view['button_start_shortcut'].connect('toggled', self.on_button_start_shortcut_toggled)
        view['button_stop_shortcut'].connect('clicked', self.on_button_stop_shortcut_clicked)
        view['button_pause_shortcut'].connect('toggled', self.on_button_pause_shortcut_toggled)
        view['button_step_mode_shortcut'].connect('toggled', self.on_button_step_mode_shortcut_toggled)
        view['button_step_in_shortcut'].connect('clicked', self.on_button_step_in_shortcut_clicked)
        view['button_step_over_shortcut'].connect('clicked', self.on_button_step_over_shortcut_clicked)
        view['button_step_out_shortcut'].connect('clicked', self.on_button_step_out_shortcut_clicked)
        view['button_step_backward_shortcut'].connect('clicked', self.on_button_step_backward_shortcut_clicked)

        # Connect Debug console buttons' signals to their corresponding methods
        view['button_show_debug'].connect('toggled', self.on_debug_content_change)
        view['button_show_info'].connect('toggled', self.on_debug_content_change)
        view['button_show_warning'].connect('toggled', self.on_debug_content_change)
        view['button_show_error'].connect('toggled', self.on_debug_content_change)

        view['upper_notebook'].connect('switch-page', self.on_notebook_tab_switch, view['upper_notebook_title'],
                                       view.left_bar_window, 'upper')
        view['lower_notebook'].connect('switch-page', self.on_notebook_tab_switch, view['lower_notebook_title'],
                                       view.left_bar_window, 'lower')

        # hide not usable buttons
        self.view['step_buttons'].hide()

    def highlight_execution_of_current_sm(self, active):
        notebook = self.get_controller('state_machines_editor_ctrl').view['notebook']
        page_num = self.get_controller('state_machines_editor_ctrl').view['notebook'].get_current_page()
        page = self.get_controller('state_machines_editor_ctrl').view['notebook'].get_nth_page(page_num)
        label = notebook.get_tab_label(page).get_children()[0]
        if active:
            label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse(gui_config.colors['STATE_MACHINE_ACTIVE']))
            label.modify_fg(gtk.STATE_INSENSITIVE, gtk.gdk.color_parse(gui_config.colors['STATE_MACHINE_ACTIVE']))
        else:
            label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse(gui_config.colors['STATE_MACHINE_NOT_ACTIVE']))
            label.modify_fg(gtk.STATE_INSENSITIVE, gtk.gdk.color_parse(gui_config.colors['STATE_MACHINE_NOT_ACTIVE']))

    @ExtendedController.observe("execution_engine", after=True)
    def model_changed(self, model, prop_name, info):
        label_string = str(rafcon.statemachine.singleton.state_machine_execution_engine.status.execution_mode)
        label_string = label_string.replace("STATE_MACHINE_EXECUTION_STATUS.", "")
        self.view['execution_status_label'].set_text(label_string)

        if rafcon.statemachine.singleton.state_machine_execution_engine.status.execution_mode is StateMachineExecutionStatus.STARTED:
            self.highlight_execution_of_current_sm(True)
            self.view['step_buttons'].hide()
            self._set_single_button_active('button_start_shortcut')
        elif rafcon.statemachine.singleton.state_machine_execution_engine.status.execution_mode is StateMachineExecutionStatus.PAUSED:
            self.delay(100, self.get_controller('execution_history_ctrl').update)
            self.highlight_execution_of_current_sm(True)
            self.view['step_buttons'].hide()
            self._set_single_button_active('button_pause_shortcut')
        elif rafcon.statemachine.singleton.state_machine_execution_engine.status.execution_mode is StateMachineExecutionStatus.STOPPED:
            self.delay(100, self.get_controller('execution_history_ctrl').update)
            self.highlight_execution_of_current_sm(False)
            self.view['step_buttons'].hide()
            self._set_single_button_active('button_stop_shortcut')
        else:  # all step modes
            self.delay(100, self.get_controller('execution_history_ctrl').update)
            self.highlight_execution_of_current_sm(True)
            self.view['step_buttons'].show()
            self._set_single_button_active('button_step_mode_shortcut')

    def _set_single_button_active(self, active_button_name):
        for button_name in ['button_start_shortcut', 'button_pause_shortcut', 'button_step_mode_shortcut']:
            if active_button_name == button_name:
                if not self.view[button_name].get_active():
                    self.view[button_name].set_active(True)
            else:
                if self.view[button_name].get_active():
                    self.view[button_name].set_active(False)

    def on_left_bar_return_clicked(self, widget, event=None):
        self.view['left_bar_return_button'].hide()
        self.view['top_level_h_pane'].add1(self.left_bar_child)

    def on_right_bar_return_clicked(self, widget, event=None):
        self.view['right_bar_return_button'].hide()
        self.view['right_h_pane'].add2(self.right_bar_child)

    def on_console_return_clicked(self, widget, event=None):
        self.view['console_return_button'].hide()
        self.view['central_v_pane'].add2(self.console_child)

    def on_left_bar_hide_clicked(self, widget, event=None):
        self.view['top_level_h_pane'].remove(self.left_bar_child)
        self.view['left_bar_return_button'].show()

    def on_right_bar_hide_clicked(self, widget, event=None):
        self.view['right_h_pane'].remove(self.right_bar_child)
        self.view['right_bar_return_button'].show()

    def on_console_hide_clicked(self, widget, event=None):
        self.view['central_v_pane'].remove(self.console_child)
        self.view['console_return_button'].show()

    def on_left_bar_undock_clicked(self, widget, event=None):
        """Triggered when the undock button of the left bar window.

        The left bar is undocked into a separate new window with the same size as the bar. The new window's position is
        approximately the same as the bar used to be.
        """
        self.view.left_bar_window.get_top_widget().resize(self.view['top_level_h_pane'].get_position(),
                                                          self.view['left_bar'].get_allocation().height)
        self.view.left_bar_window.get_top_widget().set_position(gtk.WIN_POS_MOUSE)
        self.view['left_bar'].reparent(self.view.left_bar_window['top_level_vbox'])
        self.get_controller('left_window_controller').show_window()
        self.view['undock_left_bar_button'].hide()
        self.on_left_bar_hide_clicked(None)
        self.view['left_bar_return_button'].hide()

    def on_left_bar_dock_clicked(self, widget, event=None):
        """Triggered when the dock button of the left bar window.

        If the left bar is undocked, it is docked back to the main window.
        """
        self.on_left_bar_return_clicked(None)
        self.view['left_bar'].reparent(self.view['left_bar_container'])
        #window_width, window_height = self.view.left_bar_window.get_size()
        #self.view['top_level_h_pane'].set_position(window_width)
        self.get_controller('left_window_controller').hide_window()
        self.view['undock_left_bar_button'].show()
        return True

    def on_right_bar_undock_clicked(self, widget, event=None):
        width = self.view.top_window_width - self.view['right_h_pane'].get_position() -\
                self.view['top_level_h_pane'].get_position()
        self.view.right_bar_window.get_top_widget().resize(width, self.view['right_bar'].get_allocation().height)
        self.view.right_bar_window.get_top_widget().set_position(gtk.WIN_POS_MOUSE)
        self.view['right_bar'].reparent(self.view.right_bar_window['top_level_vbox'])
        self.get_controller('right_window_controller').show_window()
        self.view['undock_right_bar_button'].hide()
        self.on_right_bar_hide_clicked(None)
        self.view['right_bar_return_button'].hide()

    def on_right_bar_dock_clicked(self, widget, event=None):
        """Triggered when the dock button of the right bar is clicked.

        If right bar is docked to main window, it is undocked into a separate new window with the same size as the bar.
        The new window's position is approximately the same as the bar used to be.
        """
        self.view['right_bar'].reparent(self.view['right_bar_container'])
        self.on_right_bar_return_clicked(None)
        self.get_controller('right_window_controller').hide_window()
        self.docked['right_bar'] = True
        self.view['undock_right_bar_button'].show()

    def on_console_undock_clicked(self, widget, event=None):
        self.view.console_window.get_top_widget().resize(self.view['right_h_pane'].get_position(),
                                                         self.view['console'].get_allocation().height)
        self.view.console_window.get_top_widget().move(self.view['top_level_h_pane'].get_position(),
                                                       self.view['central_v_pane'].get_position())
        self.view['console'].reparent(self.view.console_window['top_level_vbox'])
        self.get_controller('console_window_controller').show_window()
        self.view['undock_console_button'].hide()
        self.on_console_hide_clicked(None)
        self.view['console_return_button'].hide()

    def on_console_dock_clicked(self, widget, event=None):
        self.view['console'].reparent(self.view['console_container'])
        self.on_console_return_clicked(None)
        self.get_controller('console_window_controller').hide_window()
        self.view['undock_console_button'].show()

    # Shortcut buttons
    def on_button_start_shortcut_toggled(self, widget, event=None):
        if self.view['button_start_shortcut'].get_active():
            self.get_controller('menu_bar_controller').on_start_activate(None)

    def on_button_pause_shortcut_toggled(self, widget, event=None):
        if self.view['button_pause_shortcut'].get_active():
            self.get_controller('menu_bar_controller').on_pause_activate(None)

    def on_button_stop_shortcut_clicked(self, widget, event=None):
        self.get_controller('menu_bar_controller').on_stop_activate(None)

    def on_button_step_mode_shortcut_toggled(self, widget, event=None):
        if self.view['button_step_mode_shortcut'].get_active():
            self.get_controller("menu_bar_controller").on_step_mode_activate(None)

    def on_button_step_in_shortcut_clicked(self, widget, event=None):
        self.get_controller('menu_bar_controller').on_step_into_activate(None)
        self.delay(100, self.get_controller('execution_history_ctrl').update)

    def on_button_step_over_shortcut_clicked(self, widget, event=None):
        self.get_controller('menu_bar_controller').on_step_over_activate(None)
        self.delay(100, self.get_controller('execution_history_ctrl').update)

    def on_button_step_out_shortcut_clicked(self, widget, event=None):
        self.get_controller('menu_bar_controller').on_step_out_activate(None)
        self.delay(100, self.get_controller('execution_history_ctrl').update)

    def on_button_step_backward_shortcut_clicked(self, widget, event=None):
        self.get_controller('menu_bar_controller').on_backward_step_activate(None)
        self.delay(100, self.get_controller('execution_history_ctrl').update)

    def on_debug_content_change(self, widget, data=None):
        if self.view['button_show_info'].get_active():
            gui_config.set_config_value('LOGGING_SHOW_INFO', True)
        else:
            gui_config.set_config_value('LOGGING_SHOW_INFO', False)
        if self.view['button_show_debug'].get_active():
            gui_config.set_config_value('LOGGING_SHOW_DEBUG', True)
        else:
            gui_config.set_config_value('LOGGING_SHOW_DEBUG', False)
        if self.view['button_show_warning'].get_active():
            gui_config.set_config_value('LOGGING_SHOW_WARNING', True)
        else:
            gui_config.set_config_value('LOGGING_SHOW_WARNING', False)
        if self.view['button_show_error'].get_active():
            gui_config.set_config_value('LOGGING_SHOW_ERROR', True)
        else:
            gui_config.set_config_value('LOGGING_SHOW_ERROR', False)
        # gui_config.save_configuration()
        self.view.logging_view.update_filtered_buffer()

    @staticmethod
    def on_notebook_tab_switch(notebook, page, page_num, title_label, window, notebook_identifier):
        """Triggered whenever a left-bar notebook tab is changed.

        Updates the title of the corresponding notebook and updates the title of the left-bar window in case un-docked.

        :param notebook: The GTK notebook where a tab-change occurred
        :param page_num: The page number of the currently-selected tab
        :param title_label: The label holding the notebook's title
        :param window: The left-bar window, for which the title should be changed
        :param notebook_identifier: A string identifying whether the notebook is the upper or the lower one
        """
        title = gui_helper.set_notebook_title(notebook, page_num, title_label)
        window.reset_title(title, notebook_identifier)

    @staticmethod
    def delay(milliseconds, func):
        thread = threading.Timer(milliseconds / 1000.0, func)
        thread.start()
