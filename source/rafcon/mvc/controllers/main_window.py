"""
.. module:: main_window
   :platform: Unix, Windows
   :synopsis: The module holds the main window controller giving footage to the overall gui.

.. moduleauthor:: Franz Steinmetz


"""

import gtk

from rafcon.mvc.controllers.global_variable_manager import GlobalVariableManagerController
from rafcon.mvc.controllers.state_icons import StateIconController
from rafcon.mvc.controllers.state_machine_tree import StateMachineTreeController
from rafcon.mvc.controllers.modification_history import ModificationHistoryTreeController
from rafcon.mvc.controllers.library_tree import LibraryTreeController
from rafcon.mvc.models.state_machine_manager import StateMachineManagerModel
from rafcon.mvc.models.library_manager import LibraryManagerModel
from rafcon.mvc.shortcut_manager import ShortcutManager
from rafcon.mvc.controllers.utils.extended_controller import ExtendedController
from rafcon.mvc.controllers.states_editor import StatesEditorController
from rafcon.mvc.controllers.state_machines_editor import StateMachinesEditorController
from rafcon.mvc.controllers.menu_bar import MenuBarController
from rafcon.mvc.controllers.tool_bar import ToolBarController
from rafcon.mvc.controllers.top_tool_bar import TopToolBarMainWindowController
from rafcon.mvc.controllers.execution_history import ExecutionHistoryTreeController
from rafcon.mvc.controllers.undocked_window import UndockedWindowController
from rafcon.statemachine.enums import StateMachineExecutionStatus
import rafcon.mvc.singleton as mvc_singleton
import rafcon.statemachine.singleton
import rafcon.statemachine.config
from rafcon.mvc.config import global_gui_config as gui_config
from rafcon.mvc.runtime_config import global_runtime_config
from rafcon.mvc.utils import constants
from rafcon.mvc import gui_helper
from rafcon.utils import log

logger = log.get_logger(__name__)


class MainWindowController(ExtendedController):
    """Controller handling the main window.

    :param rafcon.mvc.models.state_machine_manager.StateMachineManagerModel state_machine_manager_model: The state
        machine manager model, holding data regarding state machines. Should be exchangeable.
    :param rafcon.mvc.views.main_window.MainWindowView view: The GTK View showing the main window.
    :ivar docked: Dict holding mappings between bars/console and their current docking-status.
    """

    def __init__(self, state_machine_manager_model, view, editor_type='PortConnectionGrouped'):
        ExtendedController.__init__(self, state_machine_manager_model, view)

        mvc_singleton.main_window_controller = self
        self.state_machine_manager_model = state_machine_manager_model
        self.editor_type = editor_type
        self.shortcut_manager = None
        self.handler_ids = {}

        # state machine manager
        assert isinstance(state_machine_manager_model, StateMachineManagerModel)
        state_machine_manager = state_machine_manager_model.state_machine_manager

        self.state_machine_execution_model = mvc_singleton.state_machine_execution_model
        self.observe_model(self.state_machine_execution_model)
        self.state_machine_execution_model.register_observer(self)

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

        # state machine tree
        state_machine_tree_controller = StateMachineTreeController(state_machine_manager_model, view.state_machine_tree)
        self.add_controller('state_machine_tree_controller', state_machine_tree_controller)

        # states editor
        states_editor_ctrl = StatesEditorController(state_machine_manager_model, view.states_editor, editor_type)
        self.add_controller('states_editor_ctrl', states_editor_ctrl)

        # state machines editor
        state_machines_editor_ctrl = StateMachinesEditorController(state_machine_manager_model,
                                                                   view.state_machines_editor)
        self.add_controller('state_machines_editor_ctrl', state_machines_editor_ctrl)

        # global variable editor
        global_variable_manager_ctrl = GlobalVariableManagerController(mvc_singleton.global_variable_manager_model,
                                                                       view.global_var_editor)
        self.add_controller('global_variable_manager_ctrl', global_variable_manager_ctrl)

        ######################################################
        # modification history
        ######################################################
        if gui_config.get_config_value('HISTORY_ENABLED'):
            state_machine_history_controller = ModificationHistoryTreeController(state_machine_manager_model,
                                                                                 view.state_machine_history)
            self.add_controller('state_machine_history_controller', state_machine_history_controller)
        self.modification_history_was_focused = False

        ######################################################
        # state machine execution history
        ######################################################
        execution_history_ctrl = ExecutionHistoryTreeController(state_machine_manager_model, view.execution_history,
                                                                state_machine_manager)
        self.add_controller('execution_history_ctrl', execution_history_ctrl)

        ######################################################
        # menu bar
        ######################################################
        menu_bar_controller = MenuBarController(state_machine_manager_model, view, self.shortcut_manager,
                                                rafcon.statemachine.singleton.state_machine_execution_engine)
        self.add_controller('menu_bar_controller', menu_bar_controller)

        ######################################################
        # tool bar
        ######################################################
        tool_bar_controller = ToolBarController(state_machine_manager_model, view.tool_bar)
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

        console_undocked_window_controller = UndockedWindowController(state_machine_manager_model,
                                                                      view.console_bar_window)
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
        view.console_bar_window.initialize_title('CONSOLE')

    def register_view(self, view):
        self.register_actions(self.shortcut_manager)

        # using helper function to connect functions to GUI elements to be able to access the handler id later on

        self.connect_button_to_function('main_window',
                                        "delete_event",
                                        self.get_controller('menu_bar_controller').on_delete_event)
        self.connect_button_to_function('main_window',
                                        "destroy",
                                        self.get_controller('menu_bar_controller').on_destroy)

        # connect left bar, right bar and console hide buttons' signals to their corresponding methods
        self.connect_button_to_function('left_bar_hide_button', "clicked", self.on_left_bar_hide_clicked)
        self.connect_button_to_function('right_bar_hide_button', "clicked", self.on_right_bar_hide_clicked)
        self.connect_button_to_function('console_hide_button', "clicked", self.on_console_hide_clicked)

        self.connect_button_to_function('left_bar_hide_button', "clicked", self.on_left_bar_hide_clicked)
        self.connect_button_to_function('right_bar_hide_button', "clicked", self.on_right_bar_hide_clicked)
        self.connect_button_to_function('console_hide_button', "clicked", self.on_console_hide_clicked)

        # Connect left bar, right bar and console return buttons' signals to their corresponding methods
        self.connect_button_to_function('left_bar_return_button', "clicked", self.on_left_bar_return_clicked)
        self.connect_button_to_function('right_bar_return_button', "clicked", self.on_right_bar_return_clicked)
        self.connect_button_to_function('console_return_button', "clicked", self.on_console_return_clicked)

        # Connect undock buttons' signals
        self.connect_button_to_function('undock_left_bar_button', "clicked", self.on_left_bar_undock_clicked)
        self.connect_button_to_function('undock_right_bar_button', "clicked", self.on_right_bar_undock_clicked)
        self.connect_button_to_function('undock_console_button', "clicked", self.on_console_bar_undock_clicked)

        # Connect Shortcut buttons' signals to their corresponding methods
        self.connect_button_to_function('button_start_shortcut', "toggled", self.on_button_start_shortcut_toggled)
        self.connect_button_to_function('button_stop_shortcut', "clicked", self.on_button_stop_shortcut_clicked)
        self.connect_button_to_function('button_pause_shortcut', "toggled", self.on_button_pause_shortcut_toggled)
        self.connect_button_to_function('button_step_mode_shortcut',
                                        "toggled",
                                        self.on_button_step_mode_shortcut_toggled)
        self.connect_button_to_function('button_step_in_shortcut',
                                        "clicked",
                                        self.on_button_step_in_shortcut_clicked)
        self.connect_button_to_function('button_step_over_shortcut',
                                        "clicked",
                                        self.on_button_step_over_shortcut_clicked)
        self.connect_button_to_function('button_step_out_shortcut',
                                        "clicked",
                                        self.on_button_step_out_shortcut_clicked)
        self.connect_button_to_function('button_step_backward_shortcut',
                                        "clicked",
                                        self.on_button_step_backward_shortcut_clicked)

        # Connect Debug console buttons' signals to their corresponding methods
        self.connect_button_to_function('button_show_debug', "toggled", self.on_debug_content_change)
        self.connect_button_to_function('button_show_info', "toggled", self.on_debug_content_change)
        self.connect_button_to_function('button_show_warning', "toggled", self.on_debug_content_change)
        self.connect_button_to_function('button_show_error', "toggled", self.on_debug_content_change)

        view['upper_notebook'].connect('switch-page', self.on_notebook_tab_switch, view['upper_notebook_title'],
                                       view.left_bar_window, 'upper')
        view['lower_notebook'].connect('switch-page', self.on_notebook_tab_switch, view['lower_notebook_title'],
                                       view.left_bar_window, 'lower')

        # hide not usable buttons
        self.view['step_buttons'].hide()

        # Initializing Main Window Size & Position
        gui_helper.set_window_size_and_position(view.get_top_widget(), 'MAIN_WINDOW')

        # Initializing Pane positions
        for config_id in constants.PANE_ID.keys():
            self.set_pane_position(config_id)
            # view[constants.PANE_ID[config_id]].get_child2().connect('size-allocate', self.print_paned_pos, config_id)

        if gui_config.get_config_value('AUTO_BACKUP_ENABLED') and gui_config.get_config_value('AUTO_RECOVERY_CHECK'):
            import rafcon.mvc.models.auto_backup as auto_backup
            auto_backup.check_for_crashed_rafcon_instances()

    def print_paned_pos(self, width, height, config_id):
        pane_id = constants.PANE_ID[config_id]
        view = self.view[pane_id]
        print "paned position {4} '{1}' is now {0}.{2}|{3}".format(view.get_position(), pane_id,
                                                                   width, height, config_id)
        # TODO scale panes proportional if window is re-sized
        # TODO right-bar-pane hold position if left-bar-pane moves or becomes hide

    def connect_button_to_function(self, view_index, button_state, function):
        handler_id = self.view[view_index].connect(button_state, function)
        self.handler_ids[view_index] = handler_id

    def switch_state_machine_execution_engine(self, new_state_machine_execution_engine):
        """
        Switch the state machine execution engine the main window controller listens to.
        :param new_state_machine_execution_engine: the new state machine execution engine for this controller
        :return:
        """
        # relieve old one
        self.relieve_model(self.state_machine_execution_model)

        # register new
        self.state_machine_execution_model = new_state_machine_execution_engine
        self.observe_model(self.state_machine_execution_model)

    def set_pane_position(self, config_id):
        """Adjusts the position of a GTK Pane to a value stored in the runtime config file. If there was no value
        stored, the pane's position is set to a default value.

        :param config_id: The pane identifier saved in the runtime config file
        """
        default_pos = constants.DEFAULT_PANE_POS[config_id]
        position = global_runtime_config.get_config_value(config_id, default_pos)
        pane_id = constants.PANE_ID[config_id]
        # prev_pos = {}
        # for pane_id_ in ['top_level_h_pane', 'right_h_pane', 'central_v_pane', 'left_bar_pane']:
        #     prev_pos[pane_id_] = self.view[pane_id_].get_position()
        self.view[pane_id].set_position(position)
        # logger.info("try to set {0}, {1}, {2}, default: {3}, now on {4}"
        #             "".format(pane_id, position, config_id, default_pos, self.view[pane_id].get_position()))
        # for pane_id_ in ['top_level_h_pane', 'right_h_pane', 'central_v_pane', 'left_bar_pane']:
        #     logger.info("after {0} position: {1} == {2}".format(pane_id_, self.view[pane_id_].get_position(),
        #                                                         prev_pos[pane_id_]))

    def highlight_execution_of_current_sm(self, active):
        if self.get_controller('state_machines_editor_ctrl') is None or \
                self.get_controller('state_machines_editor_ctrl').view is None:
            logger.debug("No state machines editor view")
            return
        notebook = self.get_controller('state_machines_editor_ctrl').view['notebook']
        page_num = self.get_controller('state_machines_editor_ctrl').view['notebook'].get_current_page()
        page = self.get_controller('state_machines_editor_ctrl').view['notebook'].get_nth_page(page_num)
        if page is None:
            logger.warning("No state machine open {0}".format(page_num))
            return
        label = notebook.get_tab_label(page).get_children()[0]
        if active:
            label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse(gui_config.colors['STATE_MACHINE_ACTIVE']))
            label.modify_fg(gtk.STATE_INSENSITIVE, gtk.gdk.color_parse(gui_config.colors['STATE_MACHINE_ACTIVE']))
        else:
            label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse(gui_config.colors['STATE_MACHINE_NOT_ACTIVE']))
            label.modify_fg(gtk.STATE_INSENSITIVE, gtk.gdk.color_parse(gui_config.colors['STATE_MACHINE_NOT_ACTIVE']))

    @ExtendedController.observe("execution_engine", after=True)
    def model_changed(self, model, prop_name, info):
        """ Highlight buttons according actual execution status."""
        label_string = str(rafcon.statemachine.singleton.state_machine_execution_engine.status.execution_mode)
        label_string = label_string.replace("STATE_MACHINE_EXECUTION_STATUS.", "")
        self.view['execution_status_label'].set_text(label_string)

        if rafcon.statemachine.singleton.state_machine_execution_engine.status.execution_mode is StateMachineExecutionStatus.STARTED:
            self.highlight_execution_of_current_sm(True)
            self.view['step_buttons'].hide()
            self._set_single_button_active('button_start_shortcut')
        elif rafcon.statemachine.singleton.state_machine_execution_engine.status.execution_mode is StateMachineExecutionStatus.PAUSED:
            self.highlight_execution_of_current_sm(True)
            self.view['step_buttons'].hide()
            self._set_single_button_active('button_pause_shortcut')
        elif rafcon.statemachine.singleton.state_machine_execution_engine.status.execution_mode is StateMachineExecutionStatus.STOPPED:
            self.highlight_execution_of_current_sm(False)
            self.view['step_buttons'].hide()
            self._set_single_button_active('button_stop_shortcut')
        else:  # all step modes
            self.highlight_execution_of_current_sm(True)
            self.view['step_buttons'].show()
            self._set_single_button_active('button_step_mode_shortcut')

    def _set_single_button_active(self, active_button_name):

        # do not let the buttons trigger the action another time => block the respective signal handlers

        button_names = ['button_start_shortcut', 'button_pause_shortcut', 'button_step_mode_shortcut']

        for button_name in button_names:
            if active_button_name == button_name:
                if not self.view[button_name].get_active():
                    # block the handler before setting the button active
                    self.view[button_name].handler_block(self.handler_ids[button_name])
                    self.view[button_name].set_active(True)
                    self.view[button_name].handler_unblock(self.handler_ids[button_name])
            else:
                if self.view[button_name].get_active():
                    self.view[button_name].handler_block(self.handler_ids[button_name])
                    self.view[button_name].set_active(False)
                    self.view[button_name].handler_unblock(self.handler_ids[button_name])

    def focus_notebook_page_of_controller(self, controller):
        """Puts the focus on the given child controller

        The method implements focus request of the notebooks in left side-bar of the main window. Thereby it is the
        master-function of focus pattern of the notebooks in left side-bar.

        Actual pattern is:
        * Execution-History is put to focus any time requested (request occur at the moment when the state-machine
        is started and stopped.
        * Modification-History one time focused while and one time after execution if requested.

        :param controller The controller which request to be focused.
        """
        # TODO think about to may substitute Controller- by View-objects it is may the better design
        if controller not in self.get_child_controllers():
            return
        # logger.info("focus controller {0}".format(controller))
        if not self.modification_history_was_focused and isinstance(controller, ModificationHistoryTreeController) and \
                self.view is not None:
            self.view.bring_tab_to_the_top('history')
            self.modification_history_was_focused = True

        if self.view is not None and isinstance(controller, ExecutionHistoryTreeController):
            self.view.bring_tab_to_the_top('execution_history')
            self.modification_history_was_focused = False

    def on_left_bar_return_clicked(self, widget, event=None):
        self.view['left_bar_return_button'].hide()
        self.view['top_level_h_pane'].pack1(self.left_bar_child, resize=True, shrink=False)

    def on_right_bar_return_clicked(self, widget, event=None):
        self.view['right_bar_return_button'].hide()
        self.view['right_h_pane'].pack2(self.right_bar_child, resize=True, shrink=False)

    def on_console_return_clicked(self, widget, event=None):
        self.view['console_return_button'].hide()
        self.view['central_v_pane'].pack2(self.console_child, resize=True, shrink=False)

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
        """Triggered when the un-dock button of the left bar is clicked.

        The left bar is un-docked into a separate new window, and the bar is hidden from the main-window by triggering
        the method on_left_bar_hide_clicked(). triggering this method shows the 'left_bar_return_button' in the
        main-window, which doesn't serve any purpose when the bar is un-docked. This button is therefore deliberately
        hidden. The un-dock button, which is also part of the bar is hidden, because the re-dock button is included in
        the top_tool_bar of the newly opened window. Not hiding it will result in two re-dock buttons visible in the new
        window. The new window's size and position are loaded from runtime_config, if they exist.
        """
        gui_helper.set_window_size_and_position(self.view.left_bar_window.get_top_widget(), 'LEFT_BAR_WINDOW')
        self.view['left_bar_pane'].reparent(self.view.left_bar_window['central_eventbox'])
        self.view['undock_left_bar_button'].hide()
        self.on_left_bar_hide_clicked(None)
        self.view['left_bar_return_button'].hide()

    def on_left_bar_dock_clicked(self, widget, event=None):
        """Triggered when the re-dock button of the left-bar window is clicked.

        The size & position of the open window are saved to the runtime_config file, and the left-bar is re-docked back
        to the main-window, and the left-bar window is hidden. The un-dock button of the bar is made visible again.
        """
        global_runtime_config.store_widget_properties(self.view.left_bar_window.get_top_widget(), 'LEFT_BAR_WINDOW')
        self.on_left_bar_return_clicked(None)
        self.view['left_bar_pane'].reparent(self.view['left_sidebar_viewport'])
        self.get_controller('left_window_controller').hide_window()
        self.view['undock_left_bar_button'].show()
        return True

    def on_right_bar_undock_clicked(self, widget, event=None):
        """Triggered when the un-dock button of the right bar is clicked.

        The right bar is un-docked into a separate new window, and the bar is hidden from the main-window by triggering
        the method on_right_bar_hide_clicked(). triggering this method shows the 'right_bar_return_button' in the
        main-window, which doesn't serve any purpose when the bar is un-docked. This button is therefore deliberately
        hidden. The un-dock button, which is also part of the bar, is hidden, because the re-dock button is included in
        the top_tool_bar of the newly opened window. Not hiding it will result in two re-dock buttons visible in the new
        window. The new window's size and position are loaded from runtime_config, if they exist.
        """
        gui_helper.set_window_size_and_position(self.view.right_bar_window.get_top_widget(), 'RIGHT_BAR_WINDOW')
        self.view['right_bar'].reparent(self.view.right_bar_window['central_eventbox'])
        self.view['undock_right_bar_button'].hide()
        self.on_right_bar_hide_clicked(None)
        self.view['right_bar_return_button'].hide()

    def on_right_bar_dock_clicked(self, widget, event=None):
        """Triggered when the re-dock button of the right-bar window is clicked.

        The size & position of the open window is saved to the runtime_config file, and the right-bar is re-docked back
        to the main-window, and the right-bar window is hidden. The un-dock button of the bar is made visible again.
        """
        global_runtime_config.store_widget_properties(self.view.right_bar_window.get_top_widget(), 'RIGHT_BAR_WINDOW')
        self.on_right_bar_return_clicked(None)
        self.view['right_bar'].reparent(self.view['right_bar_container'])
        self.get_controller('right_window_controller').hide_window()
        self.docked['right_bar'] = True
        self.view['undock_right_bar_button'].show()

    def on_console_bar_undock_clicked(self, widget, event=None):
        """Triggered when the un-dock button of the console is clicked.

        The console is un-docked into a separate new window, and the console is hidden from the main-window by
        triggering the method on_console_hide_clicked(). triggering this method shows the 'console_return_button' in the
        main-window, which doesn't serve any purpose when the bar is un-docked. This button is therefore deliberately
        hidden. The un-dock button, which is also part of the console, is hidden, because the re-dock button is included
        in the top_tool_bar of the newly opened window. Not hiding it will result in two re-dock buttons visible in the
        new window. The new window's size and position are loaded from runtime_config, if they exist.
        """
        gui_helper.set_window_size_and_position(self.view.console_bar_window.get_top_widget(), 'CONSOLE_BAR_WINDOW')
        self.view['console'].reparent(self.view.console_bar_window['central_eventbox'])
        self.view['undock_console_button'].hide()
        self.on_console_hide_clicked(None)
        self.view['console_return_button'].hide()

    def on_console_bar_dock_clicked(self, widget, event=None):
        """Triggered when the re-dock button of the console window is clicked.

        The size & position of the open window is saved to the runtime_config file, and the console is re-docked back
        to the main-window, and the console window is hidden. The un-dock button of the bar is made visible again.
        """
        global_runtime_config.store_widget_properties(self.view.console_bar_window.get_top_widget(), 'CONSOLE_BAR_WINDOW')
        self.on_console_return_clicked(None)
        self.view['console'].reparent(self.view['console_container'])
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

    def on_button_step_over_shortcut_clicked(self, widget, event=None):
        self.get_controller('menu_bar_controller').on_step_over_activate(None)

    def on_button_step_out_shortcut_clicked(self, widget, event=None):
        self.get_controller('menu_bar_controller').on_step_out_activate(None)

    def on_button_step_backward_shortcut_clicked(self, widget, event=None):
        self.get_controller('menu_bar_controller').on_backward_step_activate(None)

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
