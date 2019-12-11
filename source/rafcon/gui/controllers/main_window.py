# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Benno Voggenreiter <benno.voggenreiter@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: main_window
   :synopsis: The module holds the main window controller giving footage to the overall gui.

"""

from builtins import str
import os
import logging
from gi.repository import Gtk
from gi.repository import Gdk
from functools import partial

import rafcon.core.config
import rafcon.core.singleton
import rafcon.gui.singleton as gui_singletons
from rafcon.core.execution.execution_status import StateMachineExecutionStatus
from rafcon.gui.config import global_gui_config as gui_config
from rafcon.gui.controllers.execution_history import ExecutionHistoryTreeController
from rafcon.gui.controllers.global_variable_manager import GlobalVariableManagerController
from rafcon.gui.controllers.library_tree import LibraryTreeController
from rafcon.gui.controllers.menu_bar import MenuBarController
from rafcon.gui.controllers.modification_history import ModificationHistoryTreeController
from rafcon.gui.controllers.notification_bar import NotificationBarController
from rafcon.gui.controllers.debug_console import DebugConsoleController
from rafcon.gui.controllers.state_icons import StateIconController
from rafcon.gui.controllers.state_machine_tree import StateMachineTreeController
from rafcon.gui.controllers.state_machines_editor import StateMachinesEditorController
from rafcon.gui.controllers.states_editor import StatesEditorController
from rafcon.gui.controllers.tool_bar import ToolBarController
from rafcon.gui.controllers.execution_ticker import ExecutionTickerController
from rafcon.gui.controllers.undocked_window import UndockedWindowController
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.gui.views.main_window import MainWindowView
import rafcon.gui.helpers.label as gui_helper_label
from rafcon.gui.models.state_machine_manager import StateMachineManagerModel
from rafcon.gui.runtime_config import global_runtime_config
from rafcon.gui.shortcut_manager import ShortcutManager
from rafcon.gui.utils import constants, wait_for_gui
from rafcon.utils import log, log_helpers
from rafcon.utils import plugins

logger = log.get_logger(__name__)


class MainWindowController(ExtendedController):
    """Controller handling the main window.

    :param rafcon.gui.models.state_machine_manager.StateMachineManagerModel state_machine_manager_model: The state
        machine manager model, holding data regarding state machines. Should be exchangeable.
    :param rafcon.gui.views.main_window.MainWindowView view: The GTK View showing the main window.
    :ivar dict hidden: Dictionary that hold's mapping between bars/console and their current hidden-status.
    """

    def __init__(self, state_machine_manager_model, view):
        assert isinstance(state_machine_manager_model, StateMachineManagerModel)
        assert isinstance(view, MainWindowView)
        ExtendedController.__init__(self, state_machine_manager_model, view)

        gui_singletons.main_window_controller = self
        self.observe_model(gui_singletons.gui_config_model)

        self.shortcut_manager = None
        self.handler_ids = {}
        self.currently_pressed_keys = set()

        self.state_machine_execution_model = gui_singletons.state_machine_execution_model
        self.observe_model(self.state_machine_execution_model)

        # shortcut manager
        self.shortcut_manager = ShortcutManager(view['main_window'])

        ######################################################
        # debug console
        ######################################################
        debug_console_controller = DebugConsoleController(gui_singletons.gui_config_model,
                                                          view.debug_console_view)
        self.add_controller('debug_console_controller', debug_console_controller)

        ######################################################
        # library tree
        ######################################################
        self.library_manager_model = gui_singletons.library_manager_model
        library_controller = LibraryTreeController(self.library_manager_model, view.library_tree)
        self.add_controller('library_controller', library_controller)

        ######################################################
        # state icons
        ######################################################
        state_icon_controller = StateIconController(state_machine_manager_model, view.state_icons,
                                                    self.shortcut_manager)
        self.add_controller('state_icon_controller', state_icon_controller)

        ######################################################
        # state machine tree
        ######################################################
        state_machine_tree_controller = StateMachineTreeController(state_machine_manager_model, view.state_machine_tree)
        self.add_controller('state_machine_tree_controller', state_machine_tree_controller)

        ######################################################
        # notification bar
        ######################################################
        self.notification_bar_ctrl = NotificationBarController(gui_singletons.gui_config_model, view.notification_bar)
        self.add_controller('notification_bar_ctrl', self.notification_bar_ctrl)

        ######################################################
        # states editor
        ######################################################
        states_editor_ctrl = StatesEditorController(state_machine_manager_model, view.states_editor)
        self.add_controller('states_editor_ctrl', states_editor_ctrl)

        ######################################################
        # state machines editor
        ######################################################
        self.state_machines_editor_ctrl = StateMachinesEditorController(state_machine_manager_model,
                                                                        view.state_machines_editor)
        self.add_controller('state_machines_editor_ctrl', self.state_machines_editor_ctrl)

        ######################################################
        # global variable editor
        ######################################################
        global_variable_manager_ctrl = GlobalVariableManagerController(gui_singletons.global_variable_manager_model,
                                                                       view.global_var_editor)
        self.add_controller('global_variable_manager_ctrl', global_variable_manager_ctrl)

        ######################################################
        # modification history
        ######################################################
        state_machine_history_controller = ModificationHistoryTreeController(state_machine_manager_model,
                                                                             view.state_machine_history)
        self.add_controller('state_machine_history_controller', state_machine_history_controller)
        self.modification_history_was_focused = False

        ######################################################
        # state machine execution history
        ######################################################
        execution_history_ctrl = ExecutionHistoryTreeController(state_machine_manager_model, view.execution_history)
        self.add_controller('execution_history_ctrl', execution_history_ctrl)

        ######################################################
        # execution ticker
        ######################################################
        self.execution_ticker_ctrl = ExecutionTickerController(self.state_machine_execution_model, None)
        self.add_controller('execution_ticker_ctrl', self.execution_ticker_ctrl)

        ######################################################
        # menu bar
        ######################################################
        menu_bar_controller = MenuBarController(state_machine_manager_model, view, self.shortcut_manager,
                                                rafcon.core.singleton.state_machine_execution_engine)
        self.add_controller('menu_bar_controller', menu_bar_controller)

        ######################################################
        # tool bar
        ######################################################
        tool_bar_controller = ToolBarController(state_machine_manager_model, view.tool_bar)
        self.add_controller('tool_bar_controller', tool_bar_controller)

        ######################################################
        # Undocked Windows Controllers
        ######################################################
        for window_key in constants.UNDOCKABLE_WINDOW_KEYS:
            widget_name = window_key.lower() + "_container"
            window_ctrl_name = window_key.lower() + "_window_controller"
            undocked_window_view = getattr(view, window_key.lower() + "_window")
            redock_callback = partial(self.redock_sidebar, window_key, widget_name, window_ctrl_name)
            window_ctrl = UndockedWindowController(state_machine_manager_model, undocked_window_view, redock_callback)
            self.add_controller(window_ctrl_name, window_ctrl)

        # Initialize the Left-Bar Notebooks' titles according to initially-selected tabs
        upper_title = gui_helper_label.set_notebook_title(view['upper_notebook'],
                                                          view['upper_notebook'].get_current_page(),
                                                          view['upper_notebook_title'])
        lower_title = gui_helper_label.set_notebook_title(view['lower_notebook'],
                                                          view['lower_notebook'].get_current_page(),
                                                          view['lower_notebook_title'])

        # Initialize the Left-Bar un-docked window title
        view.left_bar_window.initialize_title(gui_helper_label.create_left_bar_window_title(upper_title, lower_title))
        view.right_bar_window.initialize_title('STATE EDITOR')
        view.console_window.initialize_title('CONSOLE')

        self.left_bar_child = view['top_level_h_pane'].get_child1()
        self.right_bar_child = view['right_h_pane'].get_child2()
        self.console_child = view['central_v_pane'].get_child2()

        self.left_bar_hidden = False
        self.right_bar_hidden = False
        self.console_hidden = False

    def destroy(self):
        if hasattr(self, '_max_position_notification_id'):
            last_pane_id = next(reversed(constants.PANE_ID.values()))
            self.view[last_pane_id].disconnect(self._max_position_notification_id)

        super(MainWindowController, self).destroy()
        # The sidebars have no corresponding controller that could destroy the views what cause the connected methods
        # to stay connected to (hold references on) the main window controller. So, we do this here. TODO D-solve it
        self.shortcut_manager.destroy()
        self.left_bar_child.destroy()
        self.right_bar_child.destroy()
        self.console_child.destroy()

    @staticmethod
    def update_widget_runtime_config(widget, event, name):
        global_runtime_config.store_widget_properties(widget, name)

    def register_view(self, view):
        super(MainWindowController, self).register_view(view)
        self.register_actions(self.shortcut_manager)

        self.view.get_top_widget().connect("key-press-event", self._on_key_press)
        self.view.get_top_widget().connect("key-release-event", self._on_key_release)

        # using helper function to connect functions to GUI elements to be able to access the handler id later on

        self.connect_button_to_function('main_window',
                                        "delete_event",
                                        self.get_controller('menu_bar_controller').on_quit_activate)

        # connect left bar, right bar and console hide buttons' signals to their corresponding methods
        self.connect_button_to_function('left_bar_hide_button', "clicked", self.on_left_bar_hide_clicked)
        self.connect_button_to_function('right_bar_hide_button', "clicked", self.on_right_bar_hide_clicked)
        self.connect_button_to_function('console_hide_button', "clicked", self.on_console_hide_clicked)

        # Connect left bar, right bar and console return buttons' signals to their corresponding methods
        self.connect_button_to_function('left_bar_return_button', "clicked", self.on_left_bar_return_clicked)
        self.connect_button_to_function('right_bar_return_button', "clicked", self.on_right_bar_return_clicked)
        self.connect_button_to_function('console_return_button', "clicked", self.on_console_return_clicked)

        # Connect undock buttons signals
        for window_key in constants.UNDOCKABLE_WINDOW_KEYS:
            self.connect_button_to_function('undock_{}_button'.format(window_key.lower()), "clicked",
                                            partial(self.undock_sidebar, window_key))

        # Connect collapse button for trees
        self.connect_button_to_function('collapse_tree_button', "clicked", self.on_collapse_button_clicked)

        # Connect Shortcut buttons' signals to their corresponding methods
        self.connect_button_to_function('button_start_shortcut', "toggled", self.on_button_start_shortcut_toggled)
        self.connect_button_to_function('button_stop_shortcut', "clicked", self.on_button_stop_shortcut_clicked)
        self.connect_button_to_function('button_pause_shortcut', "toggled", self.on_button_pause_shortcut_toggled)
        self.connect_button_to_function('button_start_from_shortcut', "clicked",
                                        self.on_button_start_from_shortcut_clicked)
        self.connect_button_to_function('button_run_to_shortcut', "clicked",
                                        self.on_button_run_to_shortcut_clicked)
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

        view['upper_notebook'].connect('switch-page', self.on_notebook_tab_switch, view['upper_notebook_title'],
                                       view.left_bar_window, 'upper')
        view['lower_notebook'].connect('switch-page', self.on_notebook_tab_switch, view['lower_notebook_title'],
                                       view.left_bar_window, 'lower')

        view.get_top_widget().connect("configure-event", self.update_widget_runtime_config, "MAIN_WINDOW")
        view.left_bar_window.get_top_widget().connect("configure-event", self.update_widget_runtime_config, "LEFT_BAR_WINDOW")
        view.right_bar_window.get_top_widget().connect("configure-event", self.update_widget_runtime_config, "RIGHT_BAR_WINDOW")
        view.console_window.get_top_widget().connect("configure-event", self.update_widget_runtime_config, "CONSOLE_WINDOW")

        # save pane positions in the runtime config on every change
        view['top_level_h_pane'].connect("button-release-event", self.update_widget_runtime_config, "LEFT_BAR_DOCKED")
        view['right_h_pane'].connect("button-release-event", self.update_widget_runtime_config, "RIGHT_BAR_DOCKED")
        view['central_v_pane'].connect("button-release-event", self.update_widget_runtime_config, "CONSOLE_DOCKED")

        # hide not usable buttons
        self.view['step_buttons'].hide()


        # Initializing Main Window Size & Position
        # secure un maximize in initial condition to restore correct position and size
        view.get_top_widget().unmaximize()
        gui_helper_label.set_window_size_and_position(view.get_top_widget(), 'MAIN')

        wait_for_gui()

        # set the hidden status of all bars
        for window_key in constants.UNDOCKABLE_WINDOW_KEYS:
            if global_runtime_config.get_config_value(window_key + '_HIDDEN'):
                func = getattr(self, 'on_{}_hide_clicked'.format(window_key.lower()))
                func(None)

        # restore undock state of bar windows
        if gui_config.get_config_value("RESTORE_UNDOCKED_SIDEBARS"):
            for window_key in constants.UNDOCKABLE_WINDOW_KEYS:
                if global_runtime_config.get_config_value(window_key + "_WINDOW_UNDOCKED"):
                    self.undock_sidebar(window_key)

        # secure maximized state
        if global_runtime_config.get_config_value("MAIN_WINDOW_MAXIMIZED"):
            wait_for_gui()
            view.get_top_widget().maximize()

        # Restore position of Gtk.Paned widgets
        first_pane_id = next(iter(constants.PANE_ID.values()))
        last_pane_config, last_pane_id = next(reversed(constants.PANE_ID.items()))

        def last_pane_property_changed(widget, property):
            if property.name == "max-position":
                self.set_pane_position(last_pane_config, max_value=widget.props.max_position)

        def deferred_pane_positioning(*args):
            self.view[first_pane_id].disconnect_by_func(deferred_pane_positioning)
            for config_id in constants.PANE_ID:
                self.set_pane_position(config_id)
            # position of last pane needs to be set again if its max-position property changes
            self._max_position_notification_id = self.view[last_pane_id].connect("notify", last_pane_property_changed)

        # Set positions after all panes have been drawn once
        self.view[first_pane_id].connect_after("draw", deferred_pane_positioning)

        plugins.run_hook("main_window_setup", self)

        # check for auto backups
        if gui_config.get_config_value('AUTO_BACKUP_ENABLED') and gui_config.get_config_value('AUTO_RECOVERY_CHECK'):
            import rafcon.gui.models.auto_backup as auto_backup
            auto_backup.check_for_crashed_rafcon_instances()

        wait_for_gui()
        # Ensure that the next message is being printed (needed for LN manager to detect finished startup)
        level = logger.level
        logger.setLevel(logging.INFO)
        logger.info("Ready")
        logger.setLevel(level)

    def connect_button_to_function(self, view_index, button_state, function, *args):
        handler_id = self.view[view_index].connect(button_state, function, *args)
        self.handler_ids[view_index] = handler_id

    def switch_state_machine_execution_engine(self, new_state_machine_execution_engine):
        """ Switch the state machine execution engine.

        :param new_state_machine_execution_engine: the new state machine execution engine for this controller
        :return:
        """
        # relieve old one
        self.relieve_model(self.state_machine_execution_model)

        # register new
        self.state_machine_execution_model = new_state_machine_execution_engine
        self.observe_model(self.state_machine_execution_model)

        # inform observing controllers
        self.state_machines_editor_ctrl.switch_state_machine_execution_engine(new_state_machine_execution_engine)

    def set_pane_position(self, config_id, max_value=None):
        """Adjusts the position of a GTK Pane to a value stored in the runtime config file. If there was no value
        stored, the pane's position is set to a default value.

        :param config_id: The pane identifier saved in the runtime config file
        """
        default_pos = constants.DEFAULT_PANE_POS[config_id]
        position = global_runtime_config.get_config_value(config_id, default_pos)
        if max_value:
            position = min(position, max_value)
        pane_id = constants.PANE_ID[config_id]
        self.view[pane_id].set_position(position)

    @ExtendedController.observe("execution_engine", after=True)
    def model_changed(self, model, prop_name, info):
        """ Highlight buttons according actual execution status. Furthermore it triggers the label redraw of the active
        state machine.
        """

        # TODO: find nice solution
        # this in only required if the GUI is terminated via Ctrl+C signal
        if not self.view:
            # this means that the main window is currently under destruction
            return

        execution_engine = rafcon.core.singleton.state_machine_execution_engine
        label_string = str(execution_engine.status.execution_mode)
        label_string = label_string.replace("STATE_MACHINE_EXECUTION_STATUS.", "")
        self.view['execution_status_label'].set_text(label_string)

        current_execution_mode = execution_engine.status.execution_mode
        if current_execution_mode is StateMachineExecutionStatus.STARTED:
            self.view['step_buttons'].hide()
            self._set_single_button_active('button_start_shortcut')
        elif current_execution_mode is StateMachineExecutionStatus.PAUSED:
            self.view['step_buttons'].hide()
            self._set_single_button_active('button_pause_shortcut')
        elif execution_engine.finished_or_stopped():
            self.view['step_buttons'].hide()
            self._set_single_button_active('button_stop_shortcut')
        else:  # all step modes
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
        self.left_bar_hidden = False

    def on_right_bar_return_clicked(self, widget, event=None):
        self.view['right_bar_return_button'].hide()
        self.view['right_h_pane'].pack2(self.right_bar_child, resize=False, shrink=False)
        self.right_bar_hidden = False

    def on_console_return_clicked(self, widget, event=None):
        self.view['console_return_button'].hide()
        self.view['central_v_pane'].pack2(self.console_child, resize=True, shrink=False)
        self.console_hidden = False

    def on_left_bar_hide_clicked(self, widget, event=None):
        self.view['top_level_h_pane'].remove(self.left_bar_child)
        self.view['left_bar_return_button'].show()
        self.left_bar_hidden = True

    def on_right_bar_hide_clicked(self, widget, event=None):
        self.view['right_h_pane'].remove(self.right_bar_child)
        self.view['right_bar_return_button'].show()
        self.right_bar_hidden = True

    def on_console_hide_clicked(self, widget, event=None):
        self.view['central_v_pane'].remove(self.console_child)
        self.view['console_return_button'].show()
        self.console_hidden = True

    def undock_window_callback(self, widget, event, undocked_window):
        if event.new_window_state & Gdk.WindowState.WITHDRAWN or event.new_window_state & Gdk.WindowState.ICONIFIED:
            undocked_window.iconify()
        else:
            undocked_window.deiconify()

    def undock_sidebar(self, window_key, widget=None, event=None):
        """Undock/separate sidebar into independent window

        The sidebar is undocked and put into a separate new window. The sidebar is hidden in the main-window by
        triggering the method on_[widget_name]_hide_clicked(). Triggering this method shows the
        [widget_name]_return_button in the main-window, which does not serve any purpose when the bar is undocked.
        This button is therefore deliberately
        hidden. The undock button, which is also part of the sidebar is hidden, because the re-dock button is
        included in the top_tool_bar of the newly opened window. Not hiding it will result in two re-dock buttons
        visible in the new window. The new window size and position are loaded from runtime_config, if they exist.
        """
        undocked_window_name = window_key.lower() + '_window'
        widget_name = window_key.lower()
        undocked_window_view = getattr(self.view, undocked_window_name)
        undocked_window = undocked_window_view.get_top_widget()
        if os.getenv("RAFCON_START_MINIMIZED", False):
            undocked_window.iconify()

        gui_helper_label.set_window_size_and_position(undocked_window, window_key)

        self.view[widget_name].get_parent().remove(self.view[widget_name])
        undocked_window_view['central_eventbox'].add(self.view[widget_name])
        self.view['undock_{}_button'.format(widget_name)].hide()
        getattr(self, 'on_{}_hide_clicked'.format(widget_name))(None)
        self.view['{}_return_button'.format(widget_name)].hide()

        main_window = self.view.get_top_widget()
        state_handler = main_window.connect('window-state-event', self.undock_window_callback, undocked_window)
        self.handler_ids[undocked_window_name] = {"state": state_handler}
        undocked_window.set_transient_for(main_window)
        main_window.grab_focus()
        global_runtime_config.set_config_value(window_key + '_WINDOW_UNDOCKED', True)

    def redock_sidebar(self, window_key, sidebar_name, controller_name, widget, event=None):
        """Redock/embed sidebar into main window

        The size & position of the open window are saved to the runtime_config file, the sidebar is redocked back
        to the main-window, and the left-bar window is hidden. The undock button of the bar is made visible again.
        """
        config_parameter_undocked = window_key + '_WINDOW_UNDOCKED'
        config_id_for_pane_position = window_key + '_DOCKED_POS'
        undocked_window_name = window_key.lower() + '_window'
        widget_name = window_key.lower()

        self.view['main_window'].disconnect(self.handler_ids[undocked_window_name]['state'])
        getattr(self, 'on_{}_return_clicked'.format(widget_name))(None)

        self.view[widget_name].get_parent().remove(self.view[widget_name])
        self.view[sidebar_name].pack_start(self.view[widget_name], True, True, 0)

        self.get_controller(controller_name).hide_window()
        self.view['undock_{}_button'.format(widget_name)].show()

        # restore the position of the pane
        self.set_pane_position(config_id_for_pane_position)

        global_runtime_config.set_config_value(config_parameter_undocked, False)
        return True

    def toggle_sidebars(self):
        # If any sidebar is shown, hide both
        if not self.left_bar_hidden or not self.right_bar_hidden:
            if not self.left_bar_hidden:
                self.on_left_bar_hide_clicked(None)
            if not self.right_bar_hidden:
                self.on_right_bar_hide_clicked(None)
        else:
            if self.left_bar_hidden:
                self.on_left_bar_return_clicked(None)
            if self.right_bar_hidden:
                self.on_right_bar_return_clicked(None)

    # Shortcut buttons
    def on_button_start_shortcut_toggled(self, widget, event=None):
        if self.view['button_start_shortcut'].get_active():
            self.get_controller('menu_bar_controller').on_start_activate(None)

    def on_button_stop_shortcut_clicked(self, widget, event=None):
        self.get_controller('menu_bar_controller').on_stop_activate(None)

    def on_button_pause_shortcut_toggled(self, widget, event=None):
        if self.view['button_pause_shortcut'].get_active():
            self.get_controller('menu_bar_controller').on_pause_activate(None)

    def on_button_start_from_shortcut_clicked(self, widget, event=None):
        self.get_controller('menu_bar_controller').on_start_from_selected_state_activate(None)

    def on_button_run_to_shortcut_clicked(self, widget, event=None):
        self.get_controller('menu_bar_controller').on_run_to_selected_state_activate(None)

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

    def on_notebook_tab_switch(self, notebook, page, page_num, title_label, window, notebook_identifier):
        """Triggered whenever a left-bar notebook tab is changed.

        Updates the title of the corresponding notebook and updates the title of the left-bar window in case un-docked.

        :param notebook: The GTK notebook where a tab-change occurred
        :param page_num: The page number of the currently-selected tab
        :param title_label: The label holding the notebook's title
        :param window: The left-bar window, for which the title should be changed
        :param notebook_identifier: A string identifying whether the notebook is the upper or the lower one
        """
        title = gui_helper_label.set_notebook_title(notebook, page_num, title_label)
        window.reset_title(title, notebook_identifier)
        self.on_switch_page_check_collapse_button(notebook, page_num)

    def on_switch_page_check_collapse_button(self, notebook, page_num):
        upper_page_num = self.view['upper_notebook'].get_current_page() if notebook is not self.view['upper_notebook'] else page_num
        upper_notebook_title = gui_helper_label.get_notebook_tab_title(self.view['upper_notebook'], upper_page_num)
        lower_page_num = self.view['lower_notebook'].get_current_page() if notebook is not self.view['lower_notebook'] else page_num
        lower_notebook_title = gui_helper_label.get_notebook_tab_title(self.view['lower_notebook'], lower_page_num)
        if any([title in upper_notebook_title for title in ['LIBRARIES', "STATES TREE"]]) or \
                any([title in lower_notebook_title for title in ['LIBRARIES', "STATES TREE"]]):
            self.view["collapse_tree_button"].show()
        else:
            self.view["collapse_tree_button"].hide()

    def on_collapse_button_clicked(self, button):
        upper_page_num = self.view['upper_notebook'].get_current_page()
        upper_notebook_title = gui_helper_label.get_notebook_tab_title(self.view['upper_notebook'], upper_page_num)
        lower_page_num = self.view['lower_notebook'].get_current_page()
        lower_notebook_title = gui_helper_label.get_notebook_tab_title(self.view['lower_notebook'], lower_page_num)
        if any(['LIBRARIES' in title for title in [upper_notebook_title, lower_notebook_title]]):
            self.get_controller('library_controller').view.collapse_all()
        if any(["STATES TREE" in title for title in [upper_notebook_title, lower_notebook_title]]):
            self.get_controller('state_machine_tree_controller').view.collapse_all()
            
    def _on_key_press(self, widget, event):
        """Updates the currently pressed keys

        :param Gtk.Widget widget: The main window
        :param Gdk.Event event: The key press event
        """
        self.currently_pressed_keys.add(event.keyval)

    def _on_key_release(self, widget, event):
        """Updates the currently pressed keys

        :param Gtk.Widget widget: The main window
        :param Gdk.Event event: The key release event
        """
        self.currently_pressed_keys.discard(event.keyval)

    def _on_key_press(self, widget, event):
        """Updates the currently pressed keys

        In addition, the sidebars are toggled if <Ctrl><Tab> is pressed.

        :param Gtk.Widget widget: The main window
        :param Gdk.Event event: The key press event
        """
        self.currently_pressed_keys.add(event.keyval)
        if event.keyval in [Gdk.KEY_Tab, Gdk.KEY_ISO_Left_Tab] and event.state & Gdk.ModifierType.CONTROL_MASK:
            self.toggle_sidebars()

    def _on_key_release(self, widget, event):
        """Updates the currently pressed keys

        :param Gtk.Widget widget: The main window
        :param Gdk.Event event: The key release event
        """
        self.currently_pressed_keys.discard(event.keyval)
        
    def prepare_destruction(self):
        """Saves current configuration of windows and panes to the runtime config file, before RAFCON is closed."""
        plugins.run_hook("pre_destruction")

        logger.debug("Saving runtime config to {0}".format(global_runtime_config.config_file_path))

        # store pane last positions
        for key, widget_name in constants.PANE_ID.items():
            global_runtime_config.store_widget_properties(self.view[widget_name], key.replace('_POS', ''))

        # store hidden or undocked widget flags correctly -> make them independent for restoring
        for window_key in constants.UNDOCKABLE_WINDOW_KEYS:
            hidden = False
            if not global_runtime_config.get_config_value(window_key + "_WINDOW_UNDOCKED"):
                hidden = getattr(self, window_key.lower() + '_hidden')
            global_runtime_config.set_config_value(window_key + '_HIDDEN', hidden)

        global_runtime_config.save_configuration()
        
        # state-editor will relieve it's model => it won't observe the state machine manager any more
        self.get_controller('states_editor_ctrl').prepare_destruction()  # avoid new state editor TODO tbd (deleted)
        rafcon.core.singleton.state_machine_manager.delete_all_state_machines()
        rafcon.core.singleton.library_manager.prepare_destruction()

        # gtkmvc installs a global glade custom handler that holds a reference to the last created View class,
        # preventing it from being destructed. By installing a dummy callback handler, after all views have been
        # created, the old handler is being removed and with it the reference, allowing all Views to be destructed.

        # Gtk TODO: check if necessary and search for replacement
        # try:
        #     from gtk import glade
        #     def dummy(*args, **kwargs):
        #         pass
        #     glade.set_custom_handler(dummy)
        # except ImportError:
        #     pass

        # Recursively destroys the main window
        self.destroy()
        from rafcon.gui.clipboard import global_clipboard
        global_clipboard.destroy()
        gui_singletons.main_window_controller = None
