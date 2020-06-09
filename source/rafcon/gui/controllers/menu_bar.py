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
.. module:: menu_bar
   :synopsis: A module that holds the menu bar controller with respective functionalities for each menu element.

"""

from builtins import str
import os
from gi.repository import GLib
from gi.repository import Gtk
from gi.repository import Gdk
from functools import partial

import rafcon.core.singleton as core_singletons
from rafcon.core.singleton import state_machine_manager, library_manager
from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from rafcon.gui import singleton as gui_singletons
from rafcon.gui.config import global_gui_config
from rafcon.gui.controllers.preferences_window import PreferencesWindowController
from rafcon.gui.controllers.utils.extended_controller import ExtendedController
from rafcon.gui.models.abstract_state import AbstractStateModel
from rafcon.gui.runtime_config import global_runtime_config
from rafcon.gui.utils import constants
from rafcon.gui.utils.dialog import RAFCONButtonDialog
from rafcon.gui.views.preferences_window import PreferencesWindowView
from rafcon.gui.views.main_window import MainWindowView
from rafcon.gui.views.utils.about_dialog import AboutDialogView
import rafcon.gui.backup.session as backup_session

import rafcon.gui.helpers.label as gui_helper_label
import rafcon.gui.helpers.state_machine as gui_helper_state_machine
import rafcon.gui.helpers.utility as gui_helper_utility

from rafcon.utils import plugins
from rafcon.utils import log

logger = log.get_logger(__name__)


class MenuBarController(ExtendedController):
    """Controller handling the Menu Bar

    The class to trigger all the actions, available in the menu bar.

    :param rafcon.gui.models.state_machine_manager.StateMachineManagerModel state_machine_manager_model: The state
        machine manager model, holding data regarding state machines. Should be exchangeable.
    :param rafcon.gui.views.main_window.MainWindowView view: The GTK View showing the Menu Bar and Menu Items.
    """

    def __init__(self, state_machine_manager_model, view, shortcut_manager, sm_execution_engine):
        assert isinstance(view, MainWindowView)
        ExtendedController.__init__(self, state_machine_manager_model, view.menu_bar)
        self.shortcut_manager = shortcut_manager
        self.logging_console_view = view.debug_console_view.logging_console_view
        self.main_window_view = view
        self.observe_model(gui_singletons.core_config_model)
        self.observe_model(gui_singletons.gui_config_model)
        self.observe_model(gui_singletons.runtime_config_model)

        self._destroyed = False
        self.handler_ids = {}
        self.registered_shortcut_callbacks = {}
        self.registered_view = False
        # this is not a model but the state machine execution engine of the core (or the overwritten execution engine
        # of the monitoring plugin
        self.state_machine_execution_engine = sm_execution_engine
        self.full_screen_flag = False
        self.full_screen_window = Gtk.Window(type=Gtk.WindowType.TOPLEVEL)
        self.main_position = None
        self.sm_notebook = self.main_window_view.state_machines_editor['notebook']
        self.full_screen_window.add_accel_group(self.shortcut_manager.accel_group)
        self.main_window_view.right_bar_window.get_top_widget().add_accel_group(self.shortcut_manager.accel_group)
        self.main_window_view.left_bar_window.get_top_widget().add_accel_group(self.shortcut_manager.accel_group)
        self.main_window_view.console_window.get_top_widget().add_accel_group(self.shortcut_manager.accel_group)

    def destroy(self):
        super(MenuBarController, self).destroy()
        self.full_screen_window.destroy()

    @staticmethod
    def create_logger_warning_if_shortcuts_are_overwritten_by_menu_bar():
        shortcut_dict = global_gui_config.get_config_value('SHORTCUTS')
        shortcut_key_patterns = [elem for l in shortcut_dict.values() for elem in l]
        for key in ['E', 'F', 'V', 'X', 'H', 'e', 'f', 'v', 'x', 'h']:
            if '<Alt>' + key in shortcut_key_patterns:
                dict_pair = {k: v for k, v_list in shortcut_dict.items() for v in v_list if '<Alt>' + key == v}
                logger.warning("Your current shortcut {0} is not working because a menu-bar access key "
                               "is overwriting it.".format(dict_pair))

    def register_view(self, view):
        """Called when the View was registered"""
        super(MenuBarController, self).register_view(view)
        data_flow_mode = global_runtime_config.get_config_value("DATA_FLOW_MODE", False)
        view["data_flow_mode"].set_active(data_flow_mode)

        show_data_flows = global_runtime_config.get_config_value("SHOW_DATA_FLOWS", True)
        view["show_data_flows"].set_active(show_data_flows)

        show_data_values = global_runtime_config.get_config_value("SHOW_DATA_FLOW_VALUE_LABELS", True)
        view["show_data_values"].set_active(show_data_values)

        show_aborted_preempted = global_runtime_config.get_config_value("SHOW_ABORTED_PREEMPTED", False)
        view["show_aborted_preempted"].set_active(show_aborted_preempted)

        view["expert_view"].hide()
        view["grid"].hide()

        # use dedicated function to connect the buttons to be able to access the handler id later on
        self.connect_button_to_function('new', 'activate', self.on_new_activate)
        self.connect_button_to_function('open', 'activate', self.on_open_activate)
        self.connect_button_to_function('save', 'activate', self.on_save_activate)
        self.connect_button_to_function('save_as', 'activate', self.on_save_as_activate)
        self.connect_button_to_function('save_as_copy', 'activate', self.on_save_as_copy_activate)
        self.connect_button_to_function('menu_preferences', 'activate', self.on_menu_preferences_activate)
        self.connect_button_to_function('refresh_all', 'activate', self.on_refresh_all_activate)
        self.connect_button_to_function('refresh_libraries', 'activate', self.on_refresh_libraries_activate)
        self.connect_button_to_function('bake_state_machine', 'activate', self.on_bake_state_machine_activate)
        self.connect_button_to_function('quit', 'activate', self.on_quit_activate)

        self.connect_button_to_function('cut', 'activate', self.on_cut_selection_activate)
        self.connect_button_to_function('copy', 'activate', self.on_copy_selection_activate)
        self.connect_button_to_function('paste', 'activate', self.on_paste_clipboard_activate)
        self.connect_button_to_function('delete', 'activate', self.on_delete_activate)
        self.connect_button_to_function('is_start_state', 'activate', self.on_toggle_is_start_state_active)
        self.connect_button_to_function('add', 'activate', self.on_add_state_activate)
        self.connect_button_to_function('group', 'activate', self.on_group_states_activate)
        self.connect_button_to_function('ungroup', 'activate', self.on_ungroup_state_activate)
        self.connect_button_to_function('substitute_state', 'activate', self.on_substitute_selected_state_activate)
        self.connect_button_to_function('save_state_as', 'activate', self.on_save_selected_state_as_activate)
        self.connect_button_to_function('undo', 'activate', self.on_undo_activate)
        self.connect_button_to_function('redo', 'activate', self.on_redo_activate)
        self.connect_button_to_function('grid', 'activate', self.on_grid_toggled)

        self.connect_button_to_function('data_flow_mode', 'toggled', self.on_data_flow_mode_toggled)
        self.connect_button_to_function('show_data_flows', 'toggled', self.on_show_data_flows_toggled)
        self.connect_button_to_function('show_data_values', 'toggled', self.on_show_data_values_toggled)
        self.connect_button_to_function('show_aborted_preempted', 'toggled', self.on_show_aborted_preempted_toggled)
        self.connect_button_to_function('expert_view', 'activate', self.on_expert_view_activate)
        self.connect_button_to_function('full_screen', 'toggled', self.on_full_screen_mode_toggled)

        self.connect_button_to_function('start', 'activate', self.on_start_activate)
        self.connect_button_to_function('start_from_selected', 'activate', self.on_start_from_selected_state_activate)
        self.connect_button_to_function('run_to_selected', 'activate', self.on_run_to_selected_state_activate)
        self.connect_button_to_function('pause', 'activate', self.on_pause_activate)
        self.connect_button_to_function('stop', 'activate', self.on_stop_activate)
        self.connect_button_to_function('step_mode', 'activate', self.on_step_mode_activate)
        self.connect_button_to_function('step_into', 'activate', self.on_step_into_activate)
        self.connect_button_to_function('step_over', 'activate', self.on_step_over_activate)
        self.connect_button_to_function('step_out', 'activate', self.on_step_out_activate)
        self.connect_button_to_function('backward_step', 'activate', self.on_backward_step_activate)
        self.connect_button_to_function('about', 'activate', self.on_about_activate)
        self.full_screen_window.connect('key_press_event', self.on_escape_key_press_event_leave_full_screen)
        self.view['menu_edit'].connect('select', self.check_edit_menu_items_status)
        self.registered_view = True
        self._update_recently_opened_state_machines()
        # do not move next line - here to show warning in GUI debug console
        self.create_logger_warning_if_shortcuts_are_overwritten_by_menu_bar()

    @ExtendedController.observe('config', after=True)
    def on_config_value_changed(self, config_m, prop_name, info):
        """Callback when a config value has been changed

        :param ConfigModel config_m: The config model that has been changed
        :param str prop_name: Should always be 'config'
        :param dict info: Information e.g. about the changed config key
        """
        config_key = info['args'][1]
        # config_value = info['args'][2]

        if config_key == "LIBRARY_PATHS":
            library_manager.refresh_libraries()
        elif config_key == "SHORTCUTS":
            self.refresh_shortcuts()
        elif config_key == "recently_opened_state_machines":
            self._update_recently_opened_state_machines()

    def _update_recently_opened_state_machines(self):
        """Update the sub menu Open Recent in File menu

        Method clean's first all menu items of the sub menu 'recent open', then insert the user menu item to clean
        recent opened state machine paths and finally insert menu items for all elements in recent opened state machines
        list.
        """
        if not self.registered_view:
            return

        for item in self.view.sub_menu_open_recently.get_children():
            self.view.sub_menu_open_recently.remove(item)

        menu_item = gui_helper_label.create_menu_item("remove invalid paths", constants.ICON_ERASE,
                                                      global_runtime_config.clean_recently_opened_state_machines)
        self.view.sub_menu_open_recently.append(menu_item)
        self.view.sub_menu_open_recently.append(Gtk.SeparatorMenuItem())

        for sm_path in global_runtime_config.get_config_value("recently_opened_state_machines", []):
            # define label string
            root_state_name = gui_helper_state_machine.get_root_state_name_of_sm_file_system_path(sm_path)
            if root_state_name is None and not os.path.isdir(sm_path):
                root_state_name = 'NOT_ACCESSIBLE'
            label_string = "'{0}' in {1}".format(root_state_name, sm_path) if root_state_name is not None else sm_path

            # define icon of menu item
            is_in_libs = library_manager.is_os_path_within_library_root_paths(sm_path)
            button_image = constants.SIGN_LIB if is_in_libs else constants.BUTTON_OPEN

            # prepare state machine open call_back function
            sm_open_function = partial(self.on_open_activate, path=sm_path)

            # create and insert new menu item
            menu_item = gui_helper_label.create_menu_item(label_string, button_image, sm_open_function)
            self.view.sub_menu_open_recently.append(menu_item)

        self.view.sub_menu_open_recently.show_all()

    def on_toggle_full_screen_mode(self, *args, **kwargs):
            self.view["full_screen"].set_active(False if self.view["full_screen"].get_active() else True)

    def on_full_screen_mode_toggled(self, *args):
        if self.full_screen_flag == self.view["full_screen"].get_active():
            return False

        if self.view["full_screen"].get_active() and not self.full_screen_flag:
            self.full_screen_flag = True
            self.on_full_screen_activate()
        else:
            self.full_screen_flag = False
            self.on_full_screen_deactivate()
        return True

    def on_escape_key_press_event_leave_full_screen(self, widget, event):
        keyname = Gdk.keyval_name(event.keyval)
        if keyname == "Escape" and 'fullscreen' in self.full_screen_window.get_window().get_state().value_nicks:
            self.view["full_screen"].set_active(False)
            return True

    def on_full_screen_activate(self, *args):
        """
        function to display the currently selected state machine in full screen mode
        :param args:
        :return:
        """
        self.sm_notebook.set_show_tabs(False)

        # Hide obsolete widgets of VBox
        self.main_window_view['graphical_editor_label_event_box'].hide()
        if not global_gui_config.get_config_value("FULLSCREEN_SHOW_TOOLBAR", True):
            self.main_window_view['graphical_editor_toolbar'].hide()
        self.main_window_view['console_return_button'].hide()

        # Move whole VBox into fullscreen window
        self.main_window_view['central_v_pane'].remove(self.main_window_view['central_vbox'])
        self.full_screen_window.add(self.main_window_view['central_vbox'])

        # Show fullscreen window undecorated in same screen as main window
        position = self.main_window_view.get_top_widget().get_position()
        self.full_screen_window.show()
        self.full_screen_window.move(position[0], position[1])
        self.full_screen_window.set_decorated(False)
        self.full_screen_window.fullscreen()
        self.main_window_view.get_top_widget().iconify()

    def on_full_screen_deactivate(self):
        # Move whole VBox back into main window
        self.full_screen_window.remove(self.main_window_view['central_vbox'])
        self.main_window_view['central_v_pane'].pack1(self.main_window_view['central_vbox'], True, False)

        self.sm_notebook.set_show_tabs(True)

        # Show elements of VBox again
        self.main_window_view['graphical_editor_label_event_box'].show()
        self.main_window_view['graphical_editor_toolbar'].show()
        if not self.main_window_view['central_v_pane'].get_child2():
            self.main_window_view['console_return_button'].show()

        self.main_window_view.get_top_widget().present()
        self.full_screen_window.hide()

    def connect_button_to_function(self, view_index, button_state, function):
        """
        Connect callback to a button
        :param view_index: the index of the button in the view
        :param button_state: the state of the button the function should be connected to
        :param function: the function to be connected
        :return:
        """
        handler_id = self.view[view_index].connect(button_state, function)
        self.handler_ids[view_index] = handler_id

    def unregister_view(self):
        """import log
        Unregister all registered functions to a view element
        :return:
        """
        for handler_id in self.handler_ids.keys():
            self.view[handler_id].disconnect(self.handler_ids[handler_id])

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.gui.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
            between shortcuts and actions.
        """
        self.add_callback_to_shortcut_manager('save', partial(self.call_action_callback, "on_save_activate"))
        self.add_callback_to_shortcut_manager('save_as', partial(self.call_action_callback, "on_save_as_activate"))
        self.add_callback_to_shortcut_manager('save_as_copy', partial(self.call_action_callback,
                                                                      "on_save_as_copy_activate"))
        self.add_callback_to_shortcut_manager('save_state_as', partial(self.call_action_callback,
                                                                       "on_save_selected_state_as_activate"))
        self.add_callback_to_shortcut_manager('substitute_state', partial(self.call_action_callback,
                                                                          "on_substitute_selected_state_activate"))
        self.add_callback_to_shortcut_manager('substitute_library_with_template',
                                              partial(self.call_action_callback,
                                                      "on_substitute_library_with_template_activate"))
        self.add_callback_to_shortcut_manager('open', partial(self.call_action_callback, "on_open_activate"))
        self.add_callback_to_shortcut_manager('open_library_state_separately',
                                              self.on_open_library_state_separately_activate)
        self.add_callback_to_shortcut_manager('new', partial(self.call_action_callback, "on_new_activate"))
        self.add_callback_to_shortcut_manager('quit', partial(self.call_action_callback, "on_quit_activate"))

        self.add_callback_to_shortcut_manager('is_start_state', partial(self.call_action_callback,
                                                                        "on_toggle_is_start_state_active"))
        callback_function = partial(self.call_action_callback, "on_add_transitions_from_closest_sibling_state_active")
        self.add_callback_to_shortcut_manager('transition_from_closest_sibling_state', callback_function)
        callback_function = partial(self.call_action_callback, "on_add_transitions_to_closest_sibling_state_active")
        self.add_callback_to_shortcut_manager('transition_to_closest_sibling_state', callback_function)
        callback_function = partial(self.call_action_callback, "on_add_transitions_to_parent_state_active")
        self.add_callback_to_shortcut_manager('transition_to_parent_state', callback_function)
        self.add_callback_to_shortcut_manager('group', partial(self.call_action_callback, "on_group_states_activate"))
        self.add_callback_to_shortcut_manager('ungroup', partial(self.call_action_callback,
                                                                 "on_ungroup_state_activate"))

        self.add_callback_to_shortcut_manager('start', partial(self.call_action_callback, "on_start_activate"))
        self.add_callback_to_shortcut_manager('start_from_selected', partial(self.call_action_callback,
                                                                             "on_start_from_selected_state_activate"))
        self.add_callback_to_shortcut_manager('run_to_selected', partial(self.call_action_callback,
                                                                         "on_run_to_selected_state_activate"))

        self.add_callback_to_shortcut_manager('stop', partial(self.call_action_callback, "on_stop_activate"))
        self.add_callback_to_shortcut_manager('pause', partial(self.call_action_callback, "on_pause_activate"))
        self.add_callback_to_shortcut_manager('step_mode', partial(self.call_action_callback, "on_step_mode_activate"))
        self.add_callback_to_shortcut_manager('step', partial(self.call_action_callback, "on_step_into_activate"))
        self.add_callback_to_shortcut_manager('backward_step', partial(self.call_action_callback,
                                                                       "on_backward_step_activate"))

        self.add_callback_to_shortcut_manager('reload', partial(self.call_action_callback, "on_refresh_all_activate"))

        self.add_callback_to_shortcut_manager('show_data_flows', self.show_data_flows_toggled_shortcut)
        self.add_callback_to_shortcut_manager('show_data_values', self.show_data_values_toggled_shortcut)
        self.add_callback_to_shortcut_manager('data_flow_mode', self.data_flow_mode_toggled_shortcut)
        self.add_callback_to_shortcut_manager('show_aborted_preempted', self.show_aborted_preempted)

        self.add_callback_to_shortcut_manager('fullscreen', self.on_toggle_full_screen_mode)

    def call_action_callback(self, callback_name, *args, **kwargs):
        """Wrapper for action callbacks

        Returns True after executing the callback. This is needed in order to prevent the shortcut from being passed
        on to the system. The callback methods itself cannot return True, as they are also used with idle_add,
        which would call the method over and over again.
        :param str callback_name: The name of the method to call
        :param args: Any remaining parameters, which are passed on to the callback method
        :return: True
        """
        getattr(self, callback_name)(*args)
        return True

    def add_callback_to_shortcut_manager(self, action, callback):
        """
        Helper function to add an callback for an action to the shortcut manager.
        :param action: the action to add a shortcut for
        :param callback: the callback if the action is executed
        :return:
        """
        if action not in self.registered_shortcut_callbacks:
            self.registered_shortcut_callbacks[action] = []
        self.registered_shortcut_callbacks[action].append(callback)
        self.shortcut_manager.add_callback_for_action(action, callback)

    def remove_all_callbacks(self):
        """
        Remove all callbacks registered to the shortcut manager
        :return:
        """
        for action in self.registered_shortcut_callbacks.keys():
            for callback in self.registered_shortcut_callbacks[action]:
                self.shortcut_manager.remove_callback_for_action(action, callback)
        # delete all registered shortcut callbacks
        self.registered_shortcut_callbacks = {}

    ######################################################
    # menu bar functionality - File
    ######################################################

    def on_new_activate(self, widget=None, data=None):
        return gui_helper_state_machine.new_state_machine()

    @staticmethod
    def on_open_activate(widget=None, data=None, path=None):
        return gui_helper_state_machine.open_state_machine(path=path, recent_opened_notification=True)

    @staticmethod
    def on_open_library_state_separately_activate(widget, data=None):
        gui_helper_state_machine.open_library_state_separately()

    def on_save_activate(self, widget, data=None, delete_old_state_machine=False):
        return gui_helper_state_machine.save_state_machine(delete_old_state_machine=delete_old_state_machine,
                                                           recent_opened_notification=True)

    def on_save_as_activate(self, widget=None, data=None, path=None):
        return gui_helper_state_machine.save_state_machine_as(path=path, recent_opened_notification=True)

    def on_save_as_copy_activate(self, widget=None, data=None, path=None):
        return gui_helper_state_machine.save_state_machine_as(path, recent_opened_notification=True, as_copy=True)

    @staticmethod
    def on_refresh_libraries_activate():
        gui_helper_state_machine.refresh_libraries()

    def on_bake_state_machine_activate(self, widget, data=None, force=False):
        gui_helper_state_machine.bake_selected_state_machine()

    def on_refresh_all_activate(self, widget, data=None, force=False):
        gui_helper_state_machine.refresh_all(force=force)

    def on_refresh_selected_activate(self, widget, data=None, force=False):
        gui_helper_state_machine.refresh_selected_state_machine()

    @staticmethod
    def on_substitute_selected_state_activate(widget=None, data=None, path=None):
        return gui_helper_state_machine.substitute_selected_state_and_use_choice_dialog()

    @staticmethod
    def on_substitute_library_with_template_activate(widget=None, data=None):
        keep_name = global_gui_config.get_config_value('SUBSTITUTE_STATE_KEEPS_STATE_NAME')
        return gui_helper_state_machine.substitute_selected_library_state_with_template(keep_name)

    @staticmethod
    def on_save_selected_state_as_activate(widget=None, data=None, path=None):
        return gui_helper_state_machine.save_selected_state_as()

    @staticmethod
    def on_menu_preferences_activate(widget, data=None):
        preferences_window_view = PreferencesWindowView()
        preferences_window_ctrl = PreferencesWindowController(gui_singletons.core_config_model, preferences_window_view,
                                                              gui_singletons.gui_config_model)
        gui_singletons.main_window_controller.add_controller('preferences_window_ctrl', preferences_window_ctrl)
        preferences_window_view.show()
        preferences_window_view.get_top_widget().present()

    def on_quit_activate(self, widget, data=None, force=False):
        global_runtime_config.prepare_recently_opened_state_machines_list_for_storage()
        if force:
            backup_session.reset_session()
        if not force and global_gui_config.get_config_value("SESSION_RESTORE_ENABLED"):
            backup_session.store_session()
            force = True
        avoid_shutdown = self.on_delete_event(widget, None, force=force)
        if not avoid_shutdown:
            self.on_destroy(None)

    def on_delete_event(self, widget, event, data=None, force=False):
        logger.debug("Delete event received")

        # State machine was modified, callback method handles closing operation
        if not force and self.on_delete_check_sm_modified():
            return True  # prevents closing operation

        # independently if state machine is running or not the GUI will be closed
        self.on_delete_check_sm_running()

        gui_singletons.main_window_controller.prepare_destruction()
        return False

    def refresh_shortcuts(self):
        self.shortcut_manager.remove_shortcuts()
        self.shortcut_manager.update_shortcuts()
        for item_name, shortcuts in global_gui_config.get_config_value('SHORTCUTS', {}).items():
            if shortcuts and item_name in self.view.buttons:
                self.view.set_menu_item_accelerator(item_name, shortcuts[0], remove_old=True)
        self.create_logger_warning_if_shortcuts_are_overwritten_by_menu_bar()

    def on_delete_check_sm_modified(self):
        if state_machine_manager.has_dirty_state_machine():
            message_string = "Are you sure you want to exit RAFCON?\n\n" \
                             "The following state machines have been modified and not saved. " \
                             "These changes will get lost:"
            for sm_id, sm in state_machine_manager.state_machines.items():
                if sm.marked_dirty:
                    message_string = "%s\n#%s: %s " % (message_string, str(sm_id), sm.root_state.name)
            dialog = RAFCONButtonDialog(message_string, ["Close without saving", "Cancel"],
                                        message_type=Gtk.MessageType.WARNING, parent=self.get_root_window())
            response_id = dialog.run()
            dialog.destroy()
            if response_id == 1:  # Close without saving - button pressed
                return False
            elif response_id == 2:  # Cancel - button pressed
                logger.debug("Close main window canceled")
                return True
        else:
            return False

    def on_delete_check_sm_running(self):
        if not self.state_machine_execution_engine.finished_or_stopped():
            message_string = "The state machine is still running. Do you want to stop the execution before closing?"
            dialog = RAFCONButtonDialog(message_string, ["Stop execution", "Keep running"],
                                        message_type=Gtk.MessageType.QUESTION, parent=self.get_root_window())
            response_id = dialog.run()
            dialog.destroy()
            if response_id == 1:  # Stop execution
                self.state_machine_execution_engine.stop()
                return False
            elif response_id == 2:  # Keep running
                logger.debug("State machine will keep running!")
                return True
        else:
            return False

    def on_destroy(self, widget, data=None):
        from rafcon.gui.start import stop_gtk

        logger.debug("The GUI is being closed now")
        self.main_window_view.hide()

        stop_gtk()

    ######################################################
    # menu bar functionality - Edit
    ######################################################

    @staticmethod
    def on_toggle_is_start_state_active(widget, data=None):
        return gui_helper_state_machine.selected_state_toggle_is_start_state()

    @staticmethod
    def on_add_transitions_from_closest_sibling_state_active(widget, data=None):
        return gui_helper_utility.add_transitions_from_closest_sibling_state_to_selected_state()

    @staticmethod
    def on_add_transitions_to_closest_sibling_state_active(widget, data=None):
        return gui_helper_utility.add_transitions_to_closest_sibling_state_from_selected_state()

    @staticmethod
    def on_add_transitions_to_parent_state_active(widget, data=None):
        return gui_helper_utility.add_transitions_from_selected_state_to_parent()

    def on_copy_selection_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("copy", None, None)

    def on_paste_clipboard_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("paste", None, None)

    def on_cut_selection_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("cut", None, None)

    def on_delete_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("delete", None, None)

    def on_add_state_activate(self, widget, method=None, *arg):
        self.shortcut_manager.trigger_action("add_execution_state", None, None)

    def on_group_states_activate(self, widget, data=None):
        gui_helper_state_machine.group_selected_states_and_scoped_variables()

    def on_ungroup_state_activate(self, widget, data=None):
        gui_helper_state_machine.ungroup_selected_state()

    def on_undo_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("undo", None, None)

    def on_redo_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("redo", None, None)

    def on_grid_toggled(self, widget, data=None):
        pass

    ######################################################
    # menu bar functionality - View
    ######################################################
    def data_flow_mode_toggled_shortcut(self, *args, **kwargs):
        if self.view["data_flow_mode"].get_active():
            self.view["data_flow_mode"].set_active(False)
        else:
            self.view["data_flow_mode"].set_active(True)

    def show_data_flows_toggled_shortcut(self, *args, **kwargs):
        if self.view["show_data_flows"].get_active():
            self.view["show_data_flows"].set_active(False)
        else:
            self.view["show_data_flows"].set_active(True)

    def show_data_values_toggled_shortcut(self, *args, **kwargs):
        if self.view["show_data_values"].get_active():
            self.view["show_data_values"].set_active(False)
        else:
            self.view["show_data_values"].set_active(True)

    def show_aborted_preempted(self, *args, **kwargs):
        if self.view["show_aborted_preempted"].get_active():
            self.view["show_aborted_preempted"].set_active(False)
        else:
            self.view["show_aborted_preempted"].set_active(True)

    @staticmethod
    def on_data_flow_mode_toggled(widget, data=None):
        if widget.get_active():
            global_runtime_config.set_config_value("DATA_FLOW_MODE", True)
        else:
            global_runtime_config.set_config_value("DATA_FLOW_MODE", False)

    @staticmethod
    def on_show_data_flows_toggled(widget, data=None):
        if widget.get_active():
            global_runtime_config.set_config_value("SHOW_DATA_FLOWS", True)
        else:
            global_runtime_config.set_config_value("SHOW_DATA_FLOWS", False)

    @staticmethod
    def on_show_data_values_toggled(widget, data=None):
        if widget.get_active():
            global_runtime_config.set_config_value("SHOW_DATA_FLOW_VALUE_LABELS", True)
        else:
            global_runtime_config.set_config_value("SHOW_DATA_FLOW_VALUE_LABELS", False)

    @staticmethod
    def on_show_aborted_preempted_toggled(widget, data=None):
        if widget.get_active():
            global_runtime_config.set_config_value("SHOW_ABORTED_PREEMPTED", True)
        else:
            global_runtime_config.set_config_value("SHOW_ABORTED_PREEMPTED", False)

    def on_expert_view_activate(self, widget, data=None):
        pass

    ######################################################
    # menu bar functionality - Execution
    ######################################################

    def on_start_activate(self, widget, data=None):
        self.state_machine_execution_engine.start(self.model.selected_state_machine_id)

    def on_start_from_selected_state_activate(self, widget, data=None):
        logger.debug("Run from selected state ...")
        selection = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection
        if len(selection.states) is not 1:
            logger.error("Exactly one state must be selected!")
        else:
            self.state_machine_execution_engine.start(self.model.selected_state_machine_id,
                                                      selection.get_selected_state().state.get_path())

    def on_pause_activate(self, widget, data=None):
        self.state_machine_execution_engine.pause()

    def on_stop_activate(self, widget, data=None):
        self.state_machine_execution_engine.stop()

    def on_step_mode_activate(self, widget, data=None):
        self.state_machine_execution_engine.step_mode(self.model.selected_state_machine_id)

    def on_step_into_activate(self, widget, data=None):
        self.state_machine_execution_engine.step_into()

    def on_step_over_activate(self, widget, data=None):
        self.state_machine_execution_engine.step_over()

    def on_step_out_activate(self, widget, data=None):
        self.state_machine_execution_engine.step_out()

    def on_backward_step_activate(self, widget, data=None):
        self.state_machine_execution_engine.backward_step()

    def on_run_to_selected_state_activate(self, widget, data=None):
        logger.debug("Run to selected state ...")

        selection = gui_singletons.state_machine_manager_model.get_selected_state_machine_model().selection
        if len(selection.states) is not 1:
            logger.error("Exactly one state must be selected!")
        else:
            self.state_machine_execution_engine.run_to_selected_state(selection.get_selected_state().state.get_path(),
                                                                      self.model.selected_state_machine_id)

    ######################################################
    # menu bar functionality - Help
    ######################################################
    @staticmethod
    def on_about_activate(widget, data=None):
        about = AboutDialogView()
        gui_helper_label.set_button_children_size_request(about)
        response = about.run()
        if response == Gtk.ResponseType.DELETE_EVENT or response == Gtk.ResponseType.CANCEL:
            about.destroy()

    def check_edit_menu_items_status(self, widget):

        # check if "is start state" is used,
        is_start_state_inactive = False
        if self.model.get_selected_state_machine_model():
            state_m_list = self.model.get_selected_state_machine_model().selection.states
            selected_state_m = self.model.get_selected_state_machine_model().selection.get_selected_state()
            has_no_start_state_state_types = (BarrierConcurrencyState, PreemptiveConcurrencyState)
            if len(state_m_list) == 1 and isinstance(selected_state_m, AbstractStateModel) and \
                    not selected_state_m.state.is_root_state and \
                    not isinstance(selected_state_m.parent.state, has_no_start_state_state_types):
                # if is start state -> enabled-box
                if selected_state_m.is_start:
                    self.view.set_menu_item_icon('is_start_state', constants.BUTTON_CHECK)
                else:  # if is not start state -> empty-box
                    self.view.set_menu_item_icon('is_start_state', constants.BUTTON_SQUARE)
                self.view.set_menu_item_sensitive('is_start_state', True)
            else:
                is_start_state_inactive = True
        else:
            is_start_state_inactive = True
        if is_start_state_inactive:  # if root state or otherwise -> inactive
            self.view.set_menu_item_icon('is_start_state', constants.BUTTON_SQUARE)
            self.view.set_menu_item_sensitive('is_start_state', False)
