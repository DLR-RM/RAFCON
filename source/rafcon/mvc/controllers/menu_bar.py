"""
.. module:: menu_bar
   :platform: Unix, Windows
   :synopsis: A module that holds the menu bar controller with respective functionalities for each menu element.

.. moduleauthor:: Franz Steinmetz


"""

import os
import sys
from functools import partial

import gtk

from rafcon.statemachine import interface
from rafcon.statemachine.enums import StateMachineExecutionStatus
from rafcon.statemachine.state_machine import StateMachine
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.storage import storage
from rafcon.statemachine.singleton import state_machine_manager, library_manager

import rafcon.statemachine.singleton as core_singletons
from rafcon.mvc.models.state import StateModel
from rafcon.mvc.models.container_state import ContainerStateModel
from rafcon.mvc.models.scoped_variable import ScopedVariableModel
from rafcon.mvc import gui_helper
from rafcon.mvc import singleton as mvc_singleton
from rafcon.mvc.controllers.utils.extended_controller import ExtendedController
from rafcon.mvc.views.utils.about_dialog import MyAboutDialog
from rafcon.mvc.config import global_gui_config
from rafcon.mvc.runtime_config import global_runtime_config

from rafcon.mvc.utils.dialog import RAFCONDialog
from rafcon.utils import plugins
from rafcon.utils import log

logger = log.get_logger(__name__)


class MenuBarController(ExtendedController):
    """Controller handling the Menu Bar

    The class to trigger all the actions, available in the menu bar.

    :param rafcon.mvc.models.state_machine_manager.StateMachineManagerModel state_machine_manager_model: The state
        machine manager model, holding data regarding state machines. Should be exchangeable.
    :param rafcon.mvc.views.menu_bar.MenuBarView view: The GTK View showing the Menu Bar and Menu Items.
    """

    def __init__(self, state_machine_manager_model, view, shortcut_manager, sm_execution_engine):
        ExtendedController.__init__(self, state_machine_manager_model, view.menu_bar)
        self.state_machines_editor_ctrl = mvc_singleton.main_window_controller.\
            get_controller('state_machines_editor_ctrl')
        self.states_editor_ctrl = mvc_singleton.main_window_controller.get_controller('states_editor_ctrl')
        self.shortcut_manager = shortcut_manager
        self.logging_view = view.logging_view
        self.main_window_view = view
        self._destroyed = False
        self.handler_ids = {}
        self.registered_shortcut_callbacks = {}
        self.registered_view = False
        self.state_machine_execution_engine = sm_execution_engine

    def register_view(self, view):
        """Called when the View was registered"""
        data_flow_mode = global_runtime_config.get_config_value("DATA_FLOW_MODE", False)
        view["data_flow_mode"].set_active(data_flow_mode)

        show_all_data_flows = global_runtime_config.get_config_value("SHOW_DATA_FLOWS", True)
        view["show_all_data_flows"].set_active(show_all_data_flows)

        show_data_flow_values = global_runtime_config.get_config_value("SHOW_DATA_FLOW_VALUE_LABELS", True)
        view["show_data_flow_values"].set_active(show_data_flow_values)

        show_aborted_preempted = global_runtime_config.get_config_value("SHOW_ABORTED_PREEMPTED", False)
        view["show_aborted_preempted"].set_active(show_aborted_preempted)

        if not global_gui_config.get_config_value('GAPHAS_EDITOR'):
            view["data_flow_mode"].hide()
            view["show_data_flow_values"].hide()

        # use dedicated function to connect the buttons to be able to access the handler id later on
        self.connect_button_to_function('new', 'activate', self.on_new_activate)
        self.connect_button_to_function('open', 'activate', self.on_open_activate)
        self.connect_button_to_function('save', 'activate', self.on_save_activate)
        self.connect_button_to_function('save_as', 'activate', self.on_save_as_activate)
        self.connect_button_to_function('menu_properties', 'activate', self.on_menu_properties_activate)
        self.connect_button_to_function('refresh_all', 'activate', self.on_refresh_all_activate)
        self.connect_button_to_function('refresh_libraries', 'activate', self.on_refresh_libraries_activate)
        self.connect_button_to_function('quit', 'activate', self.on_quit_activate)

        self.connect_button_to_function('cut_selection', 'activate', self.on_cut_selection_activate)
        self.connect_button_to_function('copy_selection', 'activate', self.on_copy_selection_activate)
        self.connect_button_to_function('paste_clipboard', 'activate', self.on_paste_clipboard_activate)
        self.connect_button_to_function('delete', 'activate', self.on_delete_activate)
        self.connect_button_to_function('add_state', 'activate', self.on_add_state_activate)
        self.connect_button_to_function('group_states', 'activate', self.on_add_state_activate)
        self.connect_button_to_function('ungroup_state', 'activate', self.on_ungroup_state_activate)
        self.connect_button_to_function('undo', 'activate', self.on_undo_activate)
        self.connect_button_to_function('redo', 'activate', self.on_redo_activate)
        self.connect_button_to_function('grid', 'activate', self.on_grid_toggled)

        self.connect_button_to_function('data_flow_mode', 'toggled', self.on_data_flow_mode_toggled)
        self.connect_button_to_function('show_all_data_flows', 'toggled', self.on_show_all_data_flows_toggled)
        self.connect_button_to_function('show_data_flow_values', 'toggled', self.on_show_data_flow_values_toggled)
        self.connect_button_to_function('show_aborted_preempted', 'toggled', self.on_show_aborted_preempted_toggled)
        self.connect_button_to_function('expert_view', 'activate', self.on_expert_view_activate)

        self.connect_button_to_function('start', 'activate', self.on_start_activate)
        self.connect_button_to_function('start_from_selected_state', 'activate', self.on_start_from_selected_state_activate)
        self.connect_button_to_function('run_to_selected_state', 'activate', self.on_run_to_selected_state_activate)
        self.connect_button_to_function('pause', 'activate', self.on_pause_activate)
        self.connect_button_to_function('stop', 'activate', self.on_stop_activate)
        self.connect_button_to_function('step_mode', 'activate', self.on_step_mode_activate)
        self.connect_button_to_function('step_into', 'activate', self.on_step_into_activate)
        self.connect_button_to_function('step_over', 'activate', self.on_step_over_activate)
        self.connect_button_to_function('step_out', 'activate', self.on_step_out_activate)
        self.connect_button_to_function('backward_step', 'activate', self.on_backward_step_activate)
        self.connect_button_to_function('about', 'activate', self.on_about_activate)
        self.registered_view = True

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
        """
        Unregister all registered functions to a view element
        :return:
        """
        for handler_id in self.handler_ids.iterkeys():
            self.view[handler_id].disconnect(self.handler_ids[handler_id])

    def register_adapters(self):
        """Adapters should be registered in this method call"""
        pass

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.mvc.shortcut_manager.ShortcutManager shortcut_manager: Shortcut Manager Object holding mappings
            between shortcuts and actions.
        """
        self.add_callback_to_shortcut_manager('save', partial(self.call_action_callback, "on_save_activate"))
        self.add_callback_to_shortcut_manager('save_as', partial(self.call_action_callback, "on_save_as_activate"))
        self.add_callback_to_shortcut_manager('save_state_as', partial(self.call_action_callback,
                                                                       "on_save_selected_state_as_activate"))
        self.add_callback_to_shortcut_manager('open', partial(self.call_action_callback, "on_open_activate"))
        self.add_callback_to_shortcut_manager('new', partial(self.call_action_callback, "on_new_activate"))
        self.add_callback_to_shortcut_manager('quit', partial(self.call_action_callback, "on_quit_activate"))

        self.add_callback_to_shortcut_manager('group', partial(self.call_action_callback, "on_group_states_activate"))
        self.add_callback_to_shortcut_manager('ungroup', partial(self.call_action_callback, "on_ungroup_state_activate"))

        self.add_callback_to_shortcut_manager('start', partial(self.call_action_callback, "on_start_activate"))
        self.add_callback_to_shortcut_manager('start_from_selected', partial(self.call_action_callback,
                                                                             "on_start_from_selected_state_activate"))
        self.add_callback_to_shortcut_manager('run_to_selected', partial(self.call_action_callback,
                                                                         "on_run_to_selected_state_activate"))

        self.add_callback_to_shortcut_manager('stop', partial(self.call_action_callback, "on_stop_activate"))
        self.add_callback_to_shortcut_manager('pause', partial(self.call_action_callback, "on_pause_activate"))
        self.add_callback_to_shortcut_manager('step_mode', partial(self.call_action_callback, "on_step_mode_activate"))
        self.add_callback_to_shortcut_manager('step', partial(self.call_action_callback, "on_step_into_activate"))
        self.add_callback_to_shortcut_manager('backward_step', partial(self.call_action_callback, "on_backward_step_activate"))

        self.add_callback_to_shortcut_manager('reload', partial(self.call_action_callback, "on_refresh_all_activate"))

        self.add_callback_to_shortcut_manager('show_data_flows', self.show_all_data_flows_toggled_shortcut)
        self.add_callback_to_shortcut_manager('show_data_values', self.show_show_data_flow_values_toggled_shortcut)
        self.add_callback_to_shortcut_manager('data_flow_mode', self.data_flow_mode_toggled_shortcut)
        self.add_callback_to_shortcut_manager('show_aborted_preempted', self.show_aborted_preempted)

    def call_action_callback(self, callback_name, *args):
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
        for action in self.registered_shortcut_callbacks.iterkeys():
            for callback in self.registered_shortcut_callbacks[action]:
                self.shortcut_manager.remove_callback_for_action(action, callback)
        # delete all registered shortcut callbacks
        self.registered_shortcut_callbacks = {}

    ######################################################
    # menu bar functionality - File
    ######################################################
    def on_new_activate(self, widget=None, data=None):
        import glib
        logger.debug("Creating new state-machine...")
        root_state = HierarchyState("new root state")
        state_machine = StateMachine(root_state)
        state_machine_manager.add_state_machine(state_machine)
        state_machine_manager.activate_state_machine_id = state_machine.state_machine_id
        state_machine_m = self.model.get_selected_state_machine_model()
        # If idle_add isn't used, gaphas crashes, as the view is not ready
        glib.idle_add(state_machine_m.selection.set, state_machine_m.root_state)

        def grab_focus():
            editor_controller = self.state_machines_editor_ctrl.get_controller(state_machine.state_machine_id)
            editor_controller.view.editor.grab_focus()

        # The editor parameter of view is created belated, thus we have to use idle_add again
        glib.idle_add(grab_focus)

    def on_open_activate(self, widget=None, data=None, path=None):
        if path is None:
            if interface.open_folder_func is None:
                logger.error("No function defined for opening a folder")
                return
            load_path = interface.open_folder_func("Please choose the folder of the state machine")
            if load_path is None:
                return
        else:
            load_path = path

        try:
            state_machine = storage.load_state_machine_from_path(load_path)
            state_machine_manager.add_state_machine(state_machine)
        except (ValueError, IOError) as e:
            logger.error('Error while trying to open state machine: {0}'.format(e))

    def on_save_activate(self, widget, data=None, save_as=False):
        state_machine_m = self.model.get_selected_state_machine_model()
        if state_machine_m is None:
            return

        all_tabs = self.states_editor_ctrl.tabs.values()
        all_tabs.extend(self.states_editor_ctrl.closed_tabs.values())
        dirty_source_editor_ctrls = [tab_dict['controller'].get_controller('source_ctrl') for tab_dict in all_tabs if
                                     tab_dict['source_code_view_is_dirty'] is True
                                     and tab_dict[
                                     'state_m'].state.get_sm_for_state().state_machine_id ==
                                     state_machine_m.state_machine.state_machine_id]

        for dirty_source_editor_ctrl in dirty_source_editor_ctrls:
            def on_message_dialog_response_signal(widget, response_id):
                if response_id == 42:
                    widget.destroy()
                    dirty_source_editor_ctrl.apply_clicked(None)
                    logger.debug("Source script in editing stored before saving")

                elif response_id == 43:
                    logger.debug("Source script in editing is ignored while saving")
                    widget.destroy()

            dialog = RAFCONDialog(type=gtk.MESSAGE_WARNING, parent=self.get_root_window())
            message_string = "Are you sure you want to store the state machine without storing source Code in " \
                             "editing?\n\n" \
                             "The changes of state: %s name: %s have to be stored or ignored while saving. " % \
                             (dirty_source_editor_ctrl.model.state.get_path(), dirty_source_editor_ctrl.model.state.name)
            dialog.set_markup(message_string)
            dialog.add_button("Store", 42)
            dialog.add_button("Ignore", 43)
            dialog.grab_focus()
            dialog.finalize(on_message_dialog_response_signal)
            dialog.run()

        save_path = state_machine_m.state_machine.file_system_path
        if save_path is None:
            if not self.on_save_as_activate(widget, data=None):
                return

        logger.debug("Saving state machine to {0}".format(save_path))

        state_machine = self.model.get_selected_state_machine_model().state_machine
        storage.save_state_machine_to_path(state_machine, state_machine.file_system_path,
                                           delete_old_state_machine=False, save_as=save_as)

        self.model.get_selected_state_machine_model().store_meta_data()
        logger.debug("Successfully saved state machine and its meta data.")
        return True

    def on_save_as_activate(self, widget=None, data=None, path=None):
        if path is None:
            if interface.create_folder_func is None:
                logger.error("No function defined for creating a folder")
                return False
            path = interface.create_folder_func("Please choose a root folder and a name for the state-machine")
            if path is None:
                return False
        self.model.get_selected_state_machine_model().state_machine.file_system_path = path
        return self.on_save_activate(widget, data, save_as=True)

    def on_save_selected_state_as_activate(self, widget=None, data=None, path=None):
        selected_states = self.model.get_selected_state_machine_model().selection.get_states()
        logger.info("Save state as state machine. \n" + str(selected_states))
        logger.info("library_paths: " + str(library_manager._library_paths))
        if selected_states and len(selected_states) == 1:
            import copy
            state_m = copy.copy(selected_states[0])
            from rafcon.mvc.models.state_machine import StateMachineModel
            sm_m = StateMachineModel(StateMachine(root_state=state_m.state), self.model)
            sm_m.root_state = state_m
            from rafcon.statemachine.storage.storage import save_state_machine_to_path
            path = interface.create_folder_func("Please choose a root folder and a name for the state-machine")
            if path:
                save_state_machine_to_path(sm_m.state_machine, base_path=path, save_as=True)
                sm_m.store_meta_data()
            # check if state machine is in library path
            if any([root_path == path[:len(root_path)] for root_path in library_manager._library_paths.values()]):
                # TODO use a check box dialog with three check boxes and an confirmation and cancel button
                logger.info("DIALOG TO REQUEST SUBSTITUTION has to be insert here.")
                # logger.info("DIALOG TO REQUEST REFRESH OF LIBRARY-TREE has to be insert here.")

                def on_message_dialog_response_signal(widget, response_id):
                    if response_id in [42, 43, 44]:
                        widget.destroy()
                    if response_id == 42:
                        logger.debug("Library refresh is triggered.")
                        self.on_refresh_libraries_activate(None)
                    elif response_id == 43:
                        logger.debug("Refresh all is triggered.")
                        self.on_refresh_all_activate(None)
                    elif response_id == 44:
                        pass
                    else:
                        logger.warning("Response id: {} is not considered".format(response_id))

                dialog = RAFCONDialog(type=gtk.MESSAGE_WARNING, parent=self.get_root_window())
                message_string = "You stored your state machine in a path that is included into the library-paths." \
                                 "Do you want to refresh the library-tree or refresh all or may consider this later " \
                                 "manually?"
                dialog.set_markup(message_string)
                dialog.add_button("Refresh-Library-Tree", 42)
                dialog.add_button("Refresh-All", 43)
                dialog.add_button("May later manually", 44)
                dialog.grab_focus()
                dialog.finalize(on_message_dialog_response_signal)
                dialog.run()

            # logger.info("DIALOG TO REQUEST opening of new state machine has to be insert here.")

            def on_message_dialog_response_signal(widget, response_id):
                if response_id == 42:
                    logger.debug("Open state machine.")
                    try:
                        state_machine = storage.load_state_machine_from_path(path)
                        state_machine_manager.add_state_machine(state_machine)
                    except (ValueError, IOError) as e:
                        logger.error('Error while trying to open state machine: {0}'.format(e))
                elif response_id == 43:
                    pass
                else:
                    logger.warning("Response id: {} is not considered".format(response_id))
                if response_id in [42, 43]:
                    widget.destroy()

            dialog = RAFCONDialog(type=gtk.MESSAGE_WARNING, parent=self.get_root_window())
            message_string = "Should the newly from state created state machine be re-open?"
            dialog.set_markup(message_string)
            dialog.add_button("Yes", 42)
            dialog.add_button("No", 43)
            dialog.grab_focus()
            dialog.finalize(on_message_dialog_response_signal)
            dialog.run()
        else:
            logger.warning("Multiple states can not be saved as state machine directly. Group them before.")

    def on_menu_properties_activate(self, widget, data=None):
        # TODO: implement
        pass

    def on_refresh_libraries_activate(self, widget, data=None):
        """
        Deletes and reloads all libraries from the filesystem.
        :param widget: the main widget
        :param data: optional data
        :return:
        """
        library_manager.refresh_libraries()

    def on_refresh_all_activate(self, widget, data=None, force=False):
        """Reloads all libraries and thus all state machines as well.

        :param widget: the main widget
        :param data: optional data
        """
        if force:
            self.refresh_libs_and_state_machines()
        else:
            all_tabs = self.states_editor_ctrl.tabs.values()
            all_tabs.extend(self.states_editor_ctrl.closed_tabs.values())
            dirty_source_editor = [tab_dict['controller'] for tab_dict in all_tabs if
                                   tab_dict['source_code_view_is_dirty'] is True]
            if state_machine_manager.has_dirty_state_machine() or dirty_source_editor:

                def on_message_dialog_response_signal(widget, response_id):
                    if response_id == 42:
                        self.refresh_libs_and_state_machines()
                    else:
                        logger.debug("Refresh canceled")
                    widget.destroy()

                dialog = RAFCONDialog(type=gtk.MESSAGE_WARNING, parent=self.get_root_window())
                message_string = "Are you sure you want to reload the libraries and all state machines?\n\n" \
                                 "The following elements have been modified and not saved. " \
                                 "These changes will get lost:"
                for sm_id, sm in state_machine_manager.state_machines.iteritems():
                    if sm.marked_dirty:
                        message_string = "%s\nstate machine: #%s: %s " % (
                            message_string, str(sm_id), sm.root_state.name)
                for ctrl in dirty_source_editor:
                    message_string = "%s\nstate source: %s: %s of sm_id: #%s" % (
                        message_string, ctrl.model.state.get_path(), ctrl.model.state.name,
                        str(ctrl.model.state.get_sm_for_state().state_machine_id))
                dialog.set_markup(message_string)
                dialog.add_button("Reload anyway", 42)
                dialog.add_button("Cancel", 43)
                dialog.finalize(on_message_dialog_response_signal)
            else:
                self.refresh_libs_and_state_machines()

    def refresh_libs_and_state_machines(self):
        """Deletes all libraries and state machines and reloads them freshly from the file system."""
        library_manager.refresh_libraries()

        # delete dirty flags for state machines
        state_machine_manager.reset_dirty_flags()

        # create a dictionary from state machine id to state machine path
        state_machine_id_to_path = {}
        sm_keys = []
        for sm_id, sm in state_machine_manager.state_machines.iteritems():
            # the sm.base_path is only None if the state machine has never been loaded or saved before
            if sm.file_system_path is not None:
                # print sm.root_state.get_file_system_path()
                # cut the last directory from the path
                path_items = sm.root_state.get_file_system_path().split("/")
                new_path = path_items[0]
                for i in range(len(path_items) - 2):
                    new_path = "%s/%s" % (new_path, path_items[i + 1])
                # print new_path
                state_machine_id_to_path[sm_id] = new_path
                sm_keys.append(sm_id)

        self.states_editor_ctrl.close_all_pages()
        self.state_machines_editor_ctrl.close_all_pages()

        # reload state machines from file system
        state_machine_manager.refresh_state_machines(sm_keys, state_machine_id_to_path)

    def on_quit_activate(self, widget, data=None):
        avoid_shutdown = self.on_delete_event(self, widget, None)
        if not avoid_shutdown:
            self.on_destroy(None)

    def on_delete_event(self, widget, event, data=None):
        logger.debug("Delete event received")

        # State machine was modified, callback method handles closing operation
        if self.check_sm_modified():
            return True  # prevents closing operation
        # State machine is running, callback method handles closing operation
        if self.check_sm_running():
            return True  # prevents closing operation

        self.prepare_destruction()
        return False

    def check_sm_modified(self):
        if state_machine_manager.has_dirty_state_machine():

            def on_message_dialog_response_signal(widget, response_id):
                if response_id == 42:
                    widget.destroy()
                    if self.state_machine_execution_engine.status.execution_mode \
                            is not StateMachineExecutionStatus.STOPPED:
                        self.check_sm_running()
                    else:
                        self.prepare_destruction()
                        self.on_destroy(None)
                elif response_id == 43:
                    logger.debug("Close main window canceled")
                    widget.destroy()

            dialog = RAFCONDialog(type=gtk.MESSAGE_WARNING, parent=self.get_root_window())
            message_string = "Are you sure you want to exit RAFCON?\n\n" \
                             "The following state machines have been modified and not saved. " \
                             "These changes will get lost:"
            for sm_id, sm in state_machine_manager.state_machines.iteritems():
                if sm.marked_dirty:
                    message_string = "%s\n#%s: %s " % (message_string, str(sm_id), sm.root_state.name)
            dialog.set_markup(message_string)
            dialog.add_button("Close without saving", 42)
            dialog.add_button("Cancel", 43)
            dialog.finalize(on_message_dialog_response_signal)
            return True
        return False

    def check_sm_running(self):
        if self.state_machine_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STOPPED:

            def on_message_dialog_response_signal(widget, response_id):
                if response_id == 42:
                    self.state_machine_execution_engine.stop()
                    logger.debug("State machine is shut down now!")
                    widget.destroy()
                    self.prepare_destruction()
                    self.on_destroy(None)
                elif response_id == 43:
                    logger.debug("State machine will stay running!")
                    widget.destroy()
                    self.main_window_view.hide()
                    # state machine cannot be shutdown in a controlled manner as after self.destroy()
                    # the signal handler does not trigger any more
                    # self.destroy(None)

            dialog = RAFCONDialog(type=gtk.MESSAGE_QUESTION, parent=self.get_root_window())
            message_string = "The state machine is still running. Do you want to stop the state machine before closing?"
            dialog.set_markup(message_string)
            dialog.add_button("Stop state machine", 42)
            dialog.add_button("Keep running", 43)
            dialog.finalize(on_message_dialog_response_signal)
            return True
        return False

    def on_destroy(self, widget, data=None):

        # stop state machine
        try:
            if self.state_machine_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STOPPED:
                self.state_machine_execution_engine.stop()
                active_sm_id = self.state_machine_execution_engine.state_machine_manager.active_state_machine_id
                self.state_machine_execution_engine.state_machine_manager.state_machines[active_sm_id].root_state.join()
        except Exception as e:
            import traceback
            logger.exception("Could not stop state machine!")

        if "twisted" in sys.modules.keys():
            # from monitoring.monitoring_manager import global_monitoring_manager
            logger.info("Shutting down twisted from gtk and thus gtk itself")
            from twisted.internet import reactor
            logger.info("Shutting down monitoring manager")
            # global_monitoring_manager.shutdown()
            logger.info("Shutting down twisted")
            # the plugin may be installed but no reactor is running
            if reactor.running:
                reactor.callFromThread(reactor.stop)

        # shutdown gtk
        logger.debug("Closing main window!")
        self.main_window_view.hide()
        import glib
        glib.idle_add(gtk.main_quit)
        while gtk.events_pending():
            gtk.main_iteration(False)

    def prepare_destruction(self):
        """Saves current configuration of windows and panes to the runtime config file, before RAFCON is closed."""
        plugins.run_hook("pre_destruction")

        logger.debug("Saving runtime config")

        global_runtime_config.store_widget_properties(self.main_window_view.get_top_widget(), 'MAIN_WINDOW')
        global_runtime_config.store_widget_properties(self.main_window_view['top_level_h_pane'], 'LEFT_BAR_DOCKED')
        global_runtime_config.store_widget_properties(self.main_window_view['right_h_pane'], 'RIGHT_BAR_DOCKED')
        global_runtime_config.store_widget_properties(self.main_window_view['central_v_pane'], 'CONSOLE_DOCKED')
        global_runtime_config.store_widget_properties(self.main_window_view['left_bar_pane'], 'LEFT_BAR_INNER_PANE')

        if self.main_window_view.left_bar_window.get_top_widget().get_property('visible'):
            global_runtime_config.store_widget_properties(
                self.main_window_view.left_bar_window.get_top_widget(), 'LEFT_BAR_WINDOW')

        if self.main_window_view.right_bar_window.get_top_widget().get_property('visible'):
            global_runtime_config.store_widget_properties(
                self.main_window_view.right_bar_window.get_top_widget(), 'RIGHT_BAR_WINDOW')

        if self.main_window_view.console_bar_window.get_top_widget().get_property('visible'):
            global_runtime_config.store_widget_properties(
                self.main_window_view.console_bar_window.get_top_widget(), 'CONSOLE_BAR_WINDOW')

        global_runtime_config.save_configuration()

        import glib

        # Should close all tabs
        core_singletons.state_machine_manager.delete_all_state_machines()
        # Recursively destroys the main window

        mvc_singleton.main_window_controller.destroy()
        self.logging_view.quit_flag = True
        glib.idle_add(log.unregister_logging_view, 'main')

    ######################################################
    # menu bar functionality - Edit
    ######################################################

    def on_copy_selection_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("copy", None, None)

    def on_paste_clipboard_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("paste", None, None)

    def on_cut_selection_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("cut", None, None)

    def on_delete_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("delete", None, None)

    def on_add_state_activate(self, widget, method=None, *arg):
        self.shortcut_manager.trigger_action("add", None, None)

    def on_group_states_activate(self, widget, data=None):
        logger.debug("try to group")
        state_m_list = self.model.get_selected_state_machine_model().selection.get_states()
        all_elements = self.model.get_selected_state_machine_model().selection.get_all()
        selected_sv = [elem.scoped_variable for elem in all_elements if isinstance(elem, ScopedVariableModel)]
        if state_m_list and isinstance(state_m_list[0].parent, StateModel) or selected_sv:
            logger.debug("do group")
            state_ids_of_selected_states = [state_m.state.state_id for state_m in state_m_list]
            dp_ids_of_selected_sv = [sv.data_port_id for sv in selected_sv]
            if state_m_list:
                state_m_list[0].parent.state.group_states(state_ids_of_selected_states, dp_ids_of_selected_sv)
            else:
                selected_sv[0].parent.group_states(state_ids_of_selected_states, dp_ids_of_selected_sv)

    def on_ungroup_state_activate(self, widget, data=None):
        logger.debug("try to ungroup")
        state_m_list = self.model.get_selected_state_machine_model().selection.get_states()
        if len(state_m_list) == 1 and isinstance(state_m_list[0], ContainerStateModel) and \
                not state_m_list[0].state.is_root_state:
            logger.debug("do ungroup")
            state_m_list[0].parent.state.ungroup_state(state_m_list[0].state.state_id)

    def on_undo_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("undo", None, None)

    def on_redo_activate(self, widget, data=None):
        self.shortcut_manager.trigger_action("redo", None, None)

    def on_grid_toggled(self, widget, data=None):
        pass

    ######################################################
    # menu bar functionality - View
    ######################################################
    def data_flow_mode_toggled_shortcut(self, *args):
        if self.view["data_flow_mode"].get_active():
            self.view["data_flow_mode"].set_active(False)
        else:
            self.view["data_flow_mode"].set_active(True)

    def show_all_data_flows_toggled_shortcut(self, *args):
        if self.view["show_all_data_flows"].get_active():
            self.view["show_all_data_flows"].set_active(False)
        else:
            self.view["show_all_data_flows"].set_active(True)

    def show_show_data_flow_values_toggled_shortcut(self, *args):
        if self.view["show_data_flow_values"].get_active():
            self.view["show_data_flow_values"].set_active(False)
        else:
            self.view["show_data_flow_values"].set_active(True)

    def show_aborted_preempted(self, *args):
        if self.view["show_aborted_preempted"].get_active():
            self.view["show_aborted_preempted"].set_active(False)
        else:
            self.view["show_aborted_preempted"].set_active(True)

    def on_data_flow_mode_toggled(self, widget, data=None):
        if widget.get_active():
            global_runtime_config.set_config_value("DATA_FLOW_MODE", True)
        else:
            global_runtime_config.set_config_value("DATA_FLOW_MODE", False)

    def on_show_all_data_flows_toggled(self, widget, data=None):
        if widget.get_active():
            global_runtime_config.set_config_value("SHOW_DATA_FLOWS", True)
        else:
            global_runtime_config.set_config_value("SHOW_DATA_FLOWS", False)

    def on_show_data_flow_values_toggled(self, widget, data=None):
        if widget.get_active():
            global_runtime_config.set_config_value("SHOW_DATA_FLOW_VALUE_LABELS", True)
        else:
            global_runtime_config.set_config_value("SHOW_DATA_FLOW_VALUE_LABELS", False)

    def on_show_aborted_preempted_toggled(self, widget, data=None):
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
        sel = mvc_singleton.state_machine_manager_model.get_selected_state_machine_model().selection
        state_list = sel.get_states()
        if len(state_list) is not 1:
            logger.error("Exactly one state must be selected!")
        else:
            self.state_machine_execution_engine.start(self.model.selected_state_machine_id, state_list[0].state.get_path())

    def on_pause_activate(self, widget, data=None):
        self.state_machine_execution_engine.pause()

    def on_stop_activate(self, widget, data=None):
        self.state_machine_execution_engine.stop()

    def on_step_mode_activate(self, widget, data=None):
        self.state_machine_execution_engine.step_mode()

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

        sel = mvc_singleton.state_machine_manager_model.get_selected_state_machine_model().selection
        state_list = sel.get_states()
        if len(state_list) is not 1:
            logger.error("Exactly one state must be selected!")
        else:
            self.state_machine_execution_engine.run_to_selected_state(state_list[0].state.get_path(),
                                                                      self.model.selected_state_machine_id)

    ######################################################
    # menu bar functionality - Help
    ######################################################
    def on_about_activate(self, widget, data=None):
        about = MyAboutDialog()
        gui_helper.set_button_children_size_request(about)
        response = about.run()
        if response == gtk.RESPONSE_DELETE_EVENT or response == gtk.RESPONSE_CANCEL:
            about.destroy()
