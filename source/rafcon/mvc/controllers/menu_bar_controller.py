import gtk

from twisted.internet import reactor

from rafcon.statemachine.state_machine import StateMachine
from rafcon.statemachine.states.hierarchy_state import HierarchyState
import rafcon.statemachine.singleton
from rafcon.mvc.controllers.extended_controller import ExtendedController
from rafcon.utils import log
from rafcon.mvc.views.about_dialog import MyAboutDialog
from rafcon.statemachine.enums import StateMachineExecutionStatus

from rafcon.mvc.config import global_gui_config
from rafcon.mvc.runtime_config import global_runtime_config

logger = log.get_logger(__name__)
from rafcon.utils import helper
from rafcon.statemachine import interface
import rafcon.mvc.singleton
from functools import partial


class MenuBarController(ExtendedController):
    """
    The class to trigger all the action, available in the menu bar.
    """

    def __init__(self, state_machine_manager_model, view, state_machines_editor_ctrl, states_editor_ctrl, logging_view,
                 top_level_window, shortcut_manager):
        ExtendedController.__init__(self, state_machine_manager_model, view.menu_bar)
        self.state_machines_editor_ctrl = state_machines_editor_ctrl
        self.states_editor_ctrl = states_editor_ctrl
        self.shortcut_manager = shortcut_manager
        self.logging_view = logging_view
        self.main_window_view = view

    def register_view(self, view):
        """Called when the View was registered
        """
        data_flow_mode = global_gui_config.get_config_value("DATA_FLOW_MODE", True)
        view["data_flow_mode"].set_active(data_flow_mode)

        show_all_data_flows = global_gui_config.get_config_value("SHOW_DATA_FLOWS", True)
        view["show_all_data_flows"].set_active(show_all_data_flows)

        show_data_flow_values = global_gui_config.get_config_value("SHOW_DATA_FLOW_VALUE_LABELS", False)
        view["show_data_flow_values"].set_active(show_data_flow_values)

        show_aborted_preempted = global_gui_config.get_config_value("SHOW_ABORTED_PREEMPTED", False)
        view["show_aborted_preempted"].set_active(show_aborted_preempted)

        if not global_gui_config.get_config_value('GAPHAS_EDITOR'):
            view["data_flow_mode"].hide()
            view["show_data_flow_values"].hide()

    def register_adapters(self):
        """Adapters should be registered in this method call
        """
        pass

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.mvc.shortcut_manager.ShortcutManager shortcut_manager:
        """
        shortcut_manager.add_callback_for_action('save', partial(self.call_action_callback, "on_save_activate"))
        shortcut_manager.add_callback_for_action('save_as', partial(self.call_action_callback, "on_save_as_activate"))
        shortcut_manager.add_callback_for_action('open', partial(self.call_action_callback, "on_open_activate"))
        shortcut_manager.add_callback_for_action('new', partial(self.call_action_callback, "on_new_activate"))
        shortcut_manager.add_callback_for_action('quit', partial(self.call_action_callback, "on_quit_activate"))

        shortcut_manager.add_callback_for_action('start', partial(self.call_action_callback, "on_start_activate"))
        shortcut_manager.add_callback_for_action('stop', partial(self.call_action_callback, "on_stop_activate"))
        shortcut_manager.add_callback_for_action('pause', partial(self.call_action_callback, "on_pause_activate"))
        shortcut_manager.add_callback_for_action('step_mode', partial(self.call_action_callback, "on_step_mode_activate"))
        shortcut_manager.add_callback_for_action('step', partial(self.call_action_callback, "on_step_activate"))
        shortcut_manager.add_callback_for_action('backward_step', partial(self.call_action_callback, "on_backward_step_activate"))

        shortcut_manager.add_callback_for_action('reload', partial(self.call_action_callback, "on_refresh_all_activate"))

        shortcut_manager.add_callback_for_action('show_data_flows', self.show_all_data_flows_toggled_shortcut)
        shortcut_manager.add_callback_for_action('show_data_values', self.show_show_data_flow_values_toggled_shortcut)
        shortcut_manager.add_callback_for_action('data_flow_mode', self.data_flow_mode_toggled_shortcut)
        shortcut_manager.add_callback_for_action('show_aborted_preempted', self.show_aborted_preempted)

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

    ######################################################
    # menu bar functionality - File
    ######################################################
    def on_new_activate(self, widget=None, data=None):
        import glib
        logger.debug("Creating new state-machine...")
        root_state = HierarchyState("new root state")
        state_machine = StateMachine(root_state)
        rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
        rafcon.statemachine.singleton.state_machine_manager.activate_state_machine_id = state_machine.state_machine_id
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
            load_path = interface.open_folder_func("Please choose the folder of the state-machine")
            if load_path is None:
                return
        else:
            load_path = path

        try:
            [state_machine, version, creation_time] = rafcon.statemachine.singleton. \
                global_storage.load_statemachine_from_yaml(load_path)
            rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
        except AttributeError as e:
            logger.error('Error while trying to open state-machine: {0}'.format(e))

    def on_save_activate(self, widget, data=None, save_as=False):
        state_machine_m = self.model.get_selected_state_machine_model()
        if state_machine_m is None:
            return
        save_path = state_machine_m.state_machine.file_system_path
        if save_path is None:
            if not self.on_save_as_activate(widget, data=None):
                return

        logger.debug("Saving state machine to {0}".format(save_path))
        rafcon.statemachine.singleton.global_storage.save_statemachine_as_yaml(
            self.model.get_selected_state_machine_model().state_machine,
            self.model.get_selected_state_machine_model().state_machine.file_system_path,
            delete_old_state_machine=False, save_as=save_as)

        self.model.get_selected_state_machine_model().root_state.store_meta_data_for_state()
        logger.debug("Successfully saved graphics meta data.")

    def on_save_as_activate(self, widget=None, data=None, path=None):
        if path is None:
            if interface.create_folder_func is None:
                logger.error("No function defined for creating a folder")
                return False
            path = interface.create_folder_func("Please choose a root folder and a name for the state-machine")
            if path is None:
                return False
        self.model.get_selected_state_machine_model().state_machine.file_system_path = path
        self.on_save_activate(widget, data, save_as=True)

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
        rafcon.statemachine.singleton.library_manager.refresh_libraries()

    def on_refresh_all_activate(self, widget, data=None, force=False):
        """
        Reloads all libraries and thus all state machines as well.
        :param widget: the main widget
        :param data: optional data
        :return:
        """
        if force:
            self.refresh_libs_and_statemachines()
        else:
            if rafcon.statemachine.singleton.state_machine_manager.check_if_dirty_sms():
                message = gtk.MessageDialog(type=gtk.MESSAGE_INFO, buttons=gtk.BUTTONS_NONE, flags=gtk.DIALOG_MODAL)
                message_string = "Are you sure you want to reload the libraries and thus all state_machines. " \
                                 "The following state machines were modified and not saved: "
                for sm_id, sm in rafcon.statemachine.singleton.state_machine_manager.state_machines.iteritems():
                    if sm.marked_dirty:
                        message_string = "%s %s " % (message_string, str(sm_id))
                message_string = "%s \n(Note: all state machines that are freshly created and have never been saved " \
                                 "before will be deleted!)" % message_string
                message.set_markup(message_string)
                message.add_button("Yes", 42)
                message.add_button("No", 43)
                message.connect('response', self.on_refresh_message_dialog_response_signal)
                message.show()
            else:
                self.refresh_libs_and_statemachines()

    def on_refresh_message_dialog_response_signal(self, widget, response_id):
        if response_id == 42:
            self.refresh_libs_and_statemachines()
        else:
            logger.debug("Refresh canceled")
        widget.destroy()

    def refresh_libs_and_statemachines(self):
        """
        Deletes all libraries and state machines and reloads them freshly from the file system.
        :return:
        """
        rafcon.statemachine.singleton.library_manager.refresh_libraries()

        # delete dirty flags for state machines
        rafcon.statemachine.singleton.state_machine_manager.reset_dirty_flags()

        # create a dictionary from state machine id to state machine path
        state_machine_id_to_path = {}
        sm_keys = []
        for sm_id, sm in rafcon.statemachine.singleton.state_machine_manager.state_machines.iteritems():
            # the sm.base_path is only None if the state machine has never been loaded or saved before
            if sm.file_system_path is not None:
                # print sm.root_state.get_file_system_path()
                # cut the last directory from the path
                path_items = sm.root_state.get_file_system_path().split("/")
                new_path = path_items[0]
                for i in range(len(path_items) - 2):
                    new_path = "%s/%s" % (new_path, path_items[i + 1])
                #print new_path
                state_machine_id_to_path[sm_id] = new_path
                sm_keys.append(sm_id)

        self.states_editor_ctrl.close_all_pages()
        self.state_machines_editor_ctrl.close_all_pages()

        # reload state machines from file system
        rafcon.statemachine.singleton.state_machine_manager.refresh_state_machines(sm_keys,
                                                                                         state_machine_id_to_path)

    def on_quit_activate(self, widget, data=None):
        avoid_shutdown = self.on_delete_event(self, widget, None)
        if not avoid_shutdown:
            self.destroy(None)

    def on_delete_event(self, widget, event, data=None):
        logger.debug("Delete event received")

        # State machine was modified, callback method handles closing operation
        if self.check_sm_modified():
            return True  # prevents closing operation
        # State machine is running, callback method handles closing operation
        if self.check_sm_running():
            return True  # prevents closing operation

        self._prepare_destruction()
        return False

    def check_sm_modified(self):
        if rafcon.statemachine.singleton.state_machine_manager.check_if_dirty_sms():
            message = gtk.MessageDialog(type=gtk.MESSAGE_INFO, buttons=gtk.BUTTONS_NONE, flags=gtk.DIALOG_MODAL)
            message_string = "Are you sure you want to close the main window? " \
                             "The following state machines were modified and not saved: "
            for sm_id, sm in rafcon.statemachine.singleton.state_machine_manager.state_machines.iteritems():
                if sm.marked_dirty:
                    message_string = "%s %s " % (message_string, str(sm_id))
            message_string = "%s \n(Note: all state machines that are freshly created and have never been saved " \
                             "before will be deleted!)" % message_string
            message.set_markup(message_string)
            message.add_button("Yes", 42)
            message.add_button("No", 43)
            message.connect('response', self.on_quit_message_dialog_response_signal_open_changes)
            helper.set_button_children_size_request(message)
            message.show()
            return True
        return False

    def check_sm_running(self):
        if rafcon.statemachine.singleton.state_machine_execution_engine.status.execution_mode \
                is not StateMachineExecutionStatus.STOPPED:
            message = gtk.MessageDialog(type=gtk.MESSAGE_INFO, buttons=gtk.BUTTONS_NONE, flags=gtk.DIALOG_MODAL)
            message_string = "The state machine is still running. Do you want to stop the state machine before closing?"
            message.set_markup(message_string)
            message.add_button("Yes", 42)
            message.add_button("No", 43)
            message.connect('response', self.on_quit_message_dialog_response_signal_sm_running)
            helper.set_button_children_size_request(message)
            message.show()
            return True
        return False

    def on_quit_message_dialog_response_signal_open_changes(self, widget, response_id):
        if response_id == 42:
            widget.destroy()
            if rafcon.statemachine.singleton.state_machine_execution_engine.status.execution_mode \
                    is not StateMachineExecutionStatus.STOPPED:
                self.check_sm_running()
            else:
                self._prepare_destruction()
                self.destroy(None)
        elif response_id == 43:
            logger.debug("Close main window canceled")
            widget.destroy()

    def on_quit_message_dialog_response_signal_sm_running(self, widget, response_id):
        if response_id == 42:
            rafcon.statemachine.singleton.state_machine_execution_engine.stop()
            logger.debug("State machine is shut down now!")
            widget.destroy()
            self._prepare_destruction()
            self.destroy(None)
        elif response_id == 43:
            logger.debug("State machine will stay running!")
            widget.destroy()
            self.main_window_view.hide()
            # state machine cannot be shutdown in a controlled manner as after self.destroy()
            # the signal handler does not trigger any more
            # self.destroy(None)

    def destroy(self, widget, data=None):
        import glib
        logger.debug("Closing main window!")
        self.main_window_view.hide()
        glib.idle_add(gtk.main_quit)

    def _prepare_destruction(self):
        logger.debug("Saving runtime config")
        global_runtime_config.save_configuration(self.main_window_view)
        import glib

        # We decided on not saving the configuration when exiting
        # glib.idle_add(rafcon.statemachine.config.global_config.save_configuration)
        # glib.idle_add(rafcon.mvc.config.global_gui_config.save_configuration)
        self.logging_view.quit_flag = True
        glib.idle_add(log.debug_filter.set_logging_test_view, None)
        glib.idle_add(log.error_filter.set_logging_test_view, None)
        if reactor.running:
            glib.idle_add(reactor.stop)

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

    def on_ungroup_states_activate(self, widget, data=None):
        pass

    def on_group_states_activate(self, widget, data=None):
        pass

    def on_undo_activate(self, widget, data=None):
        pass

    def on_redo_activate(self, widget, data=None):
        pass

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
            global_gui_config.set_config_value("DATA_FLOW_MODE", True)
        else:
            global_gui_config.set_config_value("DATA_FLOW_MODE", False)

    def on_show_all_data_flows_toggled(self, widget, data=None):
        if widget.get_active():
            global_gui_config.set_config_value("SHOW_DATA_FLOWS", True)
        else:
            global_gui_config.set_config_value("SHOW_DATA_FLOWS", False)

    def on_show_data_flow_values_toggled(self, widget, data=None):
        if widget.get_active():
            global_gui_config.set_config_value("SHOW_DATA_FLOW_VALUE_LABELS", True)
        else:
            global_gui_config.set_config_value("SHOW_DATA_FLOW_VALUE_LABELS", False)

    def on_show_aborted_preempted_toggled(self, widget, data=None):
        if widget.get_active():
            global_gui_config.set_config_value("SHOW_ABORTED_PREEMPTED", True)
        else:
            global_gui_config.set_config_value("SHOW_ABORTED_PREEMPTED", False)

    def on_expert_view_activate(self, widget, data=None):
        pass

    ######################################################
    # menu bar functionality - Execution
    ######################################################
    def on_start_activate(self, widget, data=None):
        logger.debug("Start execution engine ...")
        rafcon.statemachine.singleton.state_machine_execution_engine.start(self.model.selected_state_machine_id)

    def on_start_from_selected_state_activate(self, widget, data=None):
        logger.debug("Start from selected state ...")
        sel = rafcon.mvc.singleton.state_machine_manager_model.get_selected_state_machine_model().selection
        state_list = sel.get_states()
        if len(state_list) is not 1:
            logger.error("Exactly one state must be selected!")
        else:
            rafcon.statemachine.singleton.state_machine_execution_engine.start(
                self.model.selected_state_machine_id, state_list[0].state.get_path())

    def on_pause_activate(self, widget, data=None):
        logger.debug("Pause execution engine ...")
        rafcon.statemachine.singleton.state_machine_execution_engine.pause()

    def on_stop_activate(self, widget, data=None):
        logger.debug("Stop execution engine ...")
        rafcon.statemachine.singleton.state_machine_execution_engine.stop()

    def on_step_mode_activate(self, widget, data=None):
        logger.debug("Activate execution engine step mode ...")
        rafcon.statemachine.singleton.state_machine_execution_engine.step_mode()

    def on_step_activate(self, widget, data=None):
        logger.debug("Execution step ...")
        rafcon.statemachine.singleton.state_machine_execution_engine.step()

    def on_backward_step_activate(self, widget, data=None):
        logger.debug("Executing backward step ...")
        rafcon.statemachine.singleton.state_machine_execution_engine.backward_step()

    ######################################################
    # menu bar functionality - Help
    ######################################################
    def on_about_activate(self, widget, data=None):
        about = MyAboutDialog()
        helper.set_button_children_size_request(about)
        response = about.run()
        if response == gtk.RESPONSE_DELETE_EVENT or response == gtk.RESPONSE_CANCEL:
            about.destroy()