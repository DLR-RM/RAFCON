

from rafcon.utils import log
from rafcon.core.singleton import state_machine_manager, library_manager
from rafcon.core import interface
from rafcon.core.storage import storage
from rafcon.gui.utils.dialog import RAFCONButtonDialog, ButtonDialog
from rafcon.gui.config import global_gui_config
import gtk
from rafcon.core.state_machine import StateMachine
from rafcon.core.states.hierarchy_state import HierarchyState

logger = log.get_logger(__name__)


def new_statemachine(menubar=None):
    if not menubar:
        error_no_menubar("new_statemachine")
        return
    import glib
    logger.debug("Creating new state-machine...")
    root_state = HierarchyState("new root state")
    state_machine = StateMachine(root_state)
    state_machine_manager.add_state_machine(state_machine)
    state_machine_manager.activate_state_machine_id = state_machine.state_machine_id
    state_machine_m = menubar.model.get_selected_state_machine_model()
    # If idle_add isn't used, gaphas crashes, as the view is not ready
    glib.idle_add(state_machine_m.selection.set, state_machine_m.root_state)

    def grab_focus():
        editor_controller = menubar.state_machines_editor_ctrl.get_controller(state_machine.state_machine_id)
        editor_controller.view.editor.grab_focus()

    # The editor parameter of view is created belated, thus we have to use idle_add again
    glib.idle_add(grab_focus)


def open_statemachine(path=None):
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
    except (AttributeError, ValueError, IOError) as e:
        logger.error('Error while trying to open state machine: {0}'.format(e))


def save_statemachine(menubar, widget, save_as=False, delete_old_state_machine=False):
        def on_message_dialog_response_signal(widget, response_id, source_editor_ctrl):
            state = source_editor_ctrl.model.state
            if response_id == ButtonDialog.OPTION_1.value:
                logger.debug("Applying source code changes of state '{}'".format(state.name))
                source_editor_ctrl.apply_clicked(None)

            elif response_id == ButtonDialog.OPTION_2.value:
                logger.debug("Ignoring source code changes of state '{}'".format(state.name))
            widget.destroy()

        state_machine_m = menubar.model.get_selected_state_machine_model()
        if state_machine_m is None:
            return

        all_tabs = menubar.states_editor_ctrl.tabs.values()
        all_tabs.extend(menubar.states_editor_ctrl.closed_tabs.values())
        dirty_source_editor_ctrls = [tab_dict['controller'].get_controller('source_ctrl') for tab_dict in all_tabs if
                                     tab_dict['source_code_view_is_dirty'] is True and
                                     tab_dict['state_m'].state.get_state_machine().state_machine_id ==
                                     state_machine_m.state_machine.state_machine_id]

        for dirty_source_editor_ctrl in dirty_source_editor_ctrls:
            state = dirty_source_editor_ctrl.model.state
            message_string = "The source code of the state '{}' (path: {}) has net been applied yet and would " \
                             "therefore not be stored.\n\nDo you want to apply the changes now?".format(state.name,
                                                                                                     state.get_path())
            if global_gui_config.get_config_value("AUTO_APPLY_SOURCE_CODE_CHANGES", False):
                dirty_source_editor_ctrl.apply_clicked(None)
            else:
                RAFCONButtonDialog(message_string, ["Apply", "Ignore changes"], on_message_dialog_response_signal,
                                   [dirty_source_editor_ctrl], type=gtk.MESSAGE_WARNING, parent=menubar.get_root_window())

        save_path = state_machine_m.state_machine.file_system_path
        if save_path is None:
            if not menubar.on_save_as_activate(widget, data=None):
                return

        logger.debug("Saving state machine to {0}".format(save_path))

        state_machine = menubar.model.get_selected_state_machine_model().state_machine
        storage.save_state_machine_to_path(state_machine, state_machine.file_system_path,
                                           delete_old_state_machine=delete_old_state_machine, save_as=save_as)

        menubar.model.get_selected_state_machine_model().store_meta_data()
        logger.debug("Successfully saved state machine and its meta data.")
        return True


def save_statemachine_as(menubar=None, widget=None, data=None, path=None):
    if not menubar:
        error_no_menubar("save_statemachine_as")
        return

    if path is None:
        if interface.create_folder_func is None:
            logger.error("No function defined for creating a folder")
            return False
        path = interface.create_folder_func("Please choose a root folder and a name for the state-machine")
        if path is None:
            return False
    menubar.model.get_selected_state_machine_model().state_machine.file_system_path = path
    save_statemachine(widget, data, save_as=True, delete_old_state_machine=True)


def refresh_libraries():
    library_manager.refresh_libraries()


def refresh_all(menubar=None, force=False):
    """Reloads all libraries and thus all state machines as well.
        :param menubar: the menubar where this method gets called from
        :param widget: the main widget
        :param data: optional data
    """
    if not menubar:
        error_no_menubar("refresh_all")
        return

    if force:
            menubar.refresh_libs_and_state_machines()
    else:

        # check if a state machine is still running
        if not menubar.state_machine_execution_engine.finished_or_stopped:
            if menubar.stopped_state_machine_to_proceed():
                pass  # state machine was stopped, proceeding reloading library
            else:
                return

        # check if the a dirty flag is still set
        all_tabs = menubar.states_editor_ctrl.tabs.values()
        all_tabs.extend(menubar.states_editor_ctrl.closed_tabs.values())
        dirty_source_editor = [tab_dict['controller'] for tab_dict in all_tabs if
                               tab_dict['source_code_view_is_dirty'] is True]
        if state_machine_manager.has_dirty_state_machine() or dirty_source_editor:

            def on_message_dialog_response_signal(widget, response_id):
                if response_id == ButtonDialog.OPTION_1.value:
                    menubar.refresh_libs_and_state_machines()
                else:
                    logger.debug("Refresh canceled")
                widget.destroy()

            message_string = "Are you sure you want to reload the libraries and all state machines?\n\n" \
                             "The following elements have been modified and not saved. " \
                             "These changes will get lost:"
            for sm_id, sm in state_machine_manager.state_machines.iteritems():
                if sm.marked_dirty:
                    message_string = "%s\n* State machine #%s and name '%s'" % (
                        message_string, str(sm_id), sm.root_state.name)
            for ctrl in dirty_source_editor:
                message_string = "%s\n* Source code of state with name '%s' and path '%s'" % (
                    message_string, ctrl.model.state.name, ctrl.model.state.get_path())
            RAFCONButtonDialog(message_string, ["Reload anyway", "Cancel"], on_message_dialog_response_signal,
                               type=gtk.MESSAGE_WARNING, parent=menubar.get_root_window())
        else:
            menubar.refresh_libs_and_state_machines()


def error_no_menubar(method_name="unspecified"):
    logger.error("Method '{0}' not called from a menubar, behaviour not specified".format(method_name))
