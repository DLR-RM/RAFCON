from rafcon.utils import log
from rafcon.core.singleton import state_machine_manager, library_manager
from rafcon.core import interface
from rafcon.core.storage import storage
from rafcon.gui.utils.dialog import RAFCONButtonDialog, ButtonDialog
from rafcon.gui.config import global_gui_config
import gtk
logger = log.get_logger(__name__)

def open_statemachine(widget=None, data=None, path=None):
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

def save_statemachine(self, widget, data=None, save_as=False, delete_old_state_machine=False):
        def on_message_dialog_response_signal(widget, response_id, source_editor_ctrl):
            state = source_editor_ctrl.model.state
            if response_id == ButtonDialog.OPTION_1.value:
                logger.debug("Applying source code changes of state '{}'".format(state.name))
                source_editor_ctrl.apply_clicked(None)

            elif response_id == ButtonDialog.OPTION_2.value:
                logger.debug("Ignoring source code changes of state '{}'".format(state.name))
            widget.destroy()

        state_machine_m = self.model.get_selected_state_machine_model()
        if state_machine_m is None:
            return

        all_tabs = self.states_editor_ctrl.tabs.values()
        all_tabs.extend(self.states_editor_ctrl.closed_tabs.values())
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
                                   [dirty_source_editor_ctrl], type=gtk.MESSAGE_WARNING, parent=self.get_root_window())

        save_path = state_machine_m.state_machine.file_system_path
        if save_path is None:
            if not self.on_save_as_activate(widget, data=None):
                return

        logger.debug("Saving state machine to {0}".format(save_path))

        state_machine = self.model.get_selected_state_machine_model().state_machine
        storage.save_state_machine_to_path(state_machine, state_machine.file_system_path,
                                           delete_old_state_machine=delete_old_state_machine, save_as=save_as)

        self.model.get_selected_state_machine_model().store_meta_data()
        logger.debug("Successfully saved state machine and its meta data.")
        return True