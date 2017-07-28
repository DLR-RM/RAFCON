""" Module collects methods and function to be integrated into a respective class if that is of advantage, in future.
"""
import os

from rafcon.core.storage import storage
from rafcon.utils import log, storage_utils
logger = log.get_logger(__name__)


def store_session():
    """ Stores reference backup information for all open tabs into runtime config

    The backup of never stored tabs (state machines) and not stored state machine changes will be triggered a last
    time to secure data lose.
    """
    from rafcon.gui.singleton import state_machine_manager_model, global_runtime_config
    from rafcon.gui.models.auto_backup import AutoBackupModel
    from rafcon.gui.models import AbstractStateModel
    from rafcon.gui.singleton import main_window_controller
    # check if there are dirty state machines -> use backup file structure maybe it is already stored
    for sm_m in state_machine_manager_model.state_machines.itervalues():
        if hasattr(sm_m, 'auto_backup'):
            if sm_m.state_machine.marked_dirty:
                sm_m.auto_backup.perform_temp_storage()
        else:
            # generate a backup
            sm_m.auto_backup = AutoBackupModel(sm_m)

    # collect order of tab state machine ids from state machines editor and find selected state machine page number
    state_machines_editor_ctrl = main_window_controller.get_controller('state_machines_editor_ctrl')
    number_of_pages = state_machines_editor_ctrl.view['notebook'].get_n_pages()
    selected_page_number = None
    ordered_sm_ids = []
    for page_number in range(number_of_pages):
        page = state_machines_editor_ctrl.view['notebook'].get_nth_page(page_number)
        ordered_sm_ids.append(state_machines_editor_ctrl.get_state_machine_id_for_page(page))
        if ordered_sm_ids[-1] == state_machine_manager_model.selected_state_machine_id:
            selected_page_number = page_number

    # backup state machine selection
    selection_of_selected_sm = []
    if selected_page_number:
        for model in state_machine_manager_model.get_selected_state_machine_model().selection.get_all():
            if isinstance(model, AbstractStateModel):
                # TODO extend to full range of selection -> see core_identifier action-module
                selection_of_selected_sm.append(model.state.get_path())

    # store final state machine backup meta data to backup session tabs for the next run
    list_of_tab_meta = [state_machine_manager_model.state_machines[sm_id].auto_backup.meta for sm_id in ordered_sm_ids]
    global_runtime_config.set_config_value('open_tabs', list_of_tab_meta)
    # store state-machines-editor- and selected-state-machine-selection
    global_runtime_config.set_config_value('selection', {'state_machine_page_number': selected_page_number,
                                                         'selection_of_selected_sm': selection_of_selected_sm})


def reset_session():
    """Remove all information from the current session """
    from rafcon.gui.singleton import global_runtime_config
    global_runtime_config.set_config_value('open_tabs', [])
    global_runtime_config.set_config_value('selection', {})


def restore_session_from_runtime_config():
    """ Restore stored tabs from runtime config

    The method checks if the last status of a state machine is in the backup or in tis original path and loads it
    from there. The original path of these state machines are also insert into the recently opened state machines
    list.
    """
    # TODO add a dirty lock for a crashed rafcon instance also into backup session feature
    # TODO in case a dialog is needed to give the user control
    # TODO combine this and auto-backup in one structure/controller/observer
    from rafcon.gui.singleton import state_machine_manager_model, global_runtime_config, global_gui_config
    from rafcon.gui.models.auto_backup import recover_state_machine_from_backup
    from rafcon.gui.singleton import main_window_controller
    # check if session storage exists
    open_tabs = global_runtime_config.get_config_value('open_tabs', None)
    if open_tabs is None:
        logger.info("No session found for recovery")
        return

    # load and recover state machines like they were opened before
    for idx, backup_meta_dict in enumerate(open_tabs):
        from_backup_path = None
        # TODO do this decision before storing or maybe store the last stored time in the auto backup?!
        # pick folder name dependent on time, and backup meta data existence
        # problem is that the backup time is maybe not the best choice
        if 'last_backup' in backup_meta_dict:
            last_backup_time = storage_utils.get_float_time_for_string(backup_meta_dict['last_backup']['time'])
            if 'last_saved' in backup_meta_dict:
                last_save_time = storage_utils.get_float_time_for_string(backup_meta_dict['last_saved']['time'])
                backup_marked_dirty = backup_meta_dict['last_backup']['marked_dirty']
                if last_backup_time > last_save_time and backup_marked_dirty:
                    from_backup_path = backup_meta_dict['last_backup']['file_system_path']
            else:
                from_backup_path = backup_meta_dict['last_backup']['file_system_path']
        elif 'last_saved' in backup_meta_dict:
            # print "### open last saved", sm_meta_dict['last_saved']['file_system_path']
            pass
        else:
            logger.error("A tab was stored into session storage dictionary {0} without any recovery path"
                         "".format(backup_meta_dict))
            continue

        # check in case that the backup folder is valid or use last saved path
        if from_backup_path is not None and not os.path.isdir(from_backup_path):
            logger.warning("The backup of tab {0} from backup path {1} was not possible. "
                           "The last saved path will be used for recovery, which could result is loss of changes."
                           "".format(idx, from_backup_path))
            from_backup_path = None

        # open state machine
        if from_backup_path is not None:
            # open state machine, recover mark dirty flags, cleans dirty lock files
            logger.info("Restoring from backup {0}".format(from_backup_path))
            recover_state_machine_from_backup(from_backup_path)
        else:
            if 'last_saved' not in backup_meta_dict or backup_meta_dict['last_saved']['file_system_path'] is None:
                continue
            path = backup_meta_dict['last_saved']['file_system_path']
            if not os.path.isdir(path):
                logger.warning("The tab can not be open. The backup of tab {0} from common path {1} was not "
                               "possible.".format(idx, path))
                continue
            # logger.info("backup from last saved", path, sm_meta_dict)
            state_machine = storage.load_state_machine_from_path(path)
            state_machine_manager_model.state_machine_manager.add_state_machine(state_machine)

    global_runtime_config.extend_recently_opened_by_current_open_state_machines()

    # restore backup-ed selection
    session_selection = global_runtime_config.get_config_value('selection', None)
    if session_selection and session_selection['state_machine_page_number']:
        selected_state_machine_page_number = session_selection['state_machine_page_number']
        if selected_state_machine_page_number is None:
            return
        state_machines_editor_ctrl = main_window_controller.get_controller('state_machines_editor_ctrl')
        if not state_machines_editor_ctrl.view['notebook'].get_n_pages() >= selected_state_machine_page_number:
            logger.warning("Page id {0} does not exist so session restore can not re-create selection."
                           "".format(selected_state_machine_page_number))
            return
        notebook = state_machines_editor_ctrl.view['notebook']
        page = state_machines_editor_ctrl.on_switch_page(notebook, None, selected_state_machine_page_number)
        selected_sm_id = state_machine_manager_model.selected_state_machine_id
        if not selected_sm_id == state_machines_editor_ctrl.get_state_machine_id_for_page(page):
            logger.warning("Selection was not set correctly so session restore can not re-create selection.")
            return
        selected_sm_m = state_machine_manager_model.get_selected_state_machine_model()
        selected_model_set = []
        if global_gui_config.get_config_value('GAPHAS_EDITOR'):
            import gtk
            while gtk.events_pending():
                gtk.main_iteration(False)
        for core_element_identifier in session_selection['selection_of_selected_sm']:
            selected_model_set.append(selected_sm_m.get_state_model_by_path(core_element_identifier))
        selected_sm_m.selection.set(selected_model_set)
