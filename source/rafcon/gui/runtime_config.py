# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: config
   :synopsis: Configuration for runtime parameters, such as window size and position

"""
import os
import gtk

from rafcon.core.config import ObservableConfig

from rafcon.core.storage import storage
from rafcon.utils import log, storage_utils, filesystem
logger = log.get_logger(__name__)

CONFIG_FILE = "runtime_config.yaml"


class RuntimeConfig(ObservableConfig):
    """Class to hold and load the runtime configuration"""

    def __init__(self):
        super(RuntimeConfig, self).__init__("")

    def load(self, config_file=None, path=None):
        if config_file is None:
            config_file = CONFIG_FILE
        super(RuntimeConfig, self).load(config_file, path)

    def store_widget_properties(self, widget, title):
        """Sets configuration values for widgets

        If the widget is a window, then the size and position are stored. If the widget is a pane, then only the
        position is stored. If the window is maximized the last insert position before being maximized is keep in the
        config and the maximized flag set to True. The maximized state and the last size and position are strictly
        separated by this.

        :param widget: The widget, for which the position (and possibly the size) will be stored.
        :param title: The title of the widget, which constitutes a part of its key in the configuration file.
        """
        if isinstance(widget, gtk.Window):
            maximized = bool(widget.maximize_initially)
            self.set_config_value('{0}_MAXIMIZED'.format(title), maximized)
            if maximized:
                return
            size = widget.get_size()
            self.set_config_value('{0}_SIZE'.format(title), size)
        position = widget.get_position()
        self.set_config_value('{0}_POS'.format(title), position)

    def save_configuration(self):
        # screen = main_window.get_screen()
        # logger.debug("Main window screen:, {0}".format(screen))

        # if the runtime_config was not loaded in some startup routine then load it explicitly (= create it)
        if not self.config_file_path:
            self.load()

        super(RuntimeConfig, self).save_configuration()

    def update_recently_opened_state_machines_with(self, state_machine_m):
        """ Update recently opened list with file system path of handed state machine model

        :param rafcon.gui.models.state_machine.StateMachineModel state_machine_m: State machine model to check
        :return:
        """
        sm = state_machine_m.state_machine
        if sm.file_system_path:
            # check if path is in recent path already
            # logger.info("update recent state machine: {}".format(sm.file_system_path))
            recently_opened_state_machines = self.get_config_value('recently_opened_state_machines', [])
            if sm.file_system_path in recently_opened_state_machines:
                del recently_opened_state_machines[recently_opened_state_machines.index(sm.file_system_path)]
            recently_opened_state_machines.insert(0, sm.file_system_path)
            self.set_config_value('recently_opened_state_machines', recently_opened_state_machines)

    def extend_recently_opened_by_current_open_state_machines(self):
        """ Update list with all in the state machine manager opened state machines
        """
        from rafcon.gui.singleton import state_machine_manager_model as state_machine_manager_m
        for sm_m in state_machine_manager_m.state_machines.itervalues():
            self.update_recently_opened_state_machines_with(sm_m)

    def prepare_recent_opened_state_machines_list_for_storage(self):
        """ Reduce number of paths in the recent opened state machines to limit from gui config
        """
        from rafcon.gui.singleton import global_gui_config
        num = global_gui_config.get_config_value('NUMBER_OF_RECENT_OPENED_STATE_MACHINES_STORED')
        state_machine_paths = self.get_config_value('recently_opened_state_machines', [])
        filesystem.clean_file_system_paths_from_not_existing_paths(state_machine_paths)
        self.set_config_value('recently_opened_state_machines', state_machine_paths[:num])

    def reset_session_storage(self):
        self.set_config_value('open_tabs', [])

    def store_session(self):
        """ Stores reference backup information for all open tabs into runtime config

        The backup of never stored tabs (state machines) and not stored state machine changes will be triggered a last
        time to secure data lose.
        """
        from rafcon.gui.singleton import state_machine_manager_model as state_machine_manager_m
        from rafcon.gui.models.auto_backup import AutoBackupModel
        # check if there are dirty state machines -> use backup file structure maybe it is already stored
        for sm_m in state_machine_manager_m.state_machines.itervalues():
            if hasattr(sm_m, 'auto_backup'):
                if sm_m.state_machine.marked_dirty:
                    sm_m.auto_backup.perform_temp_storage()
            else:
                # generate a backup
                sm_m.auto_backup = AutoBackupModel(sm_m)
        # store final state machine meta data to restore session in the next run
        list_of_tab_meta = [sm_m.auto_backup.meta for sm_m in state_machine_manager_m.state_machines.itervalues()]
        self.set_config_value('open_tabs', list_of_tab_meta)

    def restore_session_from_storage(self):
        """ Restore stored tabs from runtime config

        The method checks if the last status of a state machine is in the backup or in tis original path and loads it
        from there. The original path of these state machines are also insert into the recently opened state machines
        list.
        """
        # TODO add a dirty lock for a crashed rafcon instance also into restore session feature
        # TODO in case a dialog is needed to give the user control
        # TODO combine this and auto-backup in one structure/controller/observer
        from rafcon.gui.singleton import state_machine_manager_model as state_machine_manager_m
        from rafcon.gui.models.auto_backup import recover_state_machine_from_backup
        # check if session storage exists
        open_tabs = self.get_config_value('open_tabs', None)
        if open_tabs is None:
            logger.info("No session recovery from runtime config file")
            return

        # load and recover state machines like they were opened before
        for idx, sm_meta_dict in enumerate(open_tabs):
            from_backup_path = None
            # TODO do this decision before storing or maybe store the last stored time in the auto backup?!
            # pick folder name dependent on time, and backup meta data existence
            # problem is that the backup time is maybe not the best choice
            if 'last_backup' in sm_meta_dict:
                last_backup_time = storage_utils.get_float_time_for_string(sm_meta_dict['last_backup']['time'])
                if 'last_saved' in sm_meta_dict:
                    last_save_time = storage_utils.get_float_time_for_string(sm_meta_dict['last_saved']['time'])
                    backup_marked_dirty = sm_meta_dict['last_backup']['marked_dirty']
                    if last_backup_time > last_save_time and backup_marked_dirty:
                        from_backup_path = sm_meta_dict['last_backup']['file_system_path']
                else:
                    from_backup_path = sm_meta_dict['last_backup']['file_system_path']
            elif 'last_saved' in sm_meta_dict:
                # print "### open last saved", sm_meta_dict['last_saved']['file_system_path']
                pass
            else:
                logger.error("A tab was stored into session storage dictionary {0} without any recovery path"
                             "".format(sm_meta_dict))
                continue

            # check in case that the backup folder is valid or use last saved path
            if from_backup_path is not None and not os.path.isdir(from_backup_path):
                logger.warning("The restore of tab {0} from backup path {1} was not possible. "
                               "The last saved path will be used what maybe include lose of changes."
                               "".format(idx, from_backup_path))
                from_backup_path = None

            # open state machine
            if from_backup_path is not None:
                # open state machine, recover mark dirty flags, cleans dirty lock files
                logger.info("Recover from backup {0}".format(from_backup_path))
                recover_state_machine_from_backup(from_backup_path)
            else:
                if 'last_saved' not in sm_meta_dict or sm_meta_dict['last_saved']['file_system_path'] is None:
                    continue
                path = sm_meta_dict['last_saved']['file_system_path']
                if not os.path.isdir(path):
                    logger.warning("The tab can not be open. The restore of tab {0} from common path {1} was not "
                                   "possible.".format(idx, path))
                    continue
                # logger.info("restore from last saved", path, sm_meta_dict)
                state_machine = storage.load_state_machine_from_path(path)
                state_machine_manager_m.state_machine_manager.add_state_machine(state_machine)

        self.extend_recently_opened_by_current_open_state_machines()


# This variable holds the global configuration parameters for the runtime parameters
global_runtime_config = RuntimeConfig()
