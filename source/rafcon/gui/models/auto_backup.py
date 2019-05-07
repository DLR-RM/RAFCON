# Copyright (C) 2016-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from future import standard_library
standard_library.install_aliases()
from builtins import str
import os
import time
import threading

from gi.repository import Gtk
from gtkmvc3.model_mt import ModelMT

from rafcon.core.storage import storage
import rafcon.core.singleton as core_singletons

from rafcon.gui.config import global_gui_config
from rafcon.gui.models.state_machine import StateMachineModel
from rafcon.gui.utils.dialog import RAFCONCheckBoxTableDialog

from rafcon.gui.utils.constants import RAFCON_INSTANCE_LOCK_FILE_PATH
from rafcon.utils.vividict import Vividict
from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE
from rafcon.utils import log
from rafcon.utils.storage_utils import get_time_string_for_float
logger = log.get_logger(__name__)

FILE_NAME_AUTO_BACKUP = 'auto_backup.json'

MY_RAFCON_TEMP_PATH = str(os.path.sep).join(RAFCON_TEMP_PATH_BASE.split(os.path.sep)[:-1])
RAFCON_RUNTIME_BACKUP_PATH = os.path.join(RAFCON_TEMP_PATH_BASE, 'runtime_backup')
if not os.path.exists(RAFCON_RUNTIME_BACKUP_PATH):
    os.makedirs(RAFCON_RUNTIME_BACKUP_PATH)

try:
    import psutil
    process_id_list = [process.pid for process in psutil.process_iter()]
except (OSError, ImportError):
    logger.info(_("Could not retrieve list of current process ids"))
    process_id_list = []


def generate_rafcon_instance_lock_file():
    logger.debug(_("Generate lock file for RAFCON instance {0}".format(os.getpid())))
    file_handler = open(RAFCON_INSTANCE_LOCK_FILE_PATH, 'a+')
    file_handler.close()


def remove_rafcon_instance_lock_file():
    logger.debug(_("Remove lock file for RAFCON instance {0}".format(os.getpid())))
    if os.path.exists(RAFCON_INSTANCE_LOCK_FILE_PATH):
        os.remove(RAFCON_INSTANCE_LOCK_FILE_PATH)
    else:
        logger.warning(_("External remove of lock file detected!"))


def check_path_for_correct_dirty_lock_file(sm_path, path):
    for elem in os.listdir(path):
        full_path = os.path.join(path, elem)
        if not os.path.isdir(full_path) and 'dirty_lock_' in elem:
            with open(full_path) as f:
                if sm_path in f.readline().replace('\n', ''):
                    return full_path


def find_dirty_lock_file_for_state_machine_path(sm_path):
    full_path_dirty_lock = None
    # -> can be in the root tmp folder of the instance
    if MY_RAFCON_TEMP_PATH in sm_path:
        runtime_backup_path_len = len(MY_RAFCON_TEMP_PATH.split(os.sep)) + 2
        runtime_backup_path_of_rafcon_instance = os.sep.join(sm_path.split(os.sep)[:runtime_backup_path_len])
        assert 'runtime_backup' in runtime_backup_path_of_rafcon_instance
        full_path_dirty_lock = check_path_for_correct_dirty_lock_file(sm_path, runtime_backup_path_of_rafcon_instance)

    # -> or in the state machine folder
    if full_path_dirty_lock is None:
        full_path_dirty_lock = check_path_for_correct_dirty_lock_file(sm_path, sm_path)

    return full_path_dirty_lock


def move_dirty_lock_file(dirty_lock_file, sm_path):
    """ Move the dirt_lock file to the sm_path and thereby is not found by auto recovery of backup anymore """
    if dirty_lock_file is not None \
            and not dirty_lock_file == os.path.join(sm_path, dirty_lock_file.split(os.sep)[-1]):
        logger.debug("Move dirty lock from root tmp folder {0} to state machine folder {1}"
                     "".format(dirty_lock_file, os.path.join(sm_path, dirty_lock_file.split(os.sep)[-1])))
        os.rename(dirty_lock_file, os.path.join(sm_path, dirty_lock_file.split(os.sep)[-1]))


def recover_state_machine_from_backup(sm_path, pid=None, full_path_dirty_lock=None, with_gui_wait=False):

    if full_path_dirty_lock is None:
        full_path_dirty_lock = find_dirty_lock_file_for_state_machine_path(sm_path)
    # logger.info("found lock file to recover " + str(full_path_dirty_lock))

    # find last_save_file_system_path
    try:
        auto_backup_meta = storage.load_data_file(os.path.join(sm_path, FILE_NAME_AUTO_BACKUP))
    except ValueError:
        auto_backup_meta = {}
    last_save_file_system_path = None
    if 'last_saved' in auto_backup_meta and 'file_system_path' in auto_backup_meta['last_saved']:
        last_save_file_system_path = auto_backup_meta['last_saved']['file_system_path']
    elif pid is None:
        pass
    else:  # state machines with old backup format -> backward compatibility check
        reduced_path = sm_path.replace(os.path.join(MY_RAFCON_TEMP_PATH, pid, 'runtime_backup'), '')
        if os.path.isdir(reduced_path) and not reduced_path.split(os.path.sep)[1] == 'tmp':
            last_save_file_system_path = reduced_path

    # check if already open -> # TODO in future backups has to be integrated better to avoid this
    if last_save_file_system_path is not None \
            and core_singletons.state_machine_manager.is_state_machine_open(last_save_file_system_path):
        logger.info("Backup state machine is already open by other feature {0}".format(auto_backup_meta))
        move_dirty_lock_file(full_path_dirty_lock, sm_path)
        return

    state_machine = storage.load_state_machine_from_path(sm_path)

    # move dirty lock file
    move_dirty_lock_file(full_path_dirty_lock, sm_path)

    import rafcon.gui.singleton as gui_singletons
    gui_singletons.state_machine_manager.add_state_machine(state_machine)

    # TODO check this gui wait again
    # avoids that models are not generated and state machines are open without having the root state selected
    import rafcon.gui.utils
    rafcon.gui.utils.wait_for_gui()
    sm_m = gui_singletons.state_machine_manager_model.state_machines[state_machine.state_machine_id]
    assert sm_m.state_machine is state_machine

    # correct backup instance and sm-storage-path -> TODO make the add state machine better to reduce complexity, here
    # correct path after add state machine because meta data should be loaded from the backup path
    with sm_m.storage_lock:
        sm_m.state_machine._file_system_path = last_save_file_system_path

        # fix auto backup meta data
        if sm_m.auto_backup:
            if last_save_file_system_path is None:
                del sm_m.auto_backup.meta['last_saved']
            else:
                sm_m.auto_backup.meta['last_saved']['time'] = auto_backup_meta['last_saved']['time']
                sm_m.auto_backup.meta['last_saved']['file_system_path'] = sm_m.state_machine.file_system_path

    # set dirty flag -> TODO think about to make it more reliable still not fully sure that the flag is right
    # backward compatibility check
    if 'last_backup' in auto_backup_meta and 'marked_dirty' in auto_backup_meta['last_backup']:
        state_machine.marked_dirty = auto_backup_meta['last_backup']['marked_dirty']
    else:
        state_machine.marked_dirty = True  # backward compatibility

    return sm_m


def check_for_crashed_rafcon_instances():

    def on_message_dialog_response_signal(widget, response_id, found_backups, *args):

        if response_id == 1:
            for index, tuple_of_backup in enumerate(found_backups):
                path, pid, lock_file, m_time, full_path_dirty_lock = tuple_of_backup
                if path is not None and widget.list_store[index][0]:  # Open it

                    # recover state machine, marked dirty flag, marked for removal,
                    sm_m = recover_state_machine_from_backup(path, pid, full_path_dirty_lock)

                    # force backup re-initialization -> set auto-backup dirty flag to enforce backup even if not dirty
                    if sm_m is not None:
                        sm_m.auto_backup.marked_dirty = True
                        sm_m.auto_backup.check_for_auto_backup(force=True)

        if response_id in [1, 3]:
            for index, tuple_of_backup in enumerate(found_backups):
                path, pid, lock_file, m_time, full_path_dirty_lock = tuple_of_backup
                list_store_row = widget.list_store[index]
                # if open or delete is checked and the Apply or Ignore and remove button is pressed
                if (list_store_row[0] or list_store_row[2]) and response_id == 1 or response_id == 3:

                    if path is not None and response_id == 3:  # with path and Ignore and remove button is pressed
                        move_dirty_lock_file(full_path_dirty_lock, path)
                    if os.path.exists(os.path.join(MY_RAFCON_TEMP_PATH, pid, 'lock')):
                        logger.debug("Remove instance lock {0}".format(os.path.join(MY_RAFCON_TEMP_PATH, pid, 'lock')))
                        os.remove(os.path.join(MY_RAFCON_TEMP_PATH, pid, 'lock'))

        if response_id in [1, 2, 3]:
            widget.destroy()

    # find crashed RAFCON instances and not stored state machine with backups
    restorable_sm = []
    for folder in os.listdir(MY_RAFCON_TEMP_PATH):
        if not folder == str(os.getpid()) and folder not in process_id_list:
            rafcon_instance_path_to_check = os.path.join(MY_RAFCON_TEMP_PATH, folder)
            if os.path.isdir(rafcon_instance_path_to_check) and 'lock' in os.listdir(rafcon_instance_path_to_check):
                logger.info("There is tmp-data of a crashed/killed or badly closed state-machines of a RAFCON instance "
                            "in path: {}".format(rafcon_instance_path_to_check))
                restorable_sm.append((None, folder, None, None, None))

            runtime_backup_path_of_rafcon_instance = os.path.join(MY_RAFCON_TEMP_PATH, folder, 'runtime_backup')
            if os.path.exists(runtime_backup_path_of_rafcon_instance):
                for elem in os.listdir(runtime_backup_path_of_rafcon_instance):
                    full_path = os.path.join(runtime_backup_path_of_rafcon_instance, elem)
                    if not os.path.isdir(full_path) and 'dirty_lock_' in elem:
                        with open(full_path) as f:
                            path = f.readline().replace('\n', '')
                            # logger.info("{0} \n{1}".format(path, os.path.join(path, storage.STATEMACHINE_FILE)))
                            if os.path.isdir(path) and os.path.exists(os.path.join(path, storage.STATEMACHINE_FILE)):
                                logger.debug("Found restorable state machine from crashed instance {0} in path: {1}"
                                             "".format(folder, path))
                                modification_time = time.ctime(os.path.getmtime(os.path.join(MY_RAFCON_TEMP_PATH, folder)))
                                restorable_sm.append((path, folder, elem, modification_time, full_path))
                            else:
                                logger.warning("dirty_lock file without consistent state machine path '{}'!"
                                               "".format(full_path))
                                os.remove(full_path)

    # if restorable_sm:
    #     print("Restorable state machines: \n" + '\n'.join([elem[0] for elem in restorable_sm if elem[0] is not None]))

    if restorable_sm and any([path is not None for path, pid, lock_file, m_time, full_path_dirty_lock in restorable_sm]):
        message_string = "State machines of not correctly closed RAFCON instances have been found.\n\n" \
                         "This check and dialog can be disabled by setting 'AUTO_RECOVERY_CHECK': False " \
                         "in the GUI configuration file.\n\n" \
                         "The following state machines have been modified and not saved: \n"

        table_header = ["Open", "Decide Later", "Delete", "Last modified", "System path"]
        table_data = [(True if elem[0] is not None else False, False, False if elem[0] is not None else True,
                       str(elem[3]) if elem[0] is not None else "instance with pid: {0}".format(elem[1]),
                       str(elem[0]) if elem[0] is not None else "", elem) for elem in restorable_sm]

        def on_toggled(cell, path, column_id):
            other_ids = [0, 1, 2]
            other_ids.remove(column_id)
            system_path = dialog.list_store[path][5][0]
            if system_path is not None:
                dialog.list_store[path][column_id] = False if cell.get_active() else True
                for other_id in other_ids:
                    dialog.list_store[path][other_id] = False
            else:
                logger.info("Those lock is removed anytime because for instances without state machine lock "
                            "there is no recovery procedure, for now.")

        import rafcon.gui.singleton as gui_singletons
        dialog = RAFCONCheckBoxTableDialog(message_string,
                                           button_texts=("Apply", "Remind me Later.", "Ignore -> Remove all Notifications/Locks."),
                                           callback=on_message_dialog_response_signal, callback_args=[restorable_sm],
                                           table_header=table_header, table_data=table_data, toggled_callback=on_toggled,
                                           message_type=Gtk.MessageType.QUESTION,
                                           parent=gui_singletons.main_window_controller.view.get_top_widget(),
                                           width=800, standalone=False)
        dialog.activate()

    return restorable_sm


class AutoBackupModel(ModelMT):
    """ Class provides auto backup functionality for a state-machine.

    The Class AutoBackupModel requests a threading.Lock object named storage_lock in the StateMachineModel handed to it
    to avoid inconsistencies if storing in the middle of a modification by API or e.g. ModificationHistory.
    The auto-backup class can be initiated but be disabled by TIMED_TEMPORARY_STORAGE_ENABLED in the gui_config.yaml.
    There are two mode to run this class -- with fix interval checks for backup or by dynamical auto backup by setting
    the flag ONLY_FIX_FORCED_TEMPORARY_STORAGE_INTERVAL in the gui_config.yaml.
    The forced interval FORCED_TEMPORARY_STORAGE_INTERVAL is used for the fix auto backup interval and as the forced
    time interval for the dynamic auto backup. The dynamic auto backup will backup additionally if the user was not
    doing any modifications till a time horizon of TIMED_TEMPORARY_STORAGE_INTERVAL.
    The flag AUTO_RECOVERY_CHECK enables the check on not cleanly closed instances and state machines what only can be
    performed if the AUTO_RECOVERY_LOCK_ENABLED is set True to write respective lock files into the backup folders.
    If the lock file is not cleaned up the state machine and the RAFCON instance was not closed cleanly.

    """
    _tmp_storage_path = None

    def __init__(self, state_machine_model):
        ModelMT.__init__(self)

        assert isinstance(state_machine_model, StateMachineModel)
        self.state_machine_model = state_machine_model

        # variables used for lock files
        # TODO reduce those variables
        self.__destroyed = False
        self.AUTO_RECOVERY_LOCK_ENABLED = False
        if os.path.exists(os.path.join(RAFCON_TEMP_PATH_BASE, 'lock')) and \
                global_gui_config.get_config_value('AUTO_RECOVERY_LOCK_ENABLED'):
            self.AUTO_RECOVERY_LOCK_ENABLED = True
        self.lock_file_lock = threading.Lock()
        self.lock_file = None
        self.last_lock_file_name = None

        # general auto-backup variable
        self.timed_temp_storage_enabled = global_gui_config.get_config_value('AUTO_BACKUP_ENABLED')
        self.only_fix_interval = global_gui_config.get_config_value('AUTO_BACKUP_ONLY_FIX_FORCED_INTERVAL')
        self.force_temp_storage_interval = global_gui_config.get_config_value('AUTO_BACKUP_FORCED_STORAGE_INTERVAL')
        self.timed_temp_storage_interval = global_gui_config.get_config_value('AUTO_BACKUP_DYNAMIC_STORAGE_INTERVAL')
        self.last_backup_time = time.time()         # used as 'last-backup' and 'last-modification-not-backup-ed' time
        self.marked_dirty = False
        self.__perform_storage = False
        self._timer_request_time = None
        self.timer_request_lock = threading.Lock()
        self.tmp_timed_storage_thread = None
        self.meta = Vividict()
        if state_machine_model.state_machine.file_system_path is not None:
            # logger.info("store meta data of {0} to {1}".format(self, meta_data_path))
            # data used for restore tabs -> (having the information to load state machines without loading them)
            self.meta['last_saved']['time'] = state_machine_model.state_machine.last_update
            self.meta['last_saved']['file_system_path'] = state_machine_model.state_machine.file_system_path

        logger.debug("The auto-backup for state-machine {2} is {0} and set to '{1}'"
                     "".format('ENABLED' if self.timed_temp_storage_enabled else 'DISABLED',
                               'fix interval mode' if self.only_fix_interval else 'dynamic interval mode',
                               self.state_machine_model.state_machine.state_machine_id))

        # register observer before initializing check loop
        self.observe_model(self.state_machine_model)
        # initializing check loop to fully initialize the model
        if not self.only_fix_interval:
            self.perform_temp_storage()
        else:
            self.check_for_auto_backup(force=True)

    def __destroy__(self):
        self.destroy()

    def destroy(self):
        self.cancel_timed_thread()
        if not core_singletons.shut_down_signal:
            self.clean_lock_file(True)

    def prepare_destruction(self):
        """Prepares the model for destruction

        Unregister itself as observer from the state machine and the root state
        """
        try:
            self.relieve_model(self.state_machine_model)
        except KeyError:  # Might happen if the observer was already unregistered
            pass
        self.cancel_timed_thread()

    def cancel_timed_thread(self):
        if self.tmp_timed_storage_thread is not None:
            self.tmp_timed_storage_thread.cancel()
            self.tmp_timed_storage_thread.join()
            self.tmp_timed_storage_thread = None

    def check_lock_file(self):
        if self.__destroyed:
            return
        sm = self.state_machine_model.state_machine
        if sm.marked_dirty and self.lock_file is None and self.AUTO_RECOVERY_LOCK_ENABLED:
            with self.lock_file_lock:
                # logger.info('create lock {0} -> path {1}'.format(sm.state_machine_id,
                #                                                  RAFCON_RUNTIME_BACKUP_PATH +
                #                                                  '/dirty_lock_' + str(sm.state_machine_id)))
                self.lock_file = open(RAFCON_RUNTIME_BACKUP_PATH + '/dirty_lock_' + str(sm.state_machine_id), 'a+')
                self.lock_file.write(self._tmp_storage_path + '\n')
                # TODO move this and the inverse functionality to one location (capsule)
                self.lock_file.close()
                self.last_lock_file_name = self.lock_file.name

    def clean_lock_file(self, final=False):
        if self.__destroyed:
            return
        if self.lock_file and self.AUTO_RECOVERY_LOCK_ENABLED:
            with self.lock_file_lock:
                if final:
                    self.__destroyed = True
                # logger.info('clean lock {}'.format(self.state_machine_model.state_machine.state_machine_id))
                if not self.lock_file.closed:
                    self.lock_file.close()
                if os.path.exists(self.lock_file.name):
                    os.remove(self.lock_file.name)
                self.lock_file = None

    def update_tmp_storage_path(self):
        sm = self.state_machine_model.state_machine
        if sm.file_system_path is None:
            self._tmp_storage_path = os.path.join(RAFCON_RUNTIME_BACKUP_PATH, 'not_stored_' + str(sm.state_machine_id))
        else:
            self._tmp_storage_path = RAFCON_RUNTIME_BACKUP_PATH + storage.clean_path_from_deprecated_naming(
                sm.file_system_path)

    def write_backup_meta_data(self):
        """Write the auto backup meta data into the current tmp-storage path"""
        auto_backup_meta_file = os.path.join(self._tmp_storage_path, FILE_NAME_AUTO_BACKUP)
        storage.storage_utils.write_dict_to_json(self.meta, auto_backup_meta_file)

    def update_last_backup_meta_data(self):
        """Update the auto backup meta data with internal recovery information"""
        self.meta['last_backup']['time'] = get_time_string_for_float(self.last_backup_time)
        self.meta['last_backup']['file_system_path'] = self._tmp_storage_path
        self.meta['last_backup']['marked_dirty'] = self.state_machine_model.state_machine.marked_dirty

    def update_last_sm_origin_meta_data(self):
        """Update the auto backup meta data with information of the state machine origin"""
        # TODO finally maybe remove this when all backup features are integrated into one backup-structure
        # data also used e.g. to backup tabs
        self.meta['last_saved']['time'] = self.state_machine_model.state_machine.last_update
        self.meta['last_saved']['file_system_path'] = self.state_machine_model.state_machine.file_system_path

    @ModelMT.observe("state_machine", after=True)
    def change_in_state_machine_notification(self, model, prop_name, info):
        if info['method_name'] == 'marked_dirty':
            if not self.only_fix_interval:
                self.check_for_auto_backup()
            if not self.state_machine_model.state_machine.marked_dirty:
                self.clean_lock_file()
            self.marked_dirty = self.state_machine_model.state_machine.marked_dirty
        if info['method_name'] == 'file_system_path' and not self.__perform_storage:
            # logger.info("update last time stored")
            self.update_last_sm_origin_meta_data()
            self.write_backup_meta_data()

    def _check_for_dyn_timed_auto_backup(self):
        """ The method implements the timed storage feature.

         The method re-initiating a new timed thread if the state-machine not already stored to backup
         (what could be caused by the force_temp_storage_interval) or force the storing of the state-machine if there
         is no new request for a timed backup. New timed backup request are intrinsically represented by
         self._timer_request_time and initiated by the check_for_auto_backup-method.
         The feature uses only one thread for each ModificationHistoryModel and lock to be thread save.
        """
        current_time = time.time()
        with self.timer_request_lock:
            # sm = self.state_machine_model.state_machine
            # TODO check for self._timer_request_time is None to avoid and reset auto-backup in case and fix it better
            if self._timer_request_time is None:
                # logger.warning("timer_request is None")
                return
            if self.timed_temp_storage_interval < current_time - self._timer_request_time:
                # logger.info("{0} Perform timed auto-backup of state-machine {1}.".format(time.time(),
                #                                                                          sm.state_machine_id))
                self.check_for_auto_backup(force=True)
            else:
                duration_to_wait = self.timed_temp_storage_interval - (current_time - self._timer_request_time)
                hard_limit_duration_to_wait = self.force_temp_storage_interval - (current_time - self.last_backup_time)
                hard_limit_active = hard_limit_duration_to_wait < duration_to_wait
                # logger.info('{2} restart_thread {0} time to go {1}, hard limit {3}'.format(sm.state_machine_id,
                #                                                                            duration_to_wait, time.time(),
                #                                                                            hard_limit_active))
                if hard_limit_active:
                    self.set_timed_thread(hard_limit_duration_to_wait, self.check_for_auto_backup, True)
                else:
                    self.set_timed_thread(duration_to_wait, self._check_for_dyn_timed_auto_backup)

    def set_timed_thread(self, duration, func, *args):
        # logger.info("start timed thread duration: {0} func: {1}".format(duration, func))
        self.tmp_timed_storage_thread = threading.Timer(duration, func, args)
        self.tmp_timed_storage_thread.daemon = True
        self.tmp_timed_storage_thread.start()

    def perform_temp_storage(self):
        if self.__perform_storage:
            # logger.debug("Do not perform storage, one is running!")
            return
        # logger.debug('acquire lock')
        with self.state_machine_model.storage_lock, self.state_machine_model.state_machine.get_modification_lock():
            with self.timer_request_lock:
                self.__perform_storage = True
            sm = self.state_machine_model.state_machine
            logger.debug('Performing auto backup of state machine {} to temp folder'.format(sm.state_machine_id))
            self.update_tmp_storage_path()
            storage.save_state_machine_to_path(sm, self._tmp_storage_path, delete_old_state_machine=True, as_copy=True)
            self.update_last_backup_meta_data()
            self.write_backup_meta_data()
            self.state_machine_model.store_meta_data(copy_path=self._tmp_storage_path)
            self.last_backup_time = time.time()  # used as 'last-backup' time
            with self.timer_request_lock:
                self._timer_request_time = None
            self.tmp_timed_storage_thread = None
            self.__perform_storage = False
            self.marked_dirty = False
            self.check_lock_file()

    def check_for_auto_backup(self, force=False):
        """ The method implements the checks for possible auto backup of the state-machine according duration till
        the last change together with the private method _check_for_dyn_timed_auto_backup.

        If the only_fix_interval is True this function is called ones in the beginning and is called by a timed-
        threads in a fix interval.

        :param force: is a flag that force the temporary backup of the state-machine to the tmp-folder
        :return:
        """
        if not self.timed_temp_storage_enabled:
            return

        sm = self.state_machine_model.state_machine
        current_time = time.time()

        if not self.only_fix_interval and not self.marked_dirty:
            # logger.info("adjust last_backup_time " + str(sm.state_machine_id))
            self.last_backup_time = current_time         # used as 'last-modification-not-backup-ed' time

        is_not_timed_or_reached_time_to_force = \
            current_time - self.last_backup_time > self.force_temp_storage_interval or self.only_fix_interval

        if (sm.marked_dirty and is_not_timed_or_reached_time_to_force) or force:
            if not self.only_fix_interval or self.marked_dirty:
                thread = threading.Thread(target=self.perform_temp_storage)
                thread.start()
                # self.last_backup_time = current_time  # used as 'last-backup' time
            if self.only_fix_interval:
                self.set_timed_thread(self.force_temp_storage_interval, self.check_for_auto_backup)
        else:
            if not self.only_fix_interval:
                with self.timer_request_lock:
                    if self._timer_request_time is None:
                        # logger.info('{0} start_thread {1}'.format(current_time, sm.state_machine_id))
                        self._timer_request_time = current_time
                        self.set_timed_thread(self.timed_temp_storage_interval, self._check_for_dyn_timed_auto_backup)
                    else:
                        # logger.info('{0} update_thread {1}'.format(current_time, sm.state_machine_id))
                        self._timer_request_time = current_time
            else:
                self.set_timed_thread(self.force_temp_storage_interval, self.check_for_auto_backup)
