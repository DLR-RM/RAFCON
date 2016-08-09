import os
import time
import threading

import gtk
from gtkmvc import ModelMT

from rafcon.statemachine.storage import storage
import rafcon.statemachine.singleton as sm_singleton

from rafcon.mvc.config import global_gui_config
from rafcon.mvc.models.state_machine import StateMachineModel
from rafcon.mvc.utils.dialog import RAFCONDialog
import rafcon.mvc.singleton as mvc_singleton


from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE
from rafcon.utils import log
logger = log.get_logger(__name__)


MY_RAFCON_TEMP_PATH = str(os.path.sep).join(RAFCON_TEMP_PATH_BASE.split(os.path.sep)[:-1])
RAFCON_RUNTIME_BACKUP_PATH = os.path.join(RAFCON_TEMP_PATH_BASE, 'runtime_backup')
if not os.path.exists(RAFCON_RUNTIME_BACKUP_PATH):
    os.makedirs(RAFCON_RUNTIME_BACKUP_PATH)

from sys import platform as _platform
if _platform == "linux" or _platform == "linux2":
    import subprocess
    p = subprocess.Popen(['ps', '-A', '-o', 'pid'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = p.communicate()
    process_id_list = ''.join(out).replace(' ', '').replace('PID', '').split('\n')
else:
    process_id_list = []


def check_for_crashed_rafcon_instances():

    def on_message_dialog_response_signal(widget, response_id, found_backups):
        if response_id == 42:
            for path, pid, lock_file, m_time in found_backups:
                if path is not None:
                    state_machine = storage.load_state_machine_from_path(path)
                    mvc_singleton.state_machine_manager.add_state_machine(state_machine)
                    sm_m = mvc_singleton.state_machine_manager_model.state_machines[state_machine.state_machine_id]
                    assert sm_m.state_machine is state_machine

                    # correct backup instance and sm-storage-path
                    reduced_path = path.replace(os.path.join(MY_RAFCON_TEMP_PATH, pid, 'runtime_backup'), '')
                    sm_m.storage_lock.acquire()
                    if os.path.isdir(reduced_path) and not reduced_path.split(os.path.sep)[1] == 'tmp':
                        state_machine._file_system_path = reduced_path
                    else:
                        state_machine._file_system_path = None
                    sm_m.storage_lock.release()

                    # force backup re-initialization
                    state_machine.marked_dirty = True
                    sm_m.auto_backup.check_for_auto_backup(force=True)

        if response_id in [42, 44]:
            for path, pid, lock_file, m_time in found_backups:
                if path is None:
                    logger.debug("Clean up lock of RAFCON instance with pid {}".format(pid))
                else:
                    logger.debug("Clean up lock for state machine with path: {0} pid: {1} lock_file: {2}"
                                 "".format(path, pid, lock_file))
                if path is not None:
                    os.remove(os.path.join(MY_RAFCON_TEMP_PATH, pid, 'runtime_backup', lock_file))
                if os.path.exists(os.path.join(MY_RAFCON_TEMP_PATH, pid, 'lock')):
                    os.remove(os.path.join(MY_RAFCON_TEMP_PATH, pid, 'lock'))
        if response_id in [42, 43, 44]:
            widget.destroy()

    # find crashed RAFCON instances and not stored state machine with backups
    restorable_sm = []
    for folder in os.listdir(MY_RAFCON_TEMP_PATH):
        if not folder == str(os.getpid()) and folder not in process_id_list:
            rafcon_instance_path_to_check = os.path.join(MY_RAFCON_TEMP_PATH, folder)
            if os.path.isdir(rafcon_instance_path_to_check) and 'lock' in os.listdir(rafcon_instance_path_to_check):
                logger.info("There is tmp-data of a crashed/killed or badly closed state-machines of a RAFCON instance "
                            "in path: {}".format(rafcon_instance_path_to_check))
                restorable_sm.append((None, folder, None, None))

            runtime_backup_path_of_rafcon_instance = os.path.join(MY_RAFCON_TEMP_PATH, folder, 'runtime_backup')
            if os.path.exists(runtime_backup_path_of_rafcon_instance):
                for elem in os.listdir(runtime_backup_path_of_rafcon_instance):
                    full_path = os.path.join(runtime_backup_path_of_rafcon_instance, elem)
                    if not os.path.isdir(full_path) and 'dirty_lock_' in elem:
                        with open(full_path) as f:
                            path = f.readline()
                            # logger.info("{0} \n{1}".format(path, os.path.join(path, storage.STATEMACHINE_FILE)))
                            if os.path.isdir(path) and os.path.exists(os.path.join(path, storage.STATEMACHINE_FILE)):
                                logger.debug("Found restorable state machine from crashed instance {0} in path: {1}"
                                             "".format(folder, path))
                                modification_time = time.ctime(os.path.getmtime(os.path.join(MY_RAFCON_TEMP_PATH, folder)))
                                restorable_sm.append((path, folder, elem, modification_time))
                            else:
                                logger.warning("dirty_lock file without consistent state machine path {}!".format(full_path))
                                os.remove(full_path)

    # if restorable_sm:
    #     print "Restorable state machines: \n" + '\n'.join([elem[0] for elem in restorable_sm if elem[0] is not None])

    if restorable_sm and any([path is not None for path, pid, lock_file, m_time in restorable_sm]):
        dialog = RAFCONDialog(type=gtk.MESSAGE_WARNING, parent=mvc_singleton.main_window_controller.view.get_top_widget())
        message_string = "There have been found state machines of not correctly closed rafcon instances?\n\n" \
                         "The following state machines have been modified and not saved: \n" + \
                         "\n".join(["\t - modified: {} path: {}"
                                    "".format(elem[3], elem[0]) if elem[0] is not None else
                                    "- instance with pid: {}".format(elem[1]) for elem in restorable_sm]) + \
                         "\n\nThis check and dialog can be disabled by setting 'AUTO_RECOVERY_CHECK': False " \
                         "in the GUI configuration file."
        dialog.set_markup(message_string)
        hbox = dialog.get_action_area()
        vbox = hbox.parent
        msg_ctr = vbox.get_children()[0]
        text_ctr = msg_ctr.get_children()[1]
        text_ctr.get_children()[0].set_size_request(1400, -1)
        text_ctr.get_children()[1].set_size_request(1400, -1)
        dialog.add_button("Open all state machines.", 42)
        dialog.add_button("Remind me later.", 43)
        dialog.add_button("Ignore -> Remove all Notifications.", 44)
        dialog.finalize(on_message_dialog_response_signal, restorable_sm)

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
    """

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
        self.check_for_auto_backup(force=True)
        self.tmp_storage_timed_thread = None

        # register observer at the end after controller is fully initialized
        self.observe_model(self.state_machine_model)

        logger.debug("The auto-backup for state-machine {2} is {0} and set to '{1}'"
                     "".format('ENABLED' if self.timed_temp_storage_enabled else 'DISABLED',
                               'fix interval mode' if self.only_fix_interval else 'dynamic interval mode',
                               self.state_machine_model.state_machine.state_machine_id))

    def __destroy__(self):
        self.destroy()

    def destroy(self):
        logger.info('destroy auto backup ' + str(self.state_machine_model.state_machine.state_machine_id))
        self.cancel_timed_thread()
        if not sm_singleton.shut_down_signal:
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
        if self.tmp_storage_timed_thread is not None:
            self.tmp_storage_timed_thread.cancel()
            self.tmp_storage_timed_thread.join()
            self.tmp_storage_timed_thread = None

    def check_lock_file(self):
        if self.__destroyed:
            return
        sm = self.state_machine_model.state_machine
        if sm.marked_dirty and self.lock_file is None and self.AUTO_RECOVERY_LOCK_ENABLED:
            self.lock_file_lock.acquire()
            # logger.info('create lock {0} -> path {1}'.format(sm.state_machine_id, self.tmp_storage_folder()))
            self.lock_file = open(RAFCON_RUNTIME_BACKUP_PATH + '/dirty_lock_' + str(sm.state_machine_id), 'a+')
            self.lock_file.write(self.tmp_storage_folder())
            self.lock_file.close()
            self.last_lock_file_name = self.lock_file.name
            self.lock_file_lock.release()

    def clean_lock_file(self, final=False):
        if self.__destroyed:
            return
        if self.lock_file and self.AUTO_RECOVERY_LOCK_ENABLED:
            self.lock_file_lock.acquire()
            if final:
                self.__destroyed = True
            # logger.info('clean lock {}'.format(self.state_machine_model.state_machine.state_machine_id))
            if not self.lock_file.closed:
                self.lock_file.close()
            if os.path.exists(self.lock_file.name):
                os.remove(self.lock_file.name)
            self.lock_file = None
            self.lock_file_lock.release()

    def tmp_storage_folder(self):
        sm = self.state_machine_model.state_machine
        if sm.file_system_path is None:
            return os.path.join(RAFCON_RUNTIME_BACKUP_PATH, 'not_stored_' + str(sm.state_machine_id))
        else:
            return os.path.join(RAFCON_RUNTIME_BACKUP_PATH + sm.file_system_path)  # leave the PLUS !!!

    @ModelMT.observe("state_machine", after=True)
    def change_in_state_machine_notification(self, model, prop_name, info):
        if info['method_name'] == 'marked_dirty':
            if not self.only_fix_interval:
                self.check_for_auto_backup()
            if not self.state_machine_model.state_machine.marked_dirty:
                self.clean_lock_file()
            self.marked_dirty = self.state_machine_model.state_machine.marked_dirty

    def _check_for_dyn_timed_auto_backup(self):
        """ The method implements the timed storage feature.

         The method re-initiating a new timed thread if the state-machine not already stored to backup
         (what could be caused by the force_temp_storage_interval) or force the storing of the state-machine if there
         is no new request for a timed backup. New timed backup request are intrinsically represented by
         self._timer_request_time and initiated by the check_for_auto_backup-method.
         The feature uses only one thread for each ModificationHistoryModel and lock to be thread save.
        """
        actual_time = time.time()
        self.timer_request_lock.acquire()
        sm = self.state_machine_model.state_machine
        # TODO check for self._timer_request_time is None to avoid and reset auto-backup in case and fix it better
        # print str(self.timed_temp_storage_interval), str(actual_time), str(self._timer_request_time)
        if self._timer_request_time is None:
            # logger.warning("timer_request is None")
            return self.timer_request_lock.release()
        if self.timed_temp_storage_interval < actual_time - self._timer_request_time:
            # logger.info("{0} Perform timed auto-backup of state-machine {1}.".format(time.time(),
            #                                                                          sm.state_machine_id))
            self.check_for_auto_backup(force=True)
        else:
            duration_to_wait = self.timed_temp_storage_interval - (actual_time - self._timer_request_time)
            hard_limit_duration_to_wait = self.force_temp_storage_interval - (actual_time - self.last_backup_time)
            hard_limit_active = hard_limit_duration_to_wait < duration_to_wait
            # logger.info('{2} restart_thread {0} time to go {1}, hard limit {3}'.format(sm.state_machine_id,
            #                                                                            duration_to_wait, time.time(),
            #                                                                            hard_limit_active))
            if hard_limit_active:
                self.set_timed_thread(hard_limit_duration_to_wait, self.check_for_auto_backup, True)
            else:
                self.set_timed_thread(duration_to_wait, self._check_for_dyn_timed_auto_backup)
        self.timer_request_lock.release()

    def set_timed_thread(self, duration, func, *args):
        # logger.info("start timed thread duration: {0} func: {1}".format(duration, func))
        self.tmp_storage_timed_thread = threading.Timer(duration, func, args)
        self.tmp_storage_timed_thread.daemon = True
        self.tmp_storage_timed_thread.start()

    def perform_temp_storage(self):
        if self.__perform_storage:
            # logger.debug("Do not perform storage, one is running!")
            return
        # logger.debug('acquire lock')
        self.state_machine_model.storage_lock.acquire()
        # logger.debug('got lock')
        self.timer_request_lock.acquire()
        self.__perform_storage = True
        self.timer_request_lock.release()
        sm = self.state_machine_model.state_machine
        logger.info('Perform auto-backup of state-machine {} to tmp-folder'.format(sm.state_machine_id))
        tmp_sm_system_path = self.tmp_storage_folder()
        storage.save_state_machine_to_path(sm, tmp_sm_system_path, delete_old_state_machine=True,
                                           save_as=True, temporary_storage=True)
        self.state_machine_model.store_meta_data(temp_path=tmp_sm_system_path)
        self.last_backup_time = time.time()  # used as 'last-backup' time
        self.timer_request_lock.acquire()
        self._timer_request_time = None
        self.timer_request_lock.release()
        self.tmp_storage_timed_thread = None
        self.__perform_storage = False
        self.marked_dirty = False
        self.check_lock_file()
        self.state_machine_model.storage_lock.release()
        # logger.debug('released lock')

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
        actual_time = time.time()

        if not self.only_fix_interval and not self.marked_dirty:
            # logger.info("adjust last_backup_time " + str(sm.state_machine_id))
            self.last_backup_time = actual_time         # used as 'last-modification-not-backup-ed' time

        is_not_timed_or_reached_time_to_force = \
            actual_time - self.last_backup_time > self.force_temp_storage_interval or self.only_fix_interval

        if (sm.marked_dirty and is_not_timed_or_reached_time_to_force) or force:
            if not self.only_fix_interval or self.marked_dirty:
                thread = threading.Thread(target=self.perform_temp_storage)
                thread.start()
                # self.last_backup_time = actual_time  # used as 'last-backup' time
            if self.only_fix_interval:
                self.set_timed_thread(self.force_temp_storage_interval, self.check_for_auto_backup)
        else:
            if not self.only_fix_interval:
                self.timer_request_lock.acquire()
                if self._timer_request_time is None:
                    # logger.info('{0} start_thread {1}'.format(actual_time, sm.state_machine_id))
                    self._timer_request_time = actual_time
                    self.set_timed_thread(self.timed_temp_storage_interval, self._check_for_dyn_timed_auto_backup)
                else:
                    # logger.info('{0} update_thread {1}'.format(actual_time, sm.state_machine_id))
                    self._timer_request_time = actual_time
                self.timer_request_lock.release()
            else:
                self.set_timed_thread(self.force_temp_storage_interval, self.check_for_auto_backup)
