import time
import threading

from gtkmvc import ModelMT

from rafcon.mvc.config import global_gui_config
from rafcon.mvc.models.state_machine import StateMachineModel

from rafcon.statemachine.storage import storage

from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE
from rafcon.utils import log
logger = log.get_logger(__name__)


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
        self.observe_model(self.state_machine_model)
        self.observe_model(self.state_machine_model.history)

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

        logger.debug("The auto-backup for state-machine {2} is {0} and set to '{1}'"
                     "".format('ENABLED' if self.timed_temp_storage_enabled else 'DISABLED',
                               'fix interval mode' if self.only_fix_interval else 'dynamic interval mode',
                               self.state_machine_model.state_machine.state_machine_id))

    def __destroy__(self):
        self.destroy()

    def destroy(self):
        if self.tmp_storage_timed_thread is not None:
            self.tmp_storage_timed_thread.cancel()

    @ModelMT.observe("state_machine", after=True)
    def change_count_of_modification_history(self, model, prop_name, info):
        if info['method_name'] == 'marked_dirty':
            if not self.only_fix_interval:
                self.check_for_auto_backup()
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
        self.tmp_storage_timed_thread.start()

    def perform_temp_storage(self):
        if self.__perform_storage:
            # logger.debug("Do not perform storage, one is running!")
            return
        self.timer_request_lock.acquire()
        self.__perform_storage = True
        self.timer_request_lock.release()
        sm = self.state_machine_model.state_machine
        logger.info('Perform auto-backup of state-machine {} to tmp-folder'.format(sm.state_machine_id))
        if sm.file_system_path is None:
            tmp_sm_system_path = RAFCON_TEMP_PATH_BASE + '/runtime_backup/not_stored_' + str(sm.state_machine_id)
        else:
            tmp_sm_system_path = RAFCON_TEMP_PATH_BASE + '/runtime_backup/' + sm.file_system_path
        # logger.debug('acquire lock')
        self.state_machine_model.storage_lock.acquire()
        # logger.debug('got lock')
        storage.save_statemachine_to_path(sm, tmp_sm_system_path, delete_old_state_machine=False,
                                          save_as=True, temporary_storage=True)
        self.state_machine_model.store_meta_data(temp_path=tmp_sm_system_path)
        self.last_backup_time = time.time()  # used as 'last-backup' time
        self.timer_request_lock.acquire()
        self._timer_request_time = None
        self.timer_request_lock.release()
        self.tmp_storage_timed_thread = None
        self.__perform_storage = False
        self.marked_dirty = False
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
            # logger.info("adjust last_backup_time")
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
