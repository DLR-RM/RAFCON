import os
import datetime
import shelve
import subprocess
from threading import Lock

from rafcon.core.config import global_config
from rafcon.core.execution.consumers.abstract_execution_history_consumer import AbstractExecutionHistoryConsumer
from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE
from rafcon.utils import log
logger = log.get_logger(__name__)


class FileSystemConsumer(AbstractExecutionHistoryConsumer):
    """ A class that consumes an execution history event and writes it onto the file system.
    """
    def __init__(self, root_state_name):
        super(FileSystemConsumer, self).__init__()
        self.destroyed = False
        self.filename = self._get_storage_path_on_file_system(root_state_name)
        self.store_lock = Lock()

    def register(self):
        """ Open the shelve file
        """
        try:
            # 'c' for read/write/create
            # protocol 2 cause of in some cases smaller file size
            # writeback disabled, cause we don't need caching of entries in memory but continuous writes to the disk
            self.store = shelve.open(self.filename, flag='c', protocol=2, writeback=False)
            logger.debug('Openend log file for writing %s' % self.filename)
        except Exception:
            logger.exception('Exception:')

    def consume(self, execution_history_item):
        """ Write to the store variable corresponding to a shelve file
        """
        self._store_item(execution_history_item.history_item_id, execution_history_item.to_dict())

    def unregister(self):
        """ Flush & close the shelve file
        """
        set_read_and_writable_for_all = global_config.get_config_value("EXECUTION_LOG_SET_READ_AND_WRITABLE_FOR_ALL",
                                                                       False)
        self._flush()
        self._close(set_read_and_writable_for_all)

    @staticmethod
    def _get_storage_path_on_file_system(root_state_name):
        """ Get the shelve file of a specific state machine

        :param root_state_name: the root name
        """
        base_dir = global_config.get_config_value("EXECUTION_LOG_PATH", "%RAFCON_TEMP_PATH_BASE/execution_logs")
        if base_dir.startswith('%RAFCON_TEMP_PATH_BASE'):
            base_dir = base_dir.replace('%RAFCON_TEMP_PATH_BASE', RAFCON_TEMP_PATH_BASE)
        if not os.path.exists(base_dir):
            os.makedirs(base_dir)
        shelve_name = os.path.join(base_dir, '%s_rafcon_execution_log_%s.shelve' %
                                   (str(datetime.datetime.now()),
                                    root_state_name.replace(' ', '-')))
        return shelve_name

    def _store_item(self, key, value):
        """ Flush & close the shelve file
        """
        with self.store_lock:
            try:
                self.store[key] = value
            except Exception:
                logger.exception('Exception:')

    def _flush(self):
        """ Flush the shelve file
        """
        with self.store_lock:
            try:
                self.store.close()
                self.store = shelve.open(self.filename, flag='c', protocol=2, writeback=False)
                logger.debug('Flushed log file %s' % self.filename)
            except Exception:
                if self.destroyed:
                    pass  # this is fine
                else:
                    logger.exception('Exception:')

    def _close(self, make_read_and_writable_for_all=False):
        """ Close the shelve file
        """
        with self.store_lock:
            try:
                self.store.close()
                logger.debug('Closed log file %s' % self.filename)
                if make_read_and_writable_for_all:
                    ret = subprocess.call(['chmod', 'a+rw', self.filename])
                    if ret:
                        logger.debug('Could not make log file readable for all. chmod a+rw failed on %s.' % self.filename)
                    else:
                        logger.debug('Set log file readable for all via chmod a+rw, file %s' % self.filename)
            except Exception:
                logger.exception('Exception:')

    def __del__(self):
        """ Close the shelve file
        """
        with self.store_lock:
            self.destroyed = True
            try:
                self.store.close()
                logger.debug('Closed log file %s' % self.filename)
            except Exception:
                logger.exception('Exception:')
