import os

from rafcon.utils.storage_utils import StorageUtils
from rafcon.utils import log
logger = log.get_logger(__name__)


class NetworkStorageReader:

    def __init__(self, base_path):
        self.storage_utils = StorageUtils(base_path)
        self._base_path = None
        self.base_path = os.path.abspath(base_path)
        self.sm_path, self.sm_name = os.path.split(self.base_path)
        logger.debug("Network storage reader initialized")

        self.file_storage = {}

        self.load_statemachine_files()

    def load_statemachine_files(self, path=None, base_path=None):
        if base_path is not None:
            self.base_path = base_path
        if path is None:
            path = self.base_path

        for p in os.listdir(path):
            if os.path.isdir(os.path.join(path, p)):
                self.load_statemachine_files(os.path.join(path, p))
            else:
                path_to_read = os.path.join(path, p)
                self.file_storage[path_to_read.replace(self.sm_path, "")] = open(path_to_read, 'r').read()

    @property
    def base_path(self):
        return self._base_path

    @base_path.setter
    def base_path(self, base_path):
        if not isinstance(base_path, str):
            raise TypeError("base_path must be of type str")

        self._base_path = base_path
        self.storage_utils.base_path = base_path