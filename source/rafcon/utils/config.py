import yaml
import os
import argparse
from os.path import realpath, dirname, join, exists, expanduser, expandvars, isdir

from rafcon.utils.storage_utils import StorageUtils
from rafcon.utils import log
logger = log.get_logger(__name__)


def read_file(path, filename):
    file_path = os.path.join(os.path.realpath(path), filename)

    file_content = ""
    if os.path.isfile(file_path):
        file_pointer = open(file_path, 'r')
        file_content = file_pointer.read()
        file_pointer.close()

    return file_content


def config_path(path):
    if not path or path == 'None':
        return None
    # replace ~ with /home/user
    path = expanduser(path)
    # e.g. replace ${RAFCON_PATH} with the root path of RAFCON
    path = expandvars(path)
    if not isdir(path):
        raise argparse.ArgumentTypeError("{0} is not a valid path".format(path))
    if os.access(path, os.R_OK):
        return path
    else:
        raise argparse.ArgumentTypeError("{0} is not a readable dir".format(path))


class DefaultConfig(object):
    """
    Class to hold and load the global configurations.
    """

    def __init__(self, default_config):
        assert isinstance(default_config, str)
        self.config_file = None
        self.default_config = default_config
        self.storage = StorageUtils()
        self.path = None

        if not default_config:
            self.__config_dict = {}
        else:
            self.__config_dict = yaml.load(self.default_config)

    def load(self, config_file, path=None):
        assert isinstance(config_file, str)

        if path is None:
            path = os.path.join(os.path.expanduser('~'), '.config', 'rafcon')

        if not os.path.exists(path):
            logger.warn('No configuration found, using temporary default config and create path on file system.')
            os.makedirs(path)

        config_file_path = os.path.join(path, config_file)

        # If no config file is found, create one in the desired directory
        if not os.path.isfile(config_file_path):
            try:
                if not os.path.exists(path):
                    os.makedirs(path)
                self.storage.write_dict_to_yaml(self.__config_dict, config_file_path, width=80, default_flow_style=False)
                self.config_file = config_file_path
                logger.debug("Created config file {0}".format(config_file_path))
            except Exception as e:
                logger.error('Could not write to config {0}, using temporary default configuration. '
                             'Error: {1}'.format(config_file_path, e))
        # Otherwise read the config file from the specified directory
        else:
            try:
                self.__config_dict = self.storage.load_dict_from_yaml(config_file_path)
                self.config_file = config_file_path
                logger.debug("Configuration loaded from {0}".format(config_file_path))
            except Exception as e:
                logger.error('Could not read from config {0}, using temporary default configuration. '
                             'Error: {1}'.format(config_file_path, e))

        self.path = path

    def get_config_value(self, key, default=None):
        """
        Get a specific configuration value
        :param key: the key to the configuration value
        :param default: what to return if the key is not found
        :return:
        """
        if key in self.__config_dict:
            return self.__config_dict[key]
        return default

    def set_config_value(self, key, value):
        """
        Get a specific configuration value
        :param key: the key to the configuration value
        :return:
        """
        self.__config_dict[key] = value

    def save_configuration(self):
        if self.config_file:
            self.storage.write_dict_to_yaml(self.__config_dict, self.config_file, width=80, default_flow_style=False)
            logger.debug("Saved configuration to {0}".format(self.config_file))


class ConfigError(Exception):
    """
    Exception raised for errors loading the config files
    """
    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return repr(self.msg)