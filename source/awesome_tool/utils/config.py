import yaml
import os

from awesome_tool.utils.storage_utils import StorageUtils
from awesome_tool.utils import log
logger = log.get_logger(__name__)


class DefaultConfig(object):
    """
    Class to hold and load the global configurations.
    """

    def __init__(self, default_config):
        assert isinstance(default_config, str)
        self.config_file = None
        self.default_config = default_config

        self.__config_dict = yaml.load(self.default_config)

    def load(self, config_file, opt_path=None):
        assert isinstance(config_file, str)

        self.storage = StorageUtils("~/")

        if not opt_path or not os.path.exists(opt_path):
            logger.warn('No configuration found, using temporary default config')
            return

        config_file_path = os.path.join(opt_path, config_file)

        if not os.path.exists(config_file_path):
            try:
                os.makedirs(config_file_path)
                self.storage.write_dict_to_yaml(self.__config_dict, config_file_path, width=80, default_flow_style=False)
                self.config_file = config_file_path
                logger.debug("Created config file {0}".format(config_file_path))
            except Exception as e:
                logger.error('Could not write to config {0}, using temporary default configuration. '
                             'Error: {1}'.format(config_file_path, e))
        else:
            try:
                self.__config_dict = self.storage.load_dict_from_yaml(config_file_path)
                self.config_file = config_file_path
                logger.debug("Configuration loaded from {0}".format(config_file_path))
            except Exception as e:
                logger.error('Could not read from config {0}, using temporary default configuration. '
                             'Error: {1}'.format(config_file_path, e))

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