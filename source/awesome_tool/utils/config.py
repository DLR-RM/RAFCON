import yaml
import os

from awesome_tool.utils.storage_utils import StorageUtils
from awesome_tool.utils import log
logger = log.get_logger(__name__)


CONFIG_PATH = os.getenv("HOME") + "/.awesome_tool"


class DefaultConfig(object):
    """
    Class to hold and load the global configurations.
    """

    DEFAULT_CONFIG = None
    CONFIG_FILE = None

    def __init__(self, config_file, default_config, opt_path=None):
        assert isinstance(config_file, str)
        assert isinstance(default_config, str)
        self.CONFIG_FILE = config_file
        self.DEFAULT_CONFIG = default_config

        self.storage = StorageUtils("~/")
        if opt_path and self.storage.exists_path(opt_path):
            self.__config_dict = self.storage.load_dict_from_yaml(opt_path)
            logger.info("Config initialized ... loaded configuration from %s" % str(opt_path))
            return
        elif opt_path:
            logger.info("Config file at path %s not found. Using default config instead" % str(opt_path))

        if not self.storage.exists_path(os.path.join(CONFIG_PATH, self.CONFIG_FILE)):
            self.storage.create_path(CONFIG_PATH)
            open(os.path.join(CONFIG_PATH, self.CONFIG_FILE), "a").close()
            yaml_dict = yaml.load(self.DEFAULT_CONFIG)
            self.storage.write_dict_to_yaml(yaml_dict, os.path.join(CONFIG_PATH, self.CONFIG_FILE))
        self.__config_dict = self.storage.load_dict_from_yaml(os.path.join(CONFIG_PATH, self.CONFIG_FILE))
        logger.info("Config initialized ... loaded configuration from %s" % str(os.path.join(CONFIG_PATH, self.CONFIG_FILE)))

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
        self.storage.write_dict_to_yaml(self.__config_dict, os.path.join(CONFIG_PATH, self.CONFIG_FILE))
        logger.info("Saved configuration to filesystem (path: %s)" % str(os.path.join(CONFIG_PATH, self.CONFIG_FILE)))


class ConfigError(Exception):
    """
    Exception raised for errors loading the config files
    """
    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return repr(self.msg)