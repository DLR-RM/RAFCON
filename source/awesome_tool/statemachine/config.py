"""
.. module:: config
   :platform: Unix, Windows
   :synopsis: Config module to specify global constants

.. moduleauthor:: Sebastian Brunner


"""
import yaml
import os

from awesome_tool.utils.storage_utils import StorageUtils
from awesome_tool.utils import log
logger = log.get_logger(__name__)


DEFAULT_CONFIG = """

STATE_ID_LENGTH: 6

LIBRARY_PATHS: {"test_libraries": "../../test_scripts/test_libraries",
                 "ros_libraries": "../../test_scripts/ros_libraries",
                 "turtle_libraries": "../../test_scripts/turtle_libraries"}

"""

CONFIG_PATH = os.getenv("HOME") + "/.awesome_tool"
CONFIG_FILE = "config.yaml"


class Config(object):
    """
    Class to hold and load the global configurations.
    """

    def __init__(self):
        self.storage = StorageUtils("~/")
        if not self.storage.exists_path(os.path.join(CONFIG_PATH, CONFIG_FILE)):
            self.storage.create_path(CONFIG_PATH)
            open(os.path.join(CONFIG_PATH, CONFIG_FILE), "a").close()
            yaml_dict = yaml.load(DEFAULT_CONFIG)
            self.storage.write_dict_to_yaml(yaml_dict, os.path.join(CONFIG_PATH, CONFIG_FILE))
        self.__config_dict = self.storage.load_dict_from_yaml(os.path.join(CONFIG_PATH, CONFIG_FILE))
        logger.info("Config initialized ... loaded configuration from %s" % str(os.path.join(CONFIG_PATH, CONFIG_FILE)))

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
        self.storage.write_dict_to_yaml(self.__config_dict, os.path.join(CONFIG_PATH, CONFIG_FILE))
        logger.info("Saved configuration to filesystem (path: %s)" % str(os.path.join(CONFIG_PATH, CONFIG_FILE)))

# This variable holds the global configuration parameters for the statemachine
global_config = Config()