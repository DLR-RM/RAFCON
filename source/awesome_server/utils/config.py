import yaml
import os

from awesome_server.utils.storage_utils import StorageUtils
from awesome_server.utils import helper
from awesome_server.utils import log
logger = log.get_logger(__name__)


CONFIG_PATH = os.getenv("HOME") + "/.awesome_server"

DEFAULT_CONFIG = """
TYPE: SERVER_CONFIG

TCP_PORT: 8888
UDP_PORT: 9999

NUMBER_UDP_MESSAGES_SENT: 10
NUMBER_UDP_MESSAGES_HISTORY: 100

NUMBER_OF_UDP_PACKAGES_UNTIL_TIMEOUT: 15
SECONDS_BETWEEN_UDP_RESEND: 2

HTML_SERVER_PORT: 8889
"""
CONFIG_FILE = "server_config.yaml"


class ServerConfig(object):
    """
    Class to hold and load the global configurations.
    """

    def __init__(self):

        sm_path, network_path = helper.get_opt_path()

        self.storage = StorageUtils("~/")
        if network_path and self.storage.exists_path(network_path):
            self.__config_dict = self.storage.load_dict_from_yaml(network_path)
            logger.info("Config initialized ... loaded configuration from %s" % str(network_path))
            return
        elif network_path:
            logger.info("Config file at path %s not found. Using default config instead" % str(network_path))

        if not self.storage.exists_path(os.path.join(CONFIG_PATH, CONFIG_FILE)):
            self.storage.create_path(CONFIG_PATH)
            open(os.path.join(CONFIG_PATH, CONFIG_FILE), "a").close()
            yaml_dict = yaml.load(DEFAULT_CONFIG)
            self.storage.write_dict_to_yaml(yaml_dict, os.path.join(CONFIG_PATH, CONFIG_FILE))
        self.__config_dict = self.storage.load_dict_from_yaml(os.path.join(CONFIG_PATH, CONFIG_FILE))

        if self.get_config_value("TYPE") != "SERVER_CONFIG":
            raise ConfigError("Type should be SERVER_CONFIG for Server configuration. "
                              "Please add \"TYPE: SERVER_CONFIG\" to your server_config.yaml file.")

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
        self.storage.write_dict_to_yaml(self.__config_dict, os.path.join(CONFIG_PATH, self.CONFIG_FILE),
                                        width=80, default_flow_style=False)
        logger.info("Saved configuration to filesystem (path: %s)" % str(os.path.join(CONFIG_PATH, self.CONFIG_FILE)))


class ConfigError(Exception):
    """
    Exception raised for errors loading the config files
    """
    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return repr(self.msg)


global_server_config = ServerConfig()