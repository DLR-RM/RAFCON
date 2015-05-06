import yaml
import os
import getopt
import sys

from awesome_tool.utils.storage_utils import StorageUtils
from awesome_tool.utils import log
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
"""
CONFIG_FILE = "server_config.yaml"


class ServerConfig(object):
    """
    Class to hold and load the global configurations.
    """

    def __init__(self):

        opt_path = self.get_opt_path()

        self.storage = StorageUtils("~/")
        if opt_path and self.storage.exists_path(opt_path):
            self.__config_dict = self.storage.load_dict_from_yaml(opt_path)
            logger.info("Config initialized ... loaded configuration from %s" % str(opt_path))
            return
        elif opt_path:
            logger.info("Config file at path %s not found. Using default config instead" % str(opt_path))

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
        self.storage.write_dict_to_yaml(self.__config_dict, os.path.join(CONFIG_PATH, self.CONFIG_FILE))
        logger.info("Saved configuration to filesystem (path: %s)" % str(os.path.join(CONFIG_PATH, self.CONFIG_FILE)))

    @staticmethod
    def get_opt_path():
        server_path = None

        try:
            opts, args = getopt.getopt(sys.argv[1:], "hs:", ["server=", ])
        except getopt.GetoptError:
            print '%s -s <path/to/server_config>' % sys.executable
            sys.exit(2)
        for opt, arg in opts:
            if opt == '-h':
                print '%s -s <path/to/server_config>' % sys.executable
                sys.exit()
            elif opt in ("-s", "--server"):
                server_path = arg

        return server_path


class ConfigError(Exception):
    """
    Exception raised for errors loading the config files
    """
    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return repr(self.msg)


global_server_config = ServerConfig()