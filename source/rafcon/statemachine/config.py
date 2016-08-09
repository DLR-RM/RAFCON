"""
.. module:: config
   :platform: Unix, Windows
   :synopsis: Config module to specify global constants

.. moduleauthor:: Sebastian Brunner


"""
from os import path

from yaml_configuration.config import DefaultConfig, ConfigError
from rafcon.utils import filesystem
from rafcon.utils import log

logger = log.get_logger(__name__)

CONFIG_FILE = "config.yaml"

DEFAULT_CONFIG = filesystem.read_file(path.dirname(__file__), CONFIG_FILE)


class Config(DefaultConfig):
    """ Class to hold and load the global state machine configurations.

    """

    def __init__(self, logger_object=None):
        """
        Default constructor

        :param logger_object: the logger object to pass the log output to
        :raises ConfigError: if the config type is not given in the config file
        """
        super(Config, self).__init__(DEFAULT_CONFIG, logger_object)
        self.load(CONFIG_FILE)
        if self.get_config_value("TYPE") != "SM_CONFIG":
            raise ConfigError("Type should be SM_CONFIG for state machine configuration. "
                              "Please add \"TYPE: SM_CONFIG\" to your config.yaml file.")

    def load(self, config_file=None, path=None):
        """ Loads the configuration from a specific file

        :param config_file: the name of the config file
        :param path: the path to the config file
        """
        if config_file is None:
            config_file = CONFIG_FILE
        super(Config, self).load(config_file, path)


# This variable holds the global configuration parameters for the state machine
global_config = Config(logger)
