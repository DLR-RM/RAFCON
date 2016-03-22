"""
.. module:: config
   :platform: Unix, Windows
   :synopsis: Config module to specify global constants

.. moduleauthor:: Sebastian Brunner


"""
from os import path

from rafcon.utils.config import DefaultConfig, ConfigError
from rafcon.utils import filesystem
from rafcon.utils import log

logger = log.get_logger(__name__)

CONFIG_FILE = "network_config.yaml"
# DEFAULT is a string and equals the content of the ./network_config.yaml
DEFAULT_CONFIG = filesystem.read_file(path.dirname(__file__), CONFIG_FILE)


class Config(DefaultConfig):
    """
    Class to hold and load the global network configurations.
    """

    def __init__(self):
        super(Config, self).__init__(DEFAULT_CONFIG)
        self.load(CONFIG_FILE)
        if self.get_config_value("TYPE") != "NETWORK_CONFIG":
            raise ConfigError("Type should be NETWORK_CONFIG for network configuration. "
                              "Please add \"TYPE: NETWORK_CONFIG\" to your network_config.yaml file.")

    def load(self, config_file=None, path=None):
        if config_file is None:
            config_file = CONFIG_FILE
        super(Config, self).load(config_file, path)


# This variable holds the global configuration parameters for the statemachine
global_network_config = Config()
