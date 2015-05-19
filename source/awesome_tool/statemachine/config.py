"""
.. module:: config
   :platform: Unix, Windows
   :synopsis: Config module to specify global constants

.. moduleauthor:: Sebastian Brunner


"""
from awesome_tool.utils.config import DefaultConfig, ConfigError
from awesome_tool.utils import helper
from awesome_tool.utils import log
logger = log.get_logger(__name__)


DEFAULT_CONFIG = """

TYPE: SM_CONFIG

LIBRARY_PATHS: {"generic": "../../libraries/generic",
                "test_libraries": "../../test_scripts/test_libraries",
                 "ros_libraries": "../../test_scripts/ros_libraries",
                 "turtle_libraries": "../../test_scripts/turtle_libraries"}
"""

CONFIG_FILE = "config.yaml"


class Config(DefaultConfig):
    """
    Class to hold and load the global statemachine configurations.
    """

    def __init__(self):
        super(Config, self).__init__(DEFAULT_CONFIG)
        if self.get_config_value("TYPE") != "SM_CONFIG":
            raise ConfigError("Type should be SM_CONFIG for statemachine configuration. "
                              "Please add \"TYPE: SM_CONFIG\" to your config.yaml file.")

    def load(self, path=None):
        super(Config, self).load(CONFIG_FILE, path)

# This variable holds the global configuration parameters for the statemachine
global_config = Config()