"""
.. module:: config
   :platform: Unix, Windows
   :synopsis: Configuration for runtime parameters, such as window size and position

.. moduleauthor:: Franz Steinmetz


"""

from awesome_tool.utils.config import DefaultConfig
from awesome_tool.utils import log
logger = log.get_logger(__name__)

CONFIG_FILE = "runtime_config.yaml"


class RuntimeConfig(DefaultConfig):
    """
    Class to hold and load the runtime configuration
    """

    def __init__(self):
        super(RuntimeConfig, self).__init__("")

    def load(self, config_file=None, path=None):
        if config_file is None:
            config_file = CONFIG_FILE
        super(RuntimeConfig, self).load(config_file, path)

# This variable holds the global configuration parameters for the runtime parameters
global_runtime_config = RuntimeConfig()