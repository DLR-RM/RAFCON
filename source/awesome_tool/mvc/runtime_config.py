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

    def save_configuration(self, main_window_view):
        main_window = main_window_view.get_top_widget().get_window()
        size = main_window.get_size()
        position = main_window.get_position()
        # screen = main_window.get_screen()
        # logger.debug("Main window screen:, {0}".format(screen))
        logger.debug("Main window size: {0}".format(size))
        logger.debug("Main window position: {0}".format(position))
        self.set_config_value('WINDOW_SIZE', size)
        self.set_config_value('WINDOW_POS', position)
        super(RuntimeConfig, self).save_configuration()

# This variable holds the global configuration parameters for the runtime parameters
global_runtime_config = RuntimeConfig()