"""
.. module:: config
   :platform: Unix, Windows
   :synopsis: Configuration for runtime parameters, such as window size and position

.. moduleauthor:: Franz Steinmetz


"""
import gtk
from rafcon.utils.config import DefaultConfig
from rafcon.utils import log

logger = log.get_logger(__name__)

CONFIG_FILE = "runtime_config.yaml"


class RuntimeConfig(DefaultConfig):
    """Class to hold and load the runtime configuration"""

    def __init__(self):
        super(RuntimeConfig, self).__init__("")

    def load(self, config_file=None, path=None):
        if config_file is None:
            config_file = CONFIG_FILE
        super(RuntimeConfig, self).load(config_file, path)

    def save_configuration(self, widget, title):
        if isinstance(widget, gtk.Window):
            size = widget.get_size()
            logger.debug('{0} size: {1}'.format(title, size))
            self.set_config_value('{0}_SIZE'.format(title), size)
        position = widget.get_position()
        logger.debug('{0} position: {1}'.format(title, position))
        self.set_config_value('{0}_POS'.format(title), position)
        # screen = main_window.get_screen()
        # logger.debug("Main window screen:, {0}".format(screen))

        # if the runtime_config was not loaded in some startup routine then load it explicitly (= create it)
        if not self.config_file_path:
            self.load()

        super(RuntimeConfig, self).save_configuration()


# This variable holds the global configuration parameters for the runtime parameters
global_runtime_config = RuntimeConfig()
