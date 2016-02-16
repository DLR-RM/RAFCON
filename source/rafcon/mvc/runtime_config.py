"""
.. module:: config
   :platform: Unix, Windows
   :synopsis: Configuration for runtime parameters, such as window size and position

.. moduleauthor:: Franz Steinmetz


"""
import gtk
from rafcon.mvc.views.main_window import MainWindowView
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

    def save_configuration(self, view, title):
        window = view.get_top_widget().get_window()
        size = window.get_size()
        position = window.get_position()
        # screen = main_window.get_screen()
        # logger.debug("Main window screen:, {0}".format(screen))
        logger.debug('{0} size: {1}'.format(title, size))
        logger.debug('{0} position: {1}'.format(title, position))
        self.set_config_value('{0}_SIZE'.format(title), size)
        self.set_config_value('{0}_POS'.format(title), position)
        super(RuntimeConfig, self).save_configuration()

    def widget(self, widget):
        print type(widget)
        if isinstance(widget, gtk.HPaned):
            print widget.get_name()


# This variable holds the global configuration parameters for the runtime parameters
global_runtime_config = RuntimeConfig()
