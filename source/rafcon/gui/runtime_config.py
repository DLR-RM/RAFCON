# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: config
   :synopsis: Configuration for runtime parameters, such as window size and position

"""
import gtk

from rafcon.core.config import ObservableConfig
from rafcon.utils import log

logger = log.get_logger(__name__)

CONFIG_FILE = "runtime_config.yaml"


class RuntimeConfig(ObservableConfig):
    """Class to hold and load the runtime configuration"""

    def __init__(self):
        super(RuntimeConfig, self).__init__("")

    def load(self, config_file=None, path=None):
        if config_file is None:
            config_file = CONFIG_FILE
        super(RuntimeConfig, self).load(config_file, path)

    def store_widget_properties(self, widget, title):
        """Sets configuration values for widgets

        If the widget is a window, then the size and position are stored. If the widget is a pane, then only the
        position is stored. If the window is maximized the last insert position before being maximized is keep in the
        config and the maximized flag set to True. The maximized state and the last size and position are strictly
        separated by this.

        :param widget: The widget, for which the position (and possibly the size) will be stored.
        :param title: The title of the widget, which constitutes a part of its key in the configuration file.
        """
        if isinstance(widget, gtk.Window):
            maximized = bool(widget.maximize_initially)
            self.set_config_value('{0}_MAXIMIZED'.format(title), maximized)
            if maximized:
                return
            size = widget.get_size()
            self.set_config_value('{0}_SIZE'.format(title), size)
        position = widget.get_position()
        self.set_config_value('{0}_POS'.format(title), position)

    def save_configuration(self):
        # screen = main_window.get_screen()
        # logger.debug("Main window screen:, {0}".format(screen))

        # if the runtime_config was not loaded in some startup routine then load it explicitly (= create it)
        if not self.config_file_path:
            self.load()

        super(RuntimeConfig, self).save_configuration()


# This variable holds the global configuration parameters for the runtime parameters
global_runtime_config = RuntimeConfig()
