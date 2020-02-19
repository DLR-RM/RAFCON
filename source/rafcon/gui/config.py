# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import os
import re
import yaml
try:
    from yaml import CFullLoader as FullLoader
except ImportError:
    from yaml import FullLoader
from collections import defaultdict
from pkg_resources import resource_filename, resource_string

from yaml_configuration.config import ConfigError

from rafcon.core.config import ObservableConfig
from rafcon.utils.resources import get_data_file_path
from rafcon.utils import storage_utils
from rafcon.utils import log

logger = log.get_logger(__name__)

CONFIG_FILE = "gui_config.yaml"

DEFAULT_CONFIG = str(resource_string(__name__, CONFIG_FILE).decode("utf-8"))


class GuiConfig(ObservableConfig):
    """
    Class to hold and load the global GUI configurations.
    """

    keys_requiring_state_machine_refresh = {'MAX_VISIBLE_LIBRARY_HIERARCHY', 'HISTORY_ENABLED',
                                            'AUTO_BACKUP_ENABLED', 'AUTO_BACKUP_ONLY_FIX_FORCED_INTERVAL',
                                            'AUTO_BACKUP_FORCED_STORAGE_INTERVAL',
                                            'AUTO_BACKUP_DYNAMIC_STORAGE_INTERVAL',
                                            'AUTO_RECOVERY_CHECK', 'AUTO_RECOVERY_LOCK_ENABLED'}
    keys_requiring_restart = {'USE_ICONS_AS_TAB_LABELS', 'THEME_DARK_VARIANT'}

    colors = {}
    gtk_colors = {}

    def __init__(self, logger_object=None):
        super(GuiConfig, self).__init__(DEFAULT_CONFIG, logger_object)
        if self.get_config_value("TYPE") != "GUI_CONFIG":
            raise ConfigError("Type should be GUI_CONFIG for GUI configuration. "
                              "Please add \"TYPE: GUI_CONFIG\" to your gui_config.yaml file.")
        self.path_to_tool = os.path.dirname(os.path.realpath(__file__))
        self.configure_gtk()
        self.configure_colors()

    def load(self, config_file=None, path=None):
        if config_file is None:
            config_file = CONFIG_FILE
        super(GuiConfig, self).load(config_file, path)

        self.configure_gtk()
        self.configure_colors()

    def get_theme_path(self):
        return get_data_file_path("themes", "RAFCON")

    def configure_gtk(self):
        theme_path = self.get_theme_path()
        if not theme_path:
            raise ValueError("GTK theme 'RAFCON' does not exist")

        theme_name = "RAFCON"
        dark_theme = self.get_config_value('THEME_DARK_VARIANT', True)

        # GTK_DATA_PREFIX must point to a path that contains share/themes/THEME_NAME
        gtk_data_prefix = os.path.dirname(os.path.dirname(os.path.dirname(theme_path)))
        os.environ['GTK_DATA_PREFIX'] = gtk_data_prefix
        os.environ['GTK_THEME'] = "{}{}".format(theme_name, ":dark" if dark_theme else "")

        # The env vars GTK_DATA_PREFIX and GTK_THEME must be set before Gtk is imported first to prevent GTK warnings
        # from other themes
        try:
            from gi.repository import Gtk
            settings = Gtk.Settings.get_default()
            if settings:
                settings.set_property("gtk-enable-animations", True)
                settings.set_property("gtk-theme-name", theme_name)
                settings.set_property("gtk-application-prefer-dark-theme", dark_theme)

            Gtk.Window.set_default_icon_name("rafcon" if dark_theme else "rafcon-light")
        except ImportError:
            pass

    def configure_colors(self):
        # Provide black as fallback color if theme is not found instead of crashing
        self.colors = defaultdict(lambda: "#FFFFFF")
        try:
            from gi.repository import Gdk
            self.gtk_colors = defaultdict(lambda: Gdk.RGBA(0, 0, 0).to_color())
        except ImportError:
            self.gtk_colors = defaultdict(lambda: None)
            return

        dark_theme = self.get_config_value('THEME_DARK_VARIANT', True)
        css_filename = "gtk-dark.css" if dark_theme else "gtk.css"
        # Get colors from GTKrc file
        theme_path = self.get_theme_path()
        css_file_path = os.path.join(theme_path, "gtk-3.0", css_filename)
        if not os.path.isfile(css_file_path):
            raise ValueError("GTK theme does not exist")

        with open(css_file_path) as f:
            lines = f.readlines()

        for line in lines:
            match = re.match("\s*@define-color (\w*) (#[\w]{3,6})", line)
            if match:
                color_name = match.group(1).upper()
                color_code = match.group(2)
                self.colors[color_name] = color_code
                gtk_color = Gdk.RGBA()
                if gtk_color.parse(color_code):
                    self.gtk_colors[color_name] = gtk_color.to_color()
                else:
                    self.logger.warning("Could not parse color with name '{}' and code '{}'".format(color_name,
                                                                                                    color_code))

        # Get color definitions
        colors_filename = "colors-dark.json" if dark_theme else "colors.json"
        color_file_path = os.path.join(theme_path, colors_filename)
        try:
            colors = storage_utils.load_objects_from_json(color_file_path)
        except IOError:
            raise ValueError("No color definitions found")

        for color_name, color_code in colors.items():
            # replace unicode strings with str strings
            color_name = str(color_name)
            color_code = str(color_code)
            gtk_color = Gdk.RGBA()
            if color_code.startswith("#"):
                if gtk_color.parse(color_code):
                    color = gtk_color.to_color()
                else:
                    self.logger.warning("Could not parse color with name '{}' and code '{}'".format(color_name,
                                                                                                    color_code))
                    continue
            elif color_code in self.colors:
                color = self.gtk_colors[color_code]
                color_code = self.gtk_colors[color_code]
            else:
                self.logger.warning("Undefined color alias '{}' for color name '{}'".format(color_code, color_name))
                continue
            self.gtk_colors[color_name] = color
            self.colors[color_name] = color_code

    @staticmethod
    def get_assets_path(folder=None, filename=None, for_theme=True):
        theme = "share/themes/RAFCON/" if for_theme else ""
        folder = folder + "/" if folder else ""
        filename = filename if filename else ""
        return "assets/{theme}{folder}{filename}".format(theme=theme, folder=folder, filename=filename)


global_gui_config = GuiConfig(logger)
