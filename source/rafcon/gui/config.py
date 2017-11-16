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
import gtk
import yaml
from rafcon.utils.resources import resource_filename, resource_exists, resource_string
from yaml_configuration.config import ConfigError

import rafcon.gui
from rafcon.gui.utils import wait_for_gui
from rafcon.core.config import ObservableConfig
from rafcon.utils import storage_utils
from rafcon.utils import log

logger = log.get_logger(__name__)

CONFIG_FILE = "gui_config.yaml"

DEFAULT_CONFIG = resource_string(__name__, CONFIG_FILE)


class GuiConfig(ObservableConfig):
    """
    Class to hold and load the global GUI configurations.
    """

    keys_requiring_state_machine_refresh = ('GAPHAS_EDITOR', 'MAX_VISIBLE_LIBRARY_HIERARCHY', 'HISTORY_ENABLED',
                                            'AUTO_BACKUP_ENABLED', 'AUTO_BACKUP_ONLY_FIX_FORCED_INTERVAL',
                                            'AUTO_BACKUP_FORCED_STORAGE_INTERVAL',
                                            'AUTO_BACKUP_DYNAMIC_STORAGE_INTERVAL',
                                            'AUTO_RECOVERY_CHECK', 'AUTO_RECOVERY_LOCK_ENABLED')
    keys_requiring_restart = ('USE_ICONS_AS_TAB_LABELS',)

    colors = {}
    gtk_colors = {}

    def __init__(self, logger_object=None):
        super(GuiConfig, self).__init__(DEFAULT_CONFIG, logger_object)
        self.load()
        if self.get_config_value("TYPE") != "GUI_CONFIG":
            raise ConfigError("Type should be GUI_CONFIG for GUI configuration. "
                              "Please add \"TYPE: GUI_CONFIG\" to your gui_config.yaml file.")
        self.path_to_tool = os.path.dirname(os.path.realpath(__file__))
        self.configure_gtk()
        self.configure_colors()

    def load(self, config_file=None, path=None):
        using_default_config = False
        if config_file is None:
            if path is None:
                using_default_config = True
                path, config_file = os.path.split(resource_filename(__name__, CONFIG_FILE))
            else:
                config_file = CONFIG_FILE
        super(GuiConfig, self).load(config_file, path)

        # fill up shortcuts
        if not using_default_config:
            default_gui_config = yaml.load(self.default_config) if self.default_config else {}
            shortcuts_dict = self.get_config_value('SHORTCUTS')
            for shortcut_name, shortcuts_list in default_gui_config.get('SHORTCUTS', {}).iteritems():
                if shortcut_name not in shortcuts_dict:
                    self.logger.info("Shortcut for '{0}' is {1}, now, and was taken from default config."
                                     "".format(shortcut_name, shortcuts_list))
                    shortcuts_dict[shortcut_name] = shortcuts_list if isinstance(shortcuts_list, list) else [shortcuts_list]

    def configure_gtk(self):
        if not resource_exists(__name__, self.get_assets_path("gtk-2.0", "gtkrc")):
            raise ValueError("GTK theme does not exist")
        gtkrc_file_path = resource_filename(__name__, self.get_assets_path("gtk-2.0", "gtkrc"))
        filename = resource_filename(__name__, self.get_assets_path(
            "icons", "RAFCON_figurative_mark_negative.svg", for_theme=False))
        gtk.window_set_default_icon_from_file(filename)

        # wait for all gtk events being processed before parsing the gtkrc file
        wait_for_gui()
        gtk.rc_parse(gtkrc_file_path)

    def configure_colors(self):
        # Get colors from GTKrc file
        if not resource_exists(__name__, self.get_assets_path("gtk-2.0", "gtkrc")):
            raise ValueError("GTK theme does not exist")

        gtkrc_file_path = resource_filename(__name__, self.get_assets_path("gtk-2.0", "gtkrc"))
        with open(gtkrc_file_path) as f:
            lines = f.readlines()

        for line in lines:
            if re.match("\s*gtk_color_scheme", line):
                color = re.findall(r'"(.*?)"', line)
                color = color[0]
                color = color.split(':')
                self.colors[color[0].upper()] = color[1]
                self.gtk_colors[color[0].upper()] = gtk.gdk.Color(color[1])

        # Get color definitions
        color_file_path = resource_filename(__name__, self.get_assets_path(filename="colors.json"))
        try:
            colors = storage_utils.load_objects_from_json(color_file_path)
        except IOError:
            raise ValueError("No color definitions found")

        # replace unicode strings with str strings
        colors = {str(key): str(value) for key, value in colors.iteritems()}
        gtk_colors = {str(key): gtk.gdk.Color(str(value)) for key, value in colors.iteritems()}
        self.gtk_colors.update(gtk_colors)
        self.colors.update(colors)

    def get_assets_path(self, folder=None, filename=None, for_theme=True):
        theme = "themes/{}/".format(self.get_config_value('THEME', 'dark')) if for_theme else ""
        folder = folder + "/" if folder else ""
        filename = filename if filename else ""
        return "assets/{theme}{folder}{filename}".format(theme=theme, folder=folder, filename=filename)


global_gui_config = GuiConfig(logger)
