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
import sys
import re
import gtk
import yaml
from pkg_resources import resource_filename, resource_listdir, resource_exists, resource_string
from yaml_configuration.config import ConfigError

from rafcon.core.config import ObservableConfig
from rafcon.utils import filesystem
from rafcon.utils import storage_utils
from rafcon.gui.utils import constants
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
        self.configure_fonts()
        self.configure_source_view_styles()
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
        if not resource_exists(__name__, self.get_theme_path("gtk-2.0", "gtkrc")):
            raise ValueError("GTK theme does not exist")
        gtkrc_file_path = resource_filename(__name__, self.get_theme_path("gtk-2.0", "gtkrc"))
        filename = resource_filename(__name__, "icons/RAFCON_figurative_mark_negative.svg")
        gtk.window_set_default_icon_from_file(filename)

        # wait for all gtk events being processed before parsing the gtkrc file
        while gtk.events_pending():
            gtk.main_iteration(False)
        gtk.rc_parse(gtkrc_file_path)

    def configure_fonts(self):
        tv = gtk.TextView()
        try:
            context = tv.get_pango_context()
        except Exception:
            return
        if not context:  # A Pango context is not always available
            return
        existing_fonts = context.list_families()
        existing_font_names = [font.get_name() for font in existing_fonts]

        font_user_folder = os.path.join(os.path.expanduser('~'), '.fonts')

        font_copied = False

        for font_name in constants.FONTS:
            if font_name in existing_font_names:
                logger.debug("Font '{0}' found".format(font_name))
                continue

            logger.debug("Installing font '{0}' to '{1}'".format(font_name, font_user_folder))
            if not os.path.isdir(font_user_folder):
                os.makedirs(font_user_folder)

            # A font is a folder one or more font faces
            fonts_folder = self.get_theme_path("fonts", font_name, common=True)
            for font_face in resource_listdir(__name__, fonts_folder):
                target_font_file = os.path.join(font_user_folder, font_face)
                source_font_file = resource_filename(__name__, "/".join((fonts_folder, font_face)))
                filesystem.copy_file_if_update_required(source_font_file, target_font_file)
            font_copied = True

        if font_copied:
            logger.info("Restarting RAFCON to apply new fonts...")
            python = sys.executable
            os.execl(python, python, *sys.argv)

    def configure_source_view_styles(self):
        source_view_style_user_folder = os.path.join(os.path.expanduser('~'), '.local', 'share', 'gtksourceview-2.0',
                                                     'styles')
        filesystem.create_path(source_view_style_user_folder)

        source_view_folder = self.get_theme_path("gtk-sourceview")

        # Copy all .xml source view style files from theme to local user styles folder
        for style_filename in resource_listdir(__name__, source_view_folder):
            if not style_filename.endswith(".xml"):
                continue
            source_view_style_theme_path = resource_filename(__name__, "/".join((source_view_folder, style_filename)))
            source_view_style_user_path = os.path.join(source_view_style_user_folder, style_filename)
            filesystem.copy_file_if_update_required(source_view_style_theme_path, source_view_style_user_path)

    def configure_colors(self):
        # Get colors from GTKrc file
        if not resource_exists(__name__, self.get_theme_path("gtk-2.0", "gtkrc")):
            raise ValueError("GTK theme does not exist")

        gtkrc_file_path = resource_filename(__name__, self.get_theme_path("gtk-2.0", "gtkrc"))
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
        color_file_path = resource_filename(__name__, self.get_theme_path(filename="colors.json"))
        try:
            colors = storage_utils.load_objects_from_json(color_file_path)
        except IOError:
            raise ValueError("No color definitions found")

        # replace unicode strings with str strings
        colors = {str(key): str(value) for key, value in colors.iteritems()}
        gtk_colors = {str(key): gtk.gdk.Color(str(value)) for key, value in colors.iteritems()}
        self.gtk_colors.update(gtk_colors)
        self.colors.update(colors)

    def get_theme_path(self, folder=None, filename=None, common=False):
        theme = "common" if common else self.get_config_value('THEME', 'dark')
        folder = "/" + folder if folder else ""
        filename = filename if filename else ""
        return "themes/{theme}{folder}/{filename}".format(theme=theme, folder=folder, filename=filename)


global_gui_config = GuiConfig(logger)
