import os
import sys
import re
import gtk
import yaml

from yaml_configuration.config import DefaultConfig, ConfigError
from rafcon.utils import filesystem
from rafcon.utils import storage_utils
from rafcon.mvc.utils import constants
from rafcon.utils import log

logger = log.get_logger(__name__)

CONFIG_FILE = "gui_config.yaml"

DEFAULT_CONFIG = filesystem.read_file(os.path.dirname(__file__), CONFIG_FILE)


class GuiConfig(DefaultConfig):
    """
    Class to hold and load the global GUI configurations.
    """

    colors = {}
    gtk_colors = {}

    def __init__(self, logger_object=None):
        super(GuiConfig, self).__init__(DEFAULT_CONFIG, logger_object)
        self.load(CONFIG_FILE)
        if self.get_config_value("TYPE") != "GUI_CONFIG":
            raise ConfigError("Type should be GUI_CONFIG for GUI configuration. "
                              "Please add \"TYPE: GUI_CONFIG\" to your gui_config.yaml file.")
        self.path_to_tool = os.path.dirname(os.path.realpath(__file__))
        self.configure_gtk()
        self.configure_fonts()
        self.configure_source_view_styles()
        self.configure_colors()

    def load(self, config_file=None, path=None):
        if config_file is None:
            config_file = CONFIG_FILE
        super(GuiConfig, self).load(config_file, path)

        # fill up shortcuts
        if not (config_file == CONFIG_FILE and path == os.path.dirname(__file__)):
            default_gui_config = yaml.load(self.default_config) if self.default_config else {}
            shortcuts_dict = self.get_config_value('SHORTCUTS')
            for shortcut_name, shortcuts_list in default_gui_config.get('SHORTCUTS', {}).iteritems():
                if shortcut_name not in shortcuts_dict:
                    self.logger.info("Shortcut for '{0}' is {1}, now, and was taken from default config."
                                     "".format(shortcut_name, shortcuts_list))
                    shortcuts_dict[shortcut_name] = shortcuts_list if isinstance(shortcuts_list, list) else [shortcuts_list]

    def configure_gtk(self):
        import gtk
        theme = self.get_config_value('THEME', 'dark')
        gtkrc_file_path = os.path.join(self.path_to_tool, 'themes', theme, 'gtk-2.0', 'gtkrc')
        if not os.path.exists(gtkrc_file_path):
            raise ValueError("GTK theme '{0}' does not exist".format(theme))
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

            logger.debug("Copy font '{0}' to '{1}'".format(font_name, font_user_folder))
            if not os.path.isdir(font_user_folder):
                os.makedirs(font_user_folder)
            font_origin = os.path.join(self.path_to_tool, 'themes', 'fonts', font_name)

            # A font is a folder one or more font faces
            font_faces = os.listdir(font_origin)
            for font_face in font_faces:
                target_font_file = os.path.join(font_user_folder, font_face)
                source_font_file = os.path.join(font_origin, font_face)
                filesystem.copy_file_if_update_required(source_font_file, target_font_file)
            font_copied = True

        if font_copied:
            logger.info("Restart application to apply new fonts")
            python = sys.executable
            os.execl(python, python, *sys.argv)

    def configure_source_view_styles(self):
        source_view_style_user_folder = os.path.join(os.path.expanduser('~'), '.local', 'share', 'gtksourceview-2.0',
                                                     'styles')
        filesystem.create_path(source_view_style_user_folder)
        theme = self.get_config_value('THEME', 'dark')
        source_view_style_theme_folder = os.path.join(self.path_to_tool, 'themes', theme, 'gtksw-styles')

        # Copy all .xml source view style files from theme to local user styles folder
        for style in os.listdir(source_view_style_theme_folder):
            source_view_style_theme_path = os.path.join(source_view_style_theme_folder, style)
            if not os.path.isfile(source_view_style_theme_path) or not style.endswith(".xml"):
                continue

            source_view_style_user_path = os.path.join(source_view_style_user_folder, style)
            filesystem.copy_file_if_update_required(source_view_style_theme_path, source_view_style_user_path)

    def configure_colors(self):
        theme = self.get_config_value('THEME', 'dark')

        # Get colors from GTKrc file
        gtkrc_file_path = os.path.join(self.path_to_tool, 'themes', theme, 'gtk-2.0', 'gtkrc')
        if not os.path.exists(gtkrc_file_path):
            raise ValueError("GTK theme '{0}' does not exist".format(theme))

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
        color_file_path = os.path.join(self.path_to_tool, 'themes', theme, 'colors.json')
        try:
            colors = storage_utils.load_dict_from_json(color_file_path)
        except IOError:
            raise ValueError("No color definitions for theme '{0}' found".format(theme))

        # replace unicode strings with str strings
        colors = {str(key): str(value) for key, value in colors.iteritems()}
        gtk_colors = {str(key): gtk.gdk.Color(str(value)) for key, value in colors.iteritems()}
        self.gtk_colors.update(gtk_colors)
        self.colors.update(colors)


global_gui_config = GuiConfig(logger)
