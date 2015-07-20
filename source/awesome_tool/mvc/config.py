import gtk
import os
import sys
import shutil

from awesome_tool.utils.config import DefaultConfig, ConfigError
from awesome_tool.utils import constants
from awesome_tool.utils import log
logger = log.get_logger(__name__)

DEFAULT_CONFIG = """

TYPE: GUI_CONFIG

SOURCE_EDITOR_STYLE: awesome-style

GAPHAS_EDITOR: True

WAYPOINT_SNAP_ANGLE: 45
WAYPOINT_SNAP_MAX_DIFF_ANGLE: 10
WAYPOINT_SNAP_MAX_DIFF_PIXEL: 50

SHOW_DATA_FLOWS: True
DATA_FLOW_MODE: True
SHOW_DATA_FLOW_VALUE_LABELS: False

ROTATE_NAMES_ON_CONNECTIONS: False
HISTORY_ENABLED: False
"""

CONFIG_FILE = "gui_config.yaml"


class GuiConfig(DefaultConfig):
    """
    Class to hold and load the global GUI configurations.
    """

    def __init__(self):
        super(GuiConfig, self).__init__(DEFAULT_CONFIG)
        if self.get_config_value("TYPE") != "GUI_CONFIG":
            raise ConfigError("Type should be GUI_CONFIG for GUI configuration. "
                              "Please add \"TYPE: GUI_CONFIG\" to your gui_config.yaml file.")
        self.path_to_tool = os.path.dirname(os.path.realpath(__file__))
        self.configure_gtk()
        self.configure_fonts()
        self.configure_source_view_styles()

        self.load(CONFIG_FILE)

    def load(self, config_file=None, path=None):
        if config_file is None:
            config_file = CONFIG_FILE
        super(GuiConfig, self).load(config_file, path)

    def configure_gtk(self):
        import gtk
        file_path = os.path.dirname(os.path.realpath(__file__))
        gtkrc_path = os.path.join(file_path, 'themes', 'black', 'gtk-2.0', 'gtkrc')
        gtk.rc_parse(gtkrc_path)

    def configure_fonts(self):
        tv = gtk.TextView()
        context = tv.get_pango_context()
        fonts = context.list_families()

        font_path = os.path.join(os.path.expanduser('~'), '.fonts')

        font_copied = False

        for font_name in constants.FONT_NAMES:
            found = False
            for font in fonts:
                if font.get_name() == font_name:
                    logger.debug("Font '{0}' found".format(font_name))
                    found = True
            if not found:
                logger.debug("Copy font '{0}' to '{1}'".format(font_name, font_path))
                if not os.path.isdir(font_path):
                    os.makedirs(font_path)
                font_origin = os.path.join(self.path_to_tool, constants.FONT_STYLE_PATHS[font_name])

                if os.path.isdir(font_origin):
                    font_names = os.listdir(font_origin)
                    for font_filename in font_names:
                        shutil.copy(os.path.join(font_origin, font_filename), font_path)
                else:
                    shutil.copy(font_origin, font_path)
                font_copied = True

        if font_copied:
            logger.info("Restart application to apply new fonts")
            python = sys.executable
            os.execl(python, python, * sys.argv)

    def configure_source_view_styles(self):
        path = os.path.join(os.path.expanduser('~'), '.local', 'share', 'gtksourceview-2.0', 'styles')

        if not os.path.isdir(path):
            os.makedirs(path)

        for style in constants.STYLE_NAMES:
            font_style_origin = os.path.join(self.path_to_tool, constants.FONT_STYLE_PATHS[style])
            font_style_target = os.path.join(path, style)

            # Remove old versions
            if os.path.isfile(font_style_target):
                os.remove(os.path.join(path, style))

            # Copy current version
            shutil.copy(font_style_origin, path)

global_gui_config = GuiConfig()
