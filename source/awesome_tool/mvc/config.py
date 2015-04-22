import gtk
import os
import sys

from awesome_tool.utils.config import DefaultConfig, ConfigError
from awesome_tool.utils import constants
from awesome_tool.utils import helper
from awesome_tool.utils import log
logger = log.get_logger(__name__)

DEFAULT_CONFIG = """

TYPE: GUI_CONFIG

SOURCE_EDITOR_STYLE: awesome-style

WAYPOINT_SNAP_ANGLE: 45
WAYPOINT_SNAP_MAX_DIFF_ANGLE: 10
WAYPOINT_SNAP_MAX_DIFF_PIXEL: 50

show_data_flows: true
"""

CONFIG_FILE = "gui_config.yaml"


class GuiConfig(DefaultConfig):
    """
    Class to hold and load the global GUI configurations.
    """

    def __init__(self):
        sm_path, gui_path = helper.get_opt_paths()
        DefaultConfig.__init__(self, CONFIG_FILE, DEFAULT_CONFIG, gui_path)
        if self.get_config_value("TYPE") != "GUI_CONFIG":
            raise ConfigError("Type should be GUI_CONFIG for GUI configuration. "
                              "Please add \"TYPE: GUI_CONFIG\" to your gui_config.yaml file.")
        self.configure_fonts()
        self.configure_source_view_styles()

    def configure_fonts(self):
        tv = gtk.TextView()
        context = tv.get_pango_context()
        fonts = context.list_families()

        font_path = os.getenv("HOME") + "/.fonts"

        font_copied = False

        for font_name in constants.FONT_NAMES:
            found = False
            for font in fonts:
                if font.get_name() == font_name:
                    logger.info("Font %s found" % font_name)
                    found = True
            if not found:
                logger.info("Copy font %s to %s" % (font_name, font_path))
                if not os.path.isdir(font_path):
                    os.system("mkdir -p %s" % font_path)
                os.system("cp -r %s %s" % (constants.FONT_STYLE_PATHS[font_name], font_path))
                font_copied = True

        if font_copied:
            logger.info("Restart application to apply new fonts")
            python = sys.executable
            os.execl(python, python, * sys.argv)

    def configure_source_view_styles(self):
        path = os.getenv("HOME") + "/.local/share/gtksourceview-2.0/styles"

        if not os.path.isdir(path):
            os.system("mkdir -p %s" % path)

        for style in constants.STYLE_NAMES:
            if not os.path.isfile(os.path.join(path, style)):
                logger.info("Copy style %s to %s" % (style, path))
                os.system("cp %s %s " % (constants.FONT_STYLE_PATHS[style], path))
            else:
                logger.info("Found %s" % style)

                logger.warning("REMOVE THE FOLLOWING TWO LINES AFTER COMPLETION OF %s" % style)
                os.system("rm " + path + "/%s" % style)
                os.system("cp %s %s " % (constants.FONT_STYLE_PATHS[style], path))

global_gui_config = GuiConfig()