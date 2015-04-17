"""
.. module:: config
   :platform: Unix, Windows
   :synopsis: Config module to specify global constants

.. moduleauthor:: Sebastian Brunner


"""
import yaml
import os
import sys
import gtk

from awesome_tool.utils.storage_utils import StorageUtils
from awesome_tool.utils import constants
from awesome_tool.utils import log
logger = log.get_logger(__name__)


DEFAULT_CONFIG = """

LIBRARY_PATHS: {"test_libraries": "../../test_scripts/test_libraries",
                 "ros_libraries": "../../test_scripts/ros_libraries",
                 "turtle_libraries": "../../test_scripts/turtle_libraries"}

SOURCE_EDITOR_STYLE: blue_dream

WAYPOINT_SNAP_ANGLE: 45
WAYPOINT_SNAP_MAX_DIFF_ANGLE: 10
WAYPOINT_SNAP_MAX_DIFF_PIXEL: 50
"""

CONFIG_PATH = os.getenv("HOME") + "/.awesome_tool"
CONFIG_FILE = "config.yaml"


class Config(object):
    """
    Class to hold and load the global configurations.
    """

    def __init__(self):
        self.storage = StorageUtils("~/")
        if not self.storage.exists_path(os.path.join(CONFIG_PATH, CONFIG_FILE)):
            self.storage.create_path(CONFIG_PATH)
            open(os.path.join(CONFIG_PATH, CONFIG_FILE), "a").close()
            yaml_dict = yaml.load(DEFAULT_CONFIG)
            self.storage.write_dict_to_yaml(yaml_dict, os.path.join(CONFIG_PATH, CONFIG_FILE))
        self.__config_dict = self.storage.load_dict_from_yaml(os.path.join(CONFIG_PATH, CONFIG_FILE))
        logger.info("Config initialized ... loaded configuration from %s" % str(os.path.join(CONFIG_PATH, CONFIG_FILE)))
        self.configure_fonts()
        self.configure_source_view_styles()

    def get_config_value(self, key, default=None):
        """
        Get a specific configuration value
        :param key: the key to the configuration value
        :param default: what to return if the key is not found
        :return:
        """
        if key in self.__config_dict:
            return self.__config_dict[key]
        return default

    def set_config_value(self, key, value):
        """
        Get a specific configuration value
        :param key: the key to the configuration value
        :return:
        """
        self.__config_dict[key] = value

    def save_configuration(self):
        self.storage.write_dict_to_yaml(self.__config_dict, os.path.join(CONFIG_PATH, CONFIG_FILE))
        logger.info("Saved configuration to filesystem (path: %s)" % str(os.path.join(CONFIG_PATH, CONFIG_FILE)))

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


# This variable holds the global configuration parameters for the statemachine
global_config = Config()