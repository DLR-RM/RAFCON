import gtk
import os
import sys
import shutil

from rafcon.utils.config import DefaultConfig, ConfigError, read_file
from rafcon.utils import filesystem
from rafcon.utils import constants
from rafcon.utils import log
logger = log.get_logger(__name__)

CONFIG_FILE = "gui_config.yaml"

DEFAULT_CONFIG = read_file(os.path.dirname(__file__), CONFIG_FILE)


class GuiConfig(DefaultConfig):
    """
    Class to hold and load the global GUI configurations.
    """

    def __init__(self):
        super(GuiConfig, self).__init__(DEFAULT_CONFIG)
        self.load(CONFIG_FILE)
        if self.get_config_value("TYPE") != "GUI_CONFIG":
            raise ConfigError("Type should be GUI_CONFIG for GUI configuration. "
                              "Please add \"TYPE: GUI_CONFIG\" to your gui_config.yaml file.")
        self.path_to_tool = os.path.dirname(os.path.realpath(__file__))
        self.configure_gtk()
        self.configure_fonts()
        self.configure_source_view_styles()

    def load(self, config_file=None, path=None):
        if config_file is None:
            config_file = CONFIG_FILE
        super(GuiConfig, self).load(config_file, path)

    @staticmethod
    def configure_gtk():
        import gtk
        file_path = os.path.dirname(os.path.realpath(__file__))
        gtkrc_path = os.path.join(file_path, 'themes', 'black', 'gtk-2.0', 'gtkrc')
        gtk.rc_parse(gtkrc_path)

    def configure_fonts(self):
        tv = gtk.TextView()
        context = tv.get_pango_context()
        existing_fonts = context.list_families()
        existing_font_names = [font.get_name() for font in existing_fonts]

        font_path = os.path.join(os.path.expanduser('~'), '.fonts')

        font_copied = False

        for font_name in constants.FONT_NAMES:
            if font_name in existing_font_names:
                logger.debug("Font '{0}' found".format(font_name))
                break

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
        code_style_path = os.path.join(os.path.expanduser('~'), '.local', 'share', 'gtksourceview-2.0', 'styles')

        filesystem.create_path(code_style_path)

        for style in constants.STYLE_NAMES:
            code_style_origin_path = os.path.join(self.path_to_tool, constants.FONT_STYLE_PATHS[style])
            code_style_target_path = os.path.join(code_style_path, style)

            if not os.path.isfile(code_style_target_path) or filesystem.get_md5_file_hash(code_style_origin_path) != \
                    filesystem.get_md5_file_hash(code_style_target_path):
                # Copy current version
                logger.debug("Copy code style '{0}' to '{1}'".format(style, code_style_target_path))
                shutil.copy(code_style_origin_path, code_style_path)

global_gui_config = GuiConfig()
