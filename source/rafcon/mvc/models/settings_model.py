"""
.. module:: settings_model
   :platform: Unix, Windows
   :synopsis: a module which manages the configuration settings GUI

.. moduleauthor:: Benno Voggenreiter

"""
from gtkmvc import ModelMT
from rafcon.utils import log
import yaml

from rafcon.mvc.config import global_gui_config
from rafcon.statemachine.config import global_config

logger = log.get_logger(__name__)


class SettingsModel(ModelMT):
    """
    Model which manages the configuration settings GUI
    """
    config_list = []
    config_gui_list = []
    config_library_list = []
    config_shortcut_list = []
    __observables__ = ["config_list", "config_gui_list", "config_library_list", "config_shortcut_list", ]

    def __init__(self, config_list=None, config_gui_list=None, config_library_list=None, config_shortcut_list=None, meta=None):
        ModelMT.__init__(self)
        self.config_list = config_list if config_list else []
        self.config_gui_list = config_gui_list if config_gui_list else []
        self.config_library_list = config_library_list if config_library_list else []
        self.config_shortcut_list = config_shortcut_list if config_shortcut_list else []
        self.register_observer(self)
        default_config_dict = yaml.load(global_config.default_config)
        self.config_dict = {k for k in default_config_dict.keys() if k not in ["LIBRARY_PATHS", "TYPE"]}
        default_gui_config_dict = yaml.load(global_gui_config.default_config)
        self.gui_config_dict = {k for k in default_gui_config_dict.keys() if k not in ["SHORTCUTS", "TYPE"]}

    def get_settings(self):
        """
        A function to get all values of settings listed in the dicts
        :return:
        """
        del self.config_list[:]
        for key in sorted(self.config_dict):
            if global_config.get_config_value(key) is not None:
                self.config_list.append((key, global_config.get_config_value(key)))

        del self.config_library_list[:]
        library_dict = global_config.get_config_value('LIBRARY_PATHS')
        if library_dict is not None:
            for key in sorted(library_dict.keys()):
                self.config_library_list.append((key, library_dict[key]))

        del self.config_gui_list[:]
        for key in sorted(self.gui_config_dict):
            if global_gui_config.get_config_value(key) is not None:
                self.config_gui_list.append((key, global_gui_config.get_config_value(key)))

        del self.config_shortcut_list[:]
        shortcut_dict = global_gui_config.get_config_value('SHORTCUTS')
        if shortcut_dict is not None:
            for key in sorted(shortcut_dict.keys()):
                self.config_shortcut_list.append((key, shortcut_dict[key]))

    def set_config_view_value(self, key, value, list_nr):
        """
        A method to set all config values into the config.yaml
        :param key: setting which changed
        :param value: new value for a config which shall be updated
        :param list_nr: number to select the correct list, which needs to be updated
        :return
        """
        if list_nr == 0:
            actual_list = self.config_list
        elif list_nr == 1:
            actual_list = self.config_gui_list
        elif list_nr == 2:
            actual_list = self.config_shortcut_list
            value = [value[2:-2]]
        else:
            actual_list = self.config_library_list
        for key_pair in actual_list:
            if key == key_pair[0]:
                index = actual_list.index(key_pair)
                actual_list.remove(key_pair)
                actual_list.insert(index, (key, value))
                if list_nr == 1:
                    global_gui_config.set_config_value(key, value)
                elif list_nr == 2:
                    print dict(actual_list)
                    global_gui_config.set_config_value("SHORTCUTS", dict(actual_list))
                elif list_nr == 0:
                    global_config.set_config_value(key, value)
                else:
                    global_config.set_config_value("LIBRARY_PATHS", dict(actual_list))
