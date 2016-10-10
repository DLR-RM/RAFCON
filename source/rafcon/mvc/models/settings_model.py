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
    current_core_config = []
    current_gui_config = []
    current_library_config = []
    current_shortcut_config = []
    changed_entries = {}
    __observables__ = ["current_core_config", "current_gui_config", "current_library_config",
                       "current_shortcut_config", "changed_entries"]

    _keys_requiring_state_machine_refresh = ('GAPHAS_EDITOR', 'MAX_VISIBLE_LIBRARY_HIERARCHY', 'HISTORY_ENABLED',
                                             'AUTO_BACKUP_ENABLED', 'AUTO_BACKUP_ONLY_FIX_FORCED_INTERVAL',
                                             'AUTO_BACKUP_FORCED_STORAGE_INTERVAL',
                                             'AUTO_BACKUP_DYNAMIC_STORAGE_INTERVAL',
                                             'AUTO_RECOVERY_CHECK', 'AUTO_RECOVERY_LOCK_ENABLED')
    _keys_requiring_restart = ('USE_ICONS_AS_TAB_LABELS',)

    def __init__(self, config_list=None, config_gui_list=None, config_library_list=None,
                 config_shortcut_list=None):
        ModelMT.__init__(self)
        self.current_core_config = config_list if config_list else []
        self.current_gui_config = config_gui_list if config_gui_list else []
        self.current_library_config = config_library_list if config_library_list else []
        self.current_shortcut_config = config_shortcut_list if config_shortcut_list else []
        default_config_dict = yaml.load(global_config.default_config)
        self._keys_core_config = set([k for k in default_config_dict.keys() if k not in ["LIBRARY_PATHS", "TYPE"]])
        default_gui_config_dict = yaml.load(global_gui_config.default_config)
        self._keys_gui_config = set([k for k in default_gui_config_dict.keys() if k not in ["SHORTCUTS", "TYPE"]])
        self.changed_entries = {}
        self.changed_keys_requiring_restart = set()
        self.changed_libs = []

    def detect_changes(self):
        """Detects all changes when loading a new config file and updates the view
        """
        from rafcon.mvc.singleton import main_window_controller

        for key, value in self.current_core_config:
            if value != global_config.get_config_value(key):
                self.set_preliminary_config_value(key, global_config.get_config_value(key))

        for key, value in self.current_gui_config:
            if value != global_gui_config.get_config_value(key):
                self.set_preliminary_config_value(key, global_gui_config.get_config_value(key))

        if dict(self.current_shortcut_config) != global_gui_config.get_config_value('SHORTCUTS'):
            main_window_controller.get_controller('menu_bar_controller').refresh_shortcuts_activate()

        if dict(self.current_library_config) != global_config.get_config_value("LIBRARY_PATHS"):
            main_window_controller.get_controller('menu_bar_controller').on_refresh_libraries_activate(widget=None, data=None)

    def load_settings(self):
        """A function to load all values of settings listed in the dicts
        """
        del self.current_core_config[:]
        for key in sorted(self._keys_core_config):
            if global_config.get_config_value(key) is not None:
                self.current_core_config.append((key, global_config.get_config_value(key)))

        del self.current_library_config[:]
        library_dict = global_config.get_config_value('LIBRARY_PATHS')
        if library_dict is not None:
            for key in sorted(library_dict.keys()):
                self.current_library_config.append((key, library_dict[key]))

        del self.current_gui_config[:]
        for key in sorted(self._keys_gui_config):
            if global_gui_config.get_config_value(key) is not None:
                self.current_gui_config.append((key, global_gui_config.get_config_value(key)))

        del self.current_shortcut_config[:]
        shortcut_dict = global_gui_config.get_config_value('SHORTCUTS')
        if shortcut_dict is not None:
            for key in sorted(shortcut_dict.keys()):
                self.current_shortcut_config.append((key, shortcut_dict[key]))

        self.changed_entries.clear()

    def set_preliminary_config_value(self, key, value):
        """A method to show all config values from current the config.yaml

        :param key: setting which changed
        :param value: new value for a config which shall be updated
        """
        if key in self._keys_core_config:
            changed_config = self.current_core_config
        elif key in self._keys_gui_config:
            changed_config = self.current_gui_config
        elif key in dict(self.current_shortcut_config):
            changed_config = self.current_shortcut_config
            value = [value[2:-2]]
        else:
            changed_config = self.current_library_config
        for key_pair in changed_config:
            if key == key_pair[0]:
                index = changed_config.index(key_pair)
                changed_config.remove(key_pair)
                changed_config.insert(index, (key, value))
                self.changed_entries[key] = value

    def save_and_apply_config(self):
        """Saving and applying changed settings
        """
        from rafcon.mvc.singleton import main_window_controller

        state_machine_refresh_required = False
        for config_key, config_value in self.changed_entries.iteritems():
            if config_key in SettingsModel._keys_requiring_state_machine_refresh:
                global_gui_config.set_config_value(config_key, config_value)
                state_machine_refresh_required = True
            elif config_key in SettingsModel._keys_requiring_restart:
                global_gui_config.set_config_value(config_key, config_value)
                self.changed_keys_requiring_restart.add(config_key)
            elif config_key in self._keys_core_config:
                global_config.set_config_value(config_key, config_value)
                self.changed_keys_requiring_restart.add(config_key)
            elif config_key in dict(self.current_shortcut_config):
                global_gui_config.set_config_value("SHORTCUTS", dict(self.current_shortcut_config))
                main_window_controller.get_controller('menu_bar_controller').refresh_shortcuts_activate()
            elif config_key in dict(self.current_library_config) or config_key in self.changed_libs:
                global_config.set_config_value("LIBRARY_PATHS", dict(self.current_library_config))
                main_window_controller.get_controller('menu_bar_controller').on_refresh_libraries_activate(
                    widget=None, data=None)
            else:
                global_gui_config.set_config_value(config_key, config_value)
                if config_key == "SOURCE_EDITOR_STYLE":
                    main_window_controller.get_controller('states_editor_ctrl').reload_style()
                if "LOGGING_SHOW_" in config_key:
                    if "INFO" in config_key:
                        main_window_controller.view['button_show_info'].set_active(config_value)
                    elif "DEBUG" in config_key:
                        main_window_controller.view['button_show_debug'].set_active(config_value)
                    elif "WARNING" in config_key:
                        main_window_controller.view['button_show_warning'].set_active(config_value)
                    elif "ERROR" in config_key:
                        main_window_controller.view['button_show_error'].set_active(config_value)
                    main_window_controller.view.logging_view.update_filtered_buffer()

        if state_machine_refresh_required:
            main_window_controller.get_controller('menu_bar_controller').on_refresh_all_activate(widget=None, data=None)

        global_config.save_configuration()
        global_gui_config.save_configuration()
        self.changed_entries.clear()
        del self.changed_libs[:]

    def ignore_changes(self, key, value):
        """Called when every time a checkbox is toggled, avoids the refresh of a widget if value is old value

        :param key: setting
        :param value: value
        """
        if key in self._keys_core_config:
            if value == global_config.get_config_value(key):
                self.changed_entries.pop(key)
        elif key in self._keys_gui_config:
            if value == global_gui_config.get_config_value(key):
                self.changed_entries.pop(key)

    def delete_library(self, key):
        for lib in self.current_library_config:
            if key == lib[0]:
                self.current_library_config.remove(lib)
                self.changed_entries[lib[0]] = None
                self.changed_libs.append(key)

    def add_library(self, key, value):
        self.current_library_config.append((key, value))
        self.set_preliminary_config_value(key, value)

    def set_library_key(self, key, old_key, value):
        for key_tmp in self.current_library_config:
            if key_tmp[0] == old_key:
                self.current_library_config.remove(key_tmp)
                self.current_library_config.append((key, value))
                self.set_preliminary_config_value(key, value)
