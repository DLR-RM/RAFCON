# Copyright (C) 2016-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Benno Voggenreiter <benno.voggenreiter@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: config_model
   :synopsis: a module which manages the configuration GUI

"""
from os import path
from copy import copy
from gtkmvc3.model_mt import ModelMT

from rafcon.utils import log

logger = log.get_logger(__name__)


class ConfigModel(ModelMT):
    """Model managing an ObservableConfig
    """

    config = None
    preliminary_config = None

    __observables__ = ("config", "preliminary_config")

    def __init__(self, config):
        super(ConfigModel, self).__init__()

        self.config = config
        self.preliminary_config = {}

        self.changed_keys_requiring_restart = set()

    def as_dict(self, use_preliminary=False):
        """Create a copy of the config in form of a dict

        :param bool use_preliminary: Whether to include the preliminary config
        :return: A dict with the copy of the config
        :rtype: dict
        """
        config = dict()
        for key in self.config.keys:
            if use_preliminary and key in self.preliminary_config:
                value = self.preliminary_config[key]
            else:
                value = self.config.get_config_value(key)
            config[key] = value
        return config

    def update_config(self, config_dict, config_file):
        """Update the content and reference of the config

        :param dict config_dict: The new configuration
        :param str config_file: The new file reference
        """
        config_path = path.dirname(config_file)
        self.config.config_file_path = config_file
        self.config.path = config_path
        for config_key, config_value in config_dict.items():
            if config_value != self.config.get_config_value(config_key):
                self.set_preliminary_config_value(config_key, config_value)

    def get_current_config_value(self, config_key, use_preliminary=True, default=None):
        """Returns the current config value for the given config key

        :param str config_key: Config key who's value is requested
        :param bool use_preliminary: Whether the preliminary config should be queried first
        :param default: The value to return if config key does not exist
        :return: Copy of the config value
        """
        if use_preliminary and config_key in self.preliminary_config:
            return copy(self.preliminary_config[config_key])
        return copy(self.config.get_config_value(config_key, default))

    def set_preliminary_config_value(self, config_key, config_value):
        """Stores a config value as preliminary new value

        The config value is not yet applied to the configuration. If the value is identical to the one from the
        configuration, the entry is deleted from the preliminary config.

        :param str config_key: Key of the entry
        :param config_value: New value
        """
        if config_value != self.config.get_config_value(config_key):
            self.preliminary_config[config_key] = config_value
        # If the value was reverted to its original value, we can remove the entry
        elif config_key in self.preliminary_config:
            del self.preliminary_config[config_key]

    def apply_preliminary_config(self, save=True):
        """Applies the preliminary config to the configuration

        :param bool save: Whether the config file is be be written to the file system
        :return: Whether the applied changes require a refresh of the state machines
        :rtype: bool
        """
        state_machine_refresh_required = False
        for config_key, config_value in self.preliminary_config.items():
            self.config.set_config_value(config_key, config_value)
            if config_key in self.config.keys_requiring_state_machine_refresh:
                state_machine_refresh_required = True
            elif config_key in self.config.keys_requiring_restart:
                self.changed_keys_requiring_restart.add(config_key)
            if config_key == 'AUTO_RECOVERY_LOCK_ENABLED':
                import rafcon.gui.models.auto_backup
                if config_value:
                    rafcon.gui.models.auto_backup.generate_rafcon_instance_lock_file()
                else:
                    rafcon.gui.models.auto_backup.remove_rafcon_instance_lock_file()
        self.preliminary_config.clear()

        if save:
            self.config.save_configuration()
        return state_machine_refresh_required
