# Copyright (C) 2014-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Matthias Buettner <matthias.buettner@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: config
   :synopsis: Config module to specify global constants

"""

import os
import yaml
from pathlib import Path
from rafcon.design_patterns.observer.observable import Observable

from yaml_configuration.config import DefaultConfig, ConfigError

from rafcon.utils import log

logger = log.get_logger(__name__)

CONFIG_FILE = "config.yaml"
DEFAULT_CONFIG = Path(__file__).with_name(CONFIG_FILE).read_text()
RELATIVE_PATH_KEYWORD = "$RELATIVE_PATH"


class ObservableConfig(DefaultConfig, Observable):

    keys_requiring_state_machine_refresh = set()
    keys_requiring_restart = set()

    def __init__(self, default_config_string, logger_object=None):
        DefaultConfig.__init__(self, default_config_string, logger_object, rel_config_path='rafcon')
        Observable.__init__(self)

    @Observable.observed
    def set_config_value(self, key, value):
        super(ObservableConfig, self).set_config_value(key, value)

    def as_dict(self):
        """Returns the configuration as dict

        :return: A copy of the whole configuration as dict
        :rtype: dict
        """
        return dict(self._config_dict)

    @property
    def keys(self):
        return set(self._config_dict.keys())

    def is_config_loaded_from_file(self):
        """ Returns if the configuration values were loaded from a custom file
        (and are thus not simply initiated from the default config any more).

        :return: a flag indicating if the config was loaded from a file
        """
        # Currently this can be checked by checking if self.path is not None any more
        # self.path is set in the load function of the DefaultConfig base class
        return self.path is not None

    def get_config_value(self, key, default=None):
        """Overwrites the default behavior of the get_config_value method of the DefaultConfig base class
        It supports the

        :param key: the key to the configuration value
        :param default: what to return if the key is not found
        :return: The value for the given key, if the key was found. Otherwise the default value
        """
        return_value = DefaultConfig.get_config_value(self, key, default)
        if self.is_config_loaded_from_file():
            if isinstance(return_value, str) and (RELATIVE_PATH_KEYWORD in return_value):
                return_value = return_value.replace(RELATIVE_PATH_KEYWORD+"{", self.path + os.path.sep)
                return_value = return_value.replace("}", "")
        return return_value


class Config(ObservableConfig):
    """ Class to hold and load the global state machine configurations.

    """

    keys_requiring_restart = ()
    
    # Define expected types for config values
    EXPECTED_CONFIG_TYPES = {
        'LIBRARY_RECOVERY_MODE': bool,
        'LOAD_SM_WITH_CHECKS': bool,
        'STORAGE_PATH_WITH_STATE_NAME': bool,
        'NO_PROGRAMMATIC_CHANGE_OF_LIBRARY_STATES_PERFORMED': bool,
        'IN_MEMORY_EXECUTION_HISTORY_ENABLE': bool,
        'FILE_SYSTEM_EXECUTION_HISTORY_ENABLE': bool,
        'EXECUTION_LOG_SET_READ_AND_WRITABLE_FOR_ALL': bool,
        'SCRIPT_RECOMPILATION_ON_STATE_EXECUTION': bool,
        'MAX_LENGTH_FOR_STATE_NAME_IN_STORAGE_PATH': (type(None), int),
    }

    def __init__(self, logger_object=None):
        """Default constructor

        :param logger_object: the logger object to pass the log output to
        :raises ConfigError: if the config type is not given in the config file
        """
        super(Config, self).__init__(DEFAULT_CONFIG, logger_object)
        if self.get_config_value("TYPE") != "SM_CONFIG":
            raise ConfigError("Type should be SM_CONFIG for state machine configuration. "
                              "Please add \"TYPE: SM_CONFIG\" to your config.yaml file.")

    def _validate_config_values(self):
        """Validate config values and replace invalid ones with defaults
        
        This method checks if config values match their expected types.
        For boolean values, it accepts True/true/False/false as valid.
        Any other value is considered invalid and replaced with the default.
        """
        # Get default values from the default config
        default_config_dict = yaml.safe_load(self.default_config)
        
        corrected_values = []
        
        for key, expected_type in self.EXPECTED_CONFIG_TYPES.items():
            current_value = self.get_config_value(key)
            
            if current_value is None:
                # Value not present, will be filled from default config.yaml
                continue
            
            # Handle multiple expected types (tuple) or single type
            expected_types = expected_type if isinstance(expected_type, tuple) else (expected_type,)
            
            if bool in expected_types:
                # Check if the value is a valid boolean
                if isinstance(current_value, bool):
                    # Already a boolean, no issue
                    continue
                elif isinstance(current_value, str) and current_value.lower() in ['true', 'false']:
                    # Valid string representation of boolean
                    self.set_config_value(key, current_value.lower() == 'true')
                    continue
                    
            # Special handling for None/int values
            if type(None) in expected_types and int in expected_types:
                # Check if the value is valid (None, int, or string 'None')
                if current_value is None or isinstance(current_value, int):
                    continue
                elif isinstance(current_value, str):
                    if current_value.lower() == 'none':
                        self.set_config_value(key, None)
                        continue
                    # Try to parse as int
                    try:
                        int_value = int(current_value)
                        self.set_config_value(key, int_value)
                        continue
                    except ValueError:
                        pass  # Will fall through to use default
                        
            # If we get here, the value is invalid - use default
            default_value = default_config_dict.get(key)
            if default_value is not None or (type(None) in expected_types and key in default_config_dict):
                type_names = []
                for t in expected_types:
                    if t == type(None):
                        type_names.append('None')
                    else:
                        type_names.append(t.__name__)
                expected_type_str = '/'.join(type_names)
                
                logger.warning(
                    f"Invalid value '{current_value}' for config key '{key}'. "
                    f"Expected {expected_type_str}, using default value: {default_value}"
                )
                self.set_config_value(key, default_value)
                corrected_values.append((key, current_value, default_value))
        
        # Save config if any values were corrected
        if corrected_values and self.path:
            logger.info(f"Corrected {len(corrected_values)} invalid config value(s) with defaults")
            self.save_configuration()

    def load(self, config_file=None, path=None):
        """Loads the configuration from a specific file

        :param config_file: the name of the config file
        :param path: the path to the config file
        """
        if config_file is None:
            config_file = CONFIG_FILE
            
        # Check if custom config has LIBRARY_PATHS to decide whether to prevent merging
        if path is not None:
            custom_config_path = os.path.join(path, config_file)
            if os.path.exists(custom_config_path):
                with open(custom_config_path, 'r') as f:
                    custom_config = yaml.safe_load(f)
                if custom_config and 'LIBRARY_PATHS' in custom_config:
                    # Always preserve custom LIBRARY_PATHS if present
                    self.keys_not_to_fill_up.add('LIBRARY_PATHS')
                    logger.debug("Custom config contains LIBRARY_PATHS - preserving user-defined library paths")
                else:
                    logger.debug("Custom config missing LIBRARY_PATHS - will use default library paths")
                    
        super(Config, self).load(config_file, path)
        
        # Validate config values after loading
        self._validate_config_values()


# This variable holds the global configuration parameters for the state machine
global_config = Config(logger)
