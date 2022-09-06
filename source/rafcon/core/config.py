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
from pkg_resources import resource_string
from rafcon.design_patterns.observer.observable import Observable

from yaml_configuration.config import DefaultConfig, ConfigError

from rafcon.utils import log

logger = log.get_logger(__name__)

CONFIG_FILE = "config.yaml"
DEFAULT_CONFIG = str(resource_string(__name__, CONFIG_FILE).decode("utf-8"))
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

    def __init__(self, logger_object=None):
        """Default constructor

        :param logger_object: the logger object to pass the log output to
        :raises ConfigError: if the config type is not given in the config file
        """
        super(Config, self).__init__(DEFAULT_CONFIG, logger_object)
        if self.get_config_value("TYPE") != "SM_CONFIG":
            raise ConfigError("Type should be SM_CONFIG for state machine configuration. "
                              "Please add \"TYPE: SM_CONFIG\" to your config.yaml file.")

    def load(self, config_file=None, path=None):
        """Loads the configuration from a specific file

        :param config_file: the name of the config file
        :param path: the path to the config file
        """
        if config_file is None:
            config_file = CONFIG_FILE
        super(Config, self).load(config_file, path)


# This variable holds the global configuration parameters for the state machine
global_config = Config(logger)
