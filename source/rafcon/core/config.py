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

from os.path import split
from pkg_resources import resource_string, resource_filename
from gtkmvc3.observable import Observable

from yaml_configuration.config import DefaultConfig, ConfigError

from rafcon.utils import log

logger = log.get_logger(__name__)

CONFIG_FILE = "config.yaml"

DEFAULT_CONFIG = str(resource_string(__name__, CONFIG_FILE).decode("utf-8"))


class ObservableConfig(DefaultConfig, Observable):

    keys_requiring_state_machine_refresh = set()
    keys_requiring_restart = set()
    keys_not_to_fill_up = set()

    def __init__(self, default_config_filename, logger_object=None):
        DefaultConfig.__init__(self, default_config_filename, logger_object, rel_config_path='rafcon')
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


class Config(ObservableConfig):
    """ Class to hold and load the global state machine configurations.

    """

    keys_requiring_restart = ()
    keys_not_to_fill_up = {"LIBRARY_PATHS"}

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
