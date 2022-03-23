# Copyright (C) 2014-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Sebastian Brunner <sebastian.brunner@agile-robots.de>

"""
.. module:: config
   :synopsis: Config module to specify global design configuration values

"""

from pkg_resources import resource_string
from yaml_configuration.config import ConfigError
from rafcon.core.config import ObservableConfig
from rafcon.utils import log
logger = log.get_logger(__name__)

CONFIG_FILE = "design_config.yaml"

DEFAULT_CONFIG = str(resource_string(__name__, CONFIG_FILE).decode("utf-8"))


class DesignConfig(ObservableConfig):
    """ Class to hold and load the global state machine configurations.

    """

    keys_requiring_restart = ()

    def __init__(self, logger_object=None):
        """Default constructor

        :param logger_object: the logger object to pass the log output to
        :raises ConfigError: if the config type is not given in the config file
        """
        super(DesignConfig, self).__init__(DEFAULT_CONFIG, logger_object)
        if self.get_config_value("TYPE") != "DESIGN_CONFIG":
            raise ConfigError("Type should be SM_CONFIG for state machine configuration. "
                              "Please add \"TYPE: SM_CONFIG\" to your config.yaml file.")

    def load(self, config_file=None, path=None):
        """Loads the configuration from a specific file

        :param config_file: the name of the config file
        :param path: the path to the config file
        """
        if config_file is None:
            config_file = CONFIG_FILE
        super(DesignConfig, self).load(config_file, path)

    def save_configuration(self):
        super(DesignConfig, self).save_configuration()


# This variable holds the global configuration parameters for the state machine
global_design_config = DesignConfig(logger)


def is_custom_design_enabled():
    """ Checks if a custom design is enabled

    """
    return global_design_config.get_config_value("USE_CUSTOM_DESIGN", False)
