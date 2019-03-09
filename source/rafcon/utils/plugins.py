# Copyright (C) 2016-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Michael Vilzmann <michael.vilzmann@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: plugins
   :synopsis: This is a module which can load all plugins specified in the RAFCON_PLUGIN_PATH environment variable and
            can run all pre- and post init methods of all registered plugins

"""

from builtins import str
import sys
import os
import importlib

from rafcon.utils import log
logger = log.get_logger(__name__)


plugin_dict = {}


def load_plugins():
    """Loads all plugins specified in the RAFCON_PLUGIN_PATH environment variable
    """
    plugins = os.environ.get('RAFCON_PLUGIN_PATH', '')
    plugin_list = set(plugins.split(os.pathsep))
    global plugin_dict
    for plugin_path in plugin_list:
        if not plugin_path:
            continue
        plugin_path = os.path.expandvars(os.path.expanduser(plugin_path)).strip()
        if not os.path.exists(plugin_path):
            logger.error("The specified plugin path does not exist: {}".format(plugin_path))
            continue
        dir_name, plugin_name = os.path.split(plugin_path.rstrip('/'))
        logger.info("Found plugin '{}' at {}".format(plugin_name, plugin_path))
        sys.path.insert(0, dir_name)
        if plugin_name in plugin_dict:
            logger.error("Plugin '{}' already loaded".format(plugin_name))
        else:
            try:
                module = importlib.import_module(plugin_name)
                plugin_dict[plugin_name] = module
                logger.info("Successfully loaded plugin '{}'".format(plugin_name))
            except ImportError:
                logger.exception("Could not import plugin '{}':".format(plugin_name))


def run_hook(hook_name, *args, **kwargs):
    """Runs the passed hook on all registered plugins

    The function checks, whether the hook is available in the plugin.

    :param hook_name: Name of the hook, corresponds to the function name being called
    :param args: Arguments
    :param kwargs: Keyword arguments
    """
    for module in plugin_dict.values():
        if hasattr(module, "hooks") and callable(getattr(module.hooks, hook_name, None)):
            getattr(module.hooks, hook_name)(*args, **kwargs)


def run_pre_inits():
    """Runs the pre_init methods of all registered plugins
    """
    run_hook("pre_init")


def run_on_state_machine_execution_finished():
    """Runs the on_state_machine_execution_finished methods of all registered plugins
    """
    run_hook("on_state_machine_execution_finished")


def run_post_inits(setup_config):
    """Runs the post_init methods of all registered plugins

    :param setup_config:
    """
    run_hook("post_init", setup_config)

