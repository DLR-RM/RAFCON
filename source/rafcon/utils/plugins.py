"""
.. module:: plugins
   :platform: Unix, Windows
   :synopsis: This is a module which can load all plugins specified in the RAFCON_PLUGIN_PATH environment variable and
            can run all pre- and post init methods of all registered plugins

.. moduleauthor:: Sebastian Brunner


"""

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
        plugin_path = os.path.expandvars(os.path.expanduser(plugin_path))
        if not os.path.exists(plugin_path):
            logger.error("The specified plugin path does not exist: {}".format(plugin_path))
            continue
        dir_name, plugin_name = os.path.split(plugin_path)
        logger.info("Found plugin '{}' at {}".format(plugin_name, plugin_path))
        sys.path.insert(0, dir_name)
        try:
            module = importlib.import_module(plugin_name)
            plugin_dict[plugin_name] = module
        except ImportError as e:
            logger.error("Could not import plugin '{}': {}".format(plugin_name, e))


def run_pre_inits():
    """Runs the pre_init methods of all registered plugins
    """
    for module in plugin_dict.itervalues():
        module.hooks.pre_init()


def run_post_inits(setup_config):
    """Runs the post_init methods of all registered plugins

    :param setup_config:
    """
    for module in plugin_dict.itervalues():
        module.hooks.post_init(setup_config)

