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
    """
    Loads all plugins specified in the RAFCON_PLUGIN_PATH environment variable
    :return:
    """
    plugins = os.environ.get('RAFCON_PLUGIN_PATH', None)
    if not plugins:
        return
    plugin_list = plugins.split(":")
    global plugin_dict
    for plugin in plugin_list:
        if plugin == "":
            continue
        logger.info(plugin)
        dir_name, plugin_name = os.path.split(plugin)
        sys.path.append(dir_name)
        i = importlib.import_module(plugin_name)
        plugin_dict[plugin_name] = i


def run_pre_inits():
    """
    Runs the pre_init methods of all registered plugins
    :return:
    """
    global plugin_dict
    for plugin, module in plugin_dict.iteritems():
        # print dir(module)
        # print module.__file__
        module.hooks.pre_init()


def run_post_inits(setup_config):
    """
    Runs the post_init methods of all registered plugins
    :param setup_config:
    :return:
    """
    global plugin_dict
    for plugin, module in plugin_dict.iteritems():
        module.hooks.post_init(setup_config)

