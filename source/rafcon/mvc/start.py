#!/usr/bin/env python

import os
import logging
import signal
import gtk
from os.path import realpath, dirname, join
from yaml_configuration.config import config_path

import rafcon
from rafcon.statemachine.start import parse_state_machine_path, start_profiler, stop_profiler, setup_environment, \
    reactor_required, setup_configuration, post_setup_plugins
from rafcon.statemachine.storage import storage
from rafcon.statemachine.state_machine import StateMachine
from rafcon.statemachine.states.hierarchy_state import HierarchyState
import rafcon.statemachine.singleton as sm_singletons

import rafcon.mvc.singleton as mvc_singletons
from rafcon.mvc.controllers.main_window import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView
from rafcon.mvc.config import global_gui_config
from rafcon.mvc.runtime_config import global_runtime_config
from rafcon.mvc.utils import constants

import rafcon.utils.filesystem as filesystem
from rafcon.utils import plugins
from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE

from rafcon.utils import log
logger = log.get_logger("start")


def setup_gtkmvc_logger():
    import sys
    # Apply defaults to logger of gtkmvc
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)
    stdout = logging.StreamHandler(sys.stdout)
    stdout.setFormatter(logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s"))
    stdout.setLevel(logging.DEBUG)
    logging.getLogger('gtkmvc').addHandler(stdout)


def pre_setup_plugins():
    """Loads plugins and calls the pre init hooks

    If twisted has been imported by a plugin, the gtk2reactor is installed
    """
    # load all plugins specified in the RAFCON_PLUGIN_PATH
    plugins.load_plugins()

    # check if twisted is imported and if so, install the required reactor
    if reactor_required():
        from twisted.internet import gtk2reactor
        # needed for glib.idle_add, and signals
        gtk2reactor.install()

    plugins.run_pre_inits()


def setup_mvc_environment():
    setup_environment()


def setup_argument_parser():
    """Sets up teh parser with the required arguments

    :return: The parser object
    """
    home_path = filesystem.get_home_path()

    parser = sm_singletons.argument_parser
    parser.add_argument('-n', '--new', action='store_true', help="whether to create a new state-machine")
    parser.add_argument('-o', '--open', action='store', nargs='*', type=parse_state_machine_path,
                        dest='state_machine_paths', metavar='path', help="specify directories of state-machines that "
                        "shall be opened. Paths must contain a statemachine.yaml file")
    parser.add_argument('-c', '--config', action='store', type=config_path, metavar='path', dest='config_path',
                        default=home_path, nargs='?', const=home_path,
                        help="path to the configuration file config.yaml. Use 'None' to prevent the generation of "
                             "a config file and use the default configuration. Default: {0}".format(home_path))
    parser.add_argument('-g', '--gui_config', action='store', type=config_path, metavar='path', dest='gui_config_path',
                        default=home_path, nargs='?', const=home_path,
                        help="path to the configuration file gui_config.yaml. Use 'None' to prevent the generation of "
                             "a config file and use the default configuration. Default: {0}".format(home_path))
    return parser


def setup_mvc_configuration(core_config_path, gui_config_path, runtime_config_path):
    setup_configuration(core_config_path)
    global_gui_config.load(gui_config_path)
    global_runtime_config.load(runtime_config_path)


def setup_gui():
    # Create the GUI-View
    main_window_view = MainWindowView()
    sm_manager_model = mvc_singletons.state_machine_manager_model
    main_window_controller = MainWindowController(sm_manager_model, main_window_view, editor_type='LogicDataGrouped')
    return main_window_controller


def open_state_machines(paths):
    for path in paths:
        try:
            state_machine = storage.load_state_machine_from_path(path)
            sm_singletons.state_machine_manager.add_state_machine(state_machine)
        except Exception as e:
            logger.exception("Could not load state machine '{}': {}".format(path, e))


def create_new_state_machine():
    root_state = HierarchyState()
    state_machine = StateMachine(root_state)
    sm_singletons.state_machine_manager.add_state_machine(state_machine)


def log_ready_output():
    # Ensure that the next message is being printed (needed for LN manager to detect finished startup)
    level = logger.level
    logger.setLevel(logging.INFO)
    logger.info("Ready")
    logger.setLevel(level)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, sm_singletons.signal_handler)

    setup_gtkmvc_logger()
    pre_setup_plugins()

    # from rafcon.utils import log
    logger.info("RAFCON launcher")

    setup_mvc_environment()

    parser = setup_argument_parser()
    user_input = parser.parse_args()

    # create lock file
    if global_gui_config.get_config_value('AUTO_RECOVERY_LOCK_ENABLED'):
        constants.RAFCON_INSTANCE_LOCK_FILE = open(os.path.join(RAFCON_TEMP_PATH_BASE, 'lock'), 'a+')
        constants.RAFCON_INSTANCE_LOCK_FILE.close()

    setup_mvc_configuration(user_input.config_path, user_input.gui_config_path, user_input.gui_config_path)

    if user_input.state_machine_paths:
        open_state_machines(user_input.state_machine_paths)

    if user_input.new:
        create_new_state_machine()

    main_window_controller = setup_gui()

    post_setup_plugins(user_input)

    log_ready_output()

    profiler = start_profiler()
    try:
        # check if twisted is imported
        if reactor_required():
            from twisted.internet import reactor
            reactor.run()
        else:
            gtk.main()

        logger.info("Main window was closed")

        # If there is a running state-machine, wait for it to be finished before exiting
        sm = sm_singletons.state_machine_manager.get_active_state_machine()
        if sm:
            sm.root_state.join()

    finally:
        plugins.run_hook("post_main_window_destruction")

        if profiler:
            stop_profiler(profiler)
        if global_gui_config.get_config_value('AUTO_RECOVERY_LOCK_ENABLED'):
            if os.path.exists(constants.RAFCON_INSTANCE_LOCK_FILE.name):
                os.remove(constants.RAFCON_INSTANCE_LOCK_FILE.name)
            else:
                logger.warning("External remove of lock file detected!")

    logger.info("Exiting ...")

    # this is a ugly process shutdown method but works if gtk or twisted process are still blocking
    import os
    os._exit(0)

