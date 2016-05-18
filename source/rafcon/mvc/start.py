#!/usr/bin/env python

import logging
import os
import gtk
import signal
from os.path import realpath, dirname, join, expanduser, expandvars, isdir
import sys

import rafcon
from yaml_configuration.config import config_path
from rafcon.utils import log
import rafcon.utils.filesystem as filesystem

from rafcon.statemachine.start import state_machine_path, start_profiler, stop_profiler
from rafcon.statemachine.config import global_config
from rafcon.statemachine.storage import storage
from rafcon.statemachine.state_machine import StateMachine
from rafcon.statemachine.states.hierarchy_state import HierarchyState
import rafcon.statemachine.singleton as sm_singletons

from rafcon.mvc.controllers.main_window import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView
import rafcon.mvc.singleton as mvc_singletons
from rafcon.mvc.config import global_gui_config
from rafcon.mvc.utils import constants
from rafcon.mvc.runtime_config import global_runtime_config

from rafcon.utils import plugins
from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE


def setup_logger():
    import sys
    # Apply defaults to logger of gtkmvc
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)
    stdout = logging.StreamHandler(sys.stdout)
    stdout.setFormatter(logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s"))
    stdout.setLevel(logging.DEBUG)
    logging.getLogger('gtkmvc').addHandler(stdout)


def prepare_plugins():
    # load all plugins specified in the RAFCON_PLUGIN_PATH
    plugins.load_plugins()

    # check if twisted is imported and if so, install the required reactor
    if "twisted" in sys.modules.keys():
        from twisted.internet import gtk2reactor
        # needed for glib.idle_add, and signals
        gtk2reactor.install()

    plugins.run_pre_inits()


if __name__ == '__main__':
    setup_logger()
    prepare_plugins()

    # from rafcon.utils import log
    logger = log.get_logger("start")
    logger.info("RAFCON launcher")

    rafcon_root_path = dirname(realpath(rafcon.__file__))
    if not os.environ.get('RAFCON_PATH', None):
        # set env variable RAFCON_PATH to the root directory of RAFCON
        os.environ['RAFCON_PATH'] = rafcon_root_path

    if not os.environ.get('RAFCON_LIB_PATH', None):
        # set env variable RAFCON_LIB_PATH to the library directory of RAFCON (when not using RMPM)
        os.environ['RAFCON_LIB_PATH'] = join(dirname(rafcon_root_path), 'libraries')

    home_path = filesystem.get_home_path()

    parser = sm_singletons.argument_parser

    parser.add_argument('-n', '--new', action='store_true', help="whether to create a new state-machine")
    parser.add_argument('-o', '--open', action='store', nargs='*', type=state_machine_path, dest='sm_paths',
                        metavar='path', help="specify directories of state-machines that shall be opened. Paths must "
                                             "contain a statemachine.yaml file")
    parser.add_argument('-c', '--config', action='store', type=config_path, metavar='path', dest='config_path',
                        default=home_path, nargs='?', const=home_path,
                        help="path to the configuration file config.yaml. Use 'None' to prevent the generation of "
                             "a config file and use the default configuration. Default: {0}".format(home_path))
    parser.add_argument('-g', '--gui_config', action='store', type=config_path, metavar='path', dest='gui_config_path',
                        default=home_path, nargs='?', const=home_path,
                        help="path to the configuration file gui_config.yaml. Use 'None' to prevent the generation of "
                             "a config file and use the default configuration. Default: {0}".format(home_path))

    result = parser.parse_args()
    setup_config = vars(result)

    # Make mvc directory the working directory
    # Needed for views, which assume to be in the mvc path and import glade files relatively
    os.chdir(join(rafcon_root_path, 'mvc'))

    # create lock file
    if global_gui_config.get_config_value('AUTO_RECOVERY_LOCK_ENABLED'):
        constants.RAFCON_INSTANCE_LOCK_FILE = open(os.path.join(RAFCON_TEMP_PATH_BASE, 'lock'), 'a+')
        constants.RAFCON_INSTANCE_LOCK_FILE.close()

    # Create the GUI-View
    main_window_view = MainWindowView()

    signal.signal(signal.SIGINT, sm_singletons.signal_handler)

    # load configuration files
    global_config.load(path=setup_config['config_path'])
    global_gui_config.load(path=setup_config['gui_config_path'])
    global_runtime_config.load(path=setup_config['gui_config_path'])

    # Initialize library
    sm_singletons.library_manager.initialize()

    if setup_config['sm_paths']:
        for path in setup_config['sm_paths']:
            try:
                state_machine = storage.load_state_machine_from_path(path)
                sm_singletons.state_machine_manager.add_state_machine(state_machine)
            except Exception as e:
                logger.exception("Could not load state-machine {0}".format(path))

    if setup_config['new']:
        root_state = HierarchyState()
        state_machine = StateMachine(root_state)
        sm_singletons.state_machine_manager.add_state_machine(state_machine)

    sm_manager_model = mvc_singletons.state_machine_manager_model

    main_window_controller = MainWindowController(sm_manager_model, main_window_view, editor_type='LogicDataGrouped')

    # Ensure that the next message is being printed (needed for LN manager to detect finished startup)
    level = logger.level
    logger.setLevel(logging.INFO)
    logger.info("Ready")
    logger.setLevel(level)

    plugins.run_post_inits(setup_config)

    profiler = start_profiler(logger)

    try:
        # check if twisted is imported
        if "twisted" in sys.modules.keys():
            from twisted.internet import reactor
            reactor.run()
        else:
            gtk.main()

        logger.info("Joined root state")

        # If there is a running state-machine, wait for it to be finished before exiting
        sm = sm_singletons.state_machine_manager.get_active_state_machine()
        if sm:
            sm.root_state.join()

    finally:
        if profiler:
            stop_profiler(profiler, logger)
        if global_gui_config.get_config_value('AUTO_RECOVERY_LOCK_ENABLED'):
            if os.path.exists(constants.RAFCON_INSTANCE_LOCK_FILE.name):
                os.remove(constants.RAFCON_INSTANCE_LOCK_FILE.name)
            else:
                logger.warning("External remove of lock file detected!")

    logger.info("Exiting ...")

    # this is a ugly process shutdown method but works if gtk or twisted process are still blocking
    import os
    os._exit(0)

