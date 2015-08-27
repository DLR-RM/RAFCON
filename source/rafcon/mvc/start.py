#!/opt/python/python2.7/bin/python

from twisted.internet import gtk2reactor
gtk2reactor.install()
from twisted.internet import reactor

import logging
import os
import gtk
import signal
import argparse
from os.path import realpath, dirname, join, exists, expanduser, expandvars, isdir

import rafcon
from rafcon.mvc.controllers import MainWindowController

from rafcon.utils import log
from rafcon.mvc.views.logging import LoggingView
from rafcon.mvc.views.main_window import MainWindowView
import rafcon.statemachine.singleton as sm_singletons
import rafcon.mvc.singleton as mvc_singletons

from rafcon.network.network_config import global_net_config
from rafcon.mvc.config import global_gui_config
from rafcon.mvc.runtime_config import global_runtime_config
from rafcon.statemachine.config import global_config

from rafcon.statemachine.storage.storage import StateMachineStorage
from rafcon.statemachine.state_machine import StateMachine
from rafcon.statemachine.states.hierarchy_state import HierarchyState


def setup_logger(logging_view):
    import sys
    # Set the views for the loggers
    log.debug_filter.set_logging_test_view(logging_view)
    log.error_filter.set_logging_test_view(logging_view)

    # Apply defaults to logger of gtkmvc
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)
    stdout = logging.StreamHandler(sys.stdout)
    stdout.setFormatter(logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s"))
    stdout.setLevel(logging.DEBUG)
    logging.getLogger('gtkmvc').addHandler(stdout)

    # Set logging level
    # logging.getLogger('statemachine.state').setLevel(logging.DEBUG)
    # logging.getLogger('controllers.state_properties').setLevel(logging.DEBUG)


def state_machine_path(path):
    sm_root_file = join(path, StateMachineStorage.STATEMACHINE_FILE)
    if exists(sm_root_file):
        return path
    else:
        raise argparse.ArgumentTypeError("Failed to open {0}: {1} not found in path".format(path,
                                                                                StateMachineStorage.STATEMACHINE_FILE))


def config_path(path):
    if not path or path == 'None':
        return None
    # replace ~ with /home/user
    path = expanduser(path)
    # e.g. replace ${RAFCON_PATH} with the root path of RAFCON
    path = expandvars(path)
    if not isdir(path):
        raise argparse.ArgumentTypeError("{0} is not a valid path".format(path))
    if os.access(path, os.R_OK):
        return path
    else:
        raise argparse.ArgumentTypeError("{0} is not a readable dir".format(path))


if __name__ == '__main__':
    # setup logging view first
    logging_view = LoggingView()
    setup_logger(logging_view)
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

    home_path = expanduser('~')
    if home_path:
        home_path = join(home_path, ".config", "rafcon")
    else:
        home_path = 'None'

    parser = argparse.ArgumentParser(description='Start RAFCON')

    parser.add_argument('-n', '--new', action='store_true', help="whether to create a new state-machine")
    parser.add_argument('-o', '--open', action='store', nargs='*', type=state_machine_path, dest='sm_paths',
                        metavar='path',
                        help="specify directories of state-machines that shall be opened. Paths must contain a "
                             "statemachine.yaml file")
    parser.add_argument('-c', '--config', action='store', type=config_path, metavar='path', dest='config_path',
                        default=home_path, nargs='?', const=home_path,
                        help="path to the configuration file config.yaml. Use 'None' to prevent the generation of "
                             "a config file and use the default configuration. Default: {0}".format(home_path))
    parser.add_argument('-g', '--gui_config', action='store', type=config_path, metavar='path', dest='gui_config_path',
                        default=home_path, nargs='?', const=home_path,
                        help="path to the configuration file gui_config.yaml. Use 'None' to prevent the generation of "
                             "a config file and use the default configuration. Default: {0}".format(home_path))
    parser.add_argument('-i', '--net_config', action='store', type=config_path, metavar='path', dest='net_config_path',
                        default=home_path, nargs='?', const=home_path,
                        help="path to the configuration file net_config.yaml. Use 'None' to prevent the generation of "
                             "a config file and use the default configuration. Default: {0}".format(home_path))

    result = parser.parse_args()
    setup_config = vars(result)

    signal.signal(signal.SIGINT, sm_singletons.signal_handler)

    global_config.load(path=setup_config['config_path'])
    global_gui_config.load(path=setup_config['gui_config_path'])
    global_net_config.load(path=setup_config['net_config_path'])
    global_runtime_config.load(path=setup_config['gui_config_path'])

    # Make mvc directory the working directory
    # Needed for views, which assume to be in the mvc path and import glade files relatively
    os.chdir(join(rafcon_root_path, 'mvc'))

    # Initialize library
    sm_singletons.library_manager.initialize()

    # Set base path of global storage
    sm_singletons.global_storage.base_path = "/tmp"

    # Create the GUI
    main_window_view = MainWindowView(logging_view)

    if setup_config['sm_paths']:
        storage = StateMachineStorage()
        for path in setup_config['sm_paths']:
            try:
                state_machine, version, creation_time = storage.load_statemachine_from_yaml(path)
                sm_singletons.state_machine_manager.add_state_machine(state_machine)
            except Exception as e:
                logger.error("Could not load state-machine {0}: {1}".format(path, e))

    if setup_config['new']:
        root_state = HierarchyState()
        state_machine = StateMachine(root_state)
        sm_singletons.state_machine_manager.add_state_machine(state_machine)

    sm_manager_model = mvc_singletons.state_machine_manager_model

    main_window_controller = MainWindowController(sm_manager_model, main_window_view, editor_type="LogicDataGrouped")
    main_window = main_window_view.get_top_widget()
    size = global_runtime_config.get_config_value("WINDOW_SIZE", None)
    position = global_runtime_config.get_config_value("WINDOW_POS", None)
    position = (max(0, position[0]), max(0, position[1]))
    if size:
        main_window.resize(size[0], size[1])
    if position:
        screen_width = gtk.gdk.screen_width()
        screen_height = gtk.gdk.screen_height()
        if position[0] < screen_width and position[1] < screen_height:
            main_window.move(position[0], position[1])


# Ensure that the next message is being printed (needed for LN manager to detect finished startup)
    level = logger.level
    logger.setLevel(logging.INFO)
    logger.info("Ready")
    logger.setLevel(level)
    reactor.run()
    gtk.main()
    logger.debug("Gtk main loop exited!")

    # If there is a running state-machine, wait for it to be finished before exiting
    sm = sm_singletons.state_machine_manager.get_active_state_machine()
    if sm:
        sm.root_state.join()