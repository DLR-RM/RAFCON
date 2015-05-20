#!/opt/python/python2.7/bin/python

import logging

import os
import gtk
import signal
import argparse

from awesome_tool.mvc.controllers import MainWindowController

from awesome_tool.utils import log
from awesome_tool.mvc.views.logging import LoggingView
from awesome_tool.mvc.views.main_window import MainWindowView

import awesome_tool.statemachine.singleton as sm_singletons
import awesome_tool.mvc.singleton as mvc_singletons

from awesome_tool.mvc.config import global_gui_config
from awesome_tool.statemachine.config import global_config

from awesome_tool.statemachine.storage.storage import StateMachineStorage
from awesome_tool.statemachine.state_machine import StateMachine
from awesome_tool.statemachine.states.hierarchy_state import HierarchyState


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
    sm_root_file = os.path.join(path, StateMachineStorage.STATEMACHINE_FILE)
    if os.path.exists(sm_root_file):
        return path
    else:
        raise argparse.ArgumentTypeError("Failed to open {0}: {1} not found in path".format(path,
                                                                                StateMachineStorage.STATEMACHINE_FILE))


def config_path(path):
    if not path or path == 'None':
        return None
    if not os.path.isdir(path):
        raise argparse.ArgumentTypeError("{0} is not a valid path".format(path))
    if os.access(path, os.R_OK):
        return path
    else:
        raise argparse.ArgumentTypeError("{0} is not a readable dir".format(path))


if __name__ == '__main__':
    # setup logging view first
    logging_view = LoggingView()
    setup_logger(logging_view)
    # from awesome_tool.utils import log
    logger = log.get_logger("start")
    logger.info("Awesome tool launcher")

    home_path = os.path.expanduser('~')
    if home_path:
        home_path = os.path.join(home_path, ".awesome_tool")
    else:
        home_path = 'None'

    parser = argparse.ArgumentParser(description='Start Awesome tool')

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

    result = parser.parse_args()
    setup_config = vars(result)

    signal.signal(signal.SIGINT, sm_singletons.signal_handler)

    global_config.load(setup_config['config_path'])
    global_gui_config.load(setup_config['gui_config_path'])

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

    # Ensure that the next message is being printed (needed for LN manager to detect finished startup)
    level = logger.level
    logger.setLevel(logging.INFO)
    logger.info("Ready")
    logger.setLevel(level)
    gtk.main()
    logger.debug("Gtk main loop exited!")

    # If there is a running state-machine, wait for it to be finished before exiting
    sm = sm_singletons.state_machine_manager.get_active_state_machine()
    if sm:
        sm.root_state.join()