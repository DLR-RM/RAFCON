#!/opt/python/python2.7/bin/python

from rafcon.utils import log
logger = log.get_logger("start-no-gui")
logger.info("initialize RAFCON ... ")

# from twisted.internet import gtk2reactor
# gtk2reactor.install()
# from twisted.internet import reactor

import logging
import os
import gtk
import argparse
from os.path import realpath, dirname, join, exists, expanduser, expandvars, isdir
import signal
import time
from Queue import Empty

import rafcon
from rafcon.mvc.controllers import MainWindowController
from rafcon.mvc.views.logging import LoggingView
from rafcon.mvc.views.main_window import MainWindowView
import rafcon.mvc.singleton as mvc_singletons
from rafcon.mvc.config import global_gui_config
from rafcon.mvc.runtime_config import global_runtime_config

from rafcon.statemachine.config import global_config
import rafcon.statemachine.singleton as sm_singletons
from rafcon.statemachine.storage.storage import StateMachineStorage
from rafcon.statemachine.state_machine import StateMachine
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.execution.statemachine_execution_engine import StatemachineExecutionEngine
from rafcon.statemachine.enums import StateExecutionState

from rafcon.network.network_config import global_net_config


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


def state_machine_path(path):
    sm_root_file = join(path, StateMachineStorage.STATEMACHINE_FILE)
    if exists(sm_root_file):
        return path
    else:
        raise argparse.ArgumentTypeError("Failed to open {0}: {1} not found in path".format(path,
                                                                                StateMachineStorage.STATEMACHINE_FILE))


if __name__ == '__main__':

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

    logger.info("parse arguments ... ")
    parser = argparse.ArgumentParser(description='Start RAFCON')
    parser.add_argument('-o', '--open', action='store', type=state_machine_path, dest='sm_path', metavar='path',
                        help="specify a directory of a state-machine that shall be opened and started. The path must contain a "
                             "statemachine.yaml file")
    parser.add_argument('-c', '--config', action='store', type=config_path, metavar='path', dest='config_path',
                        default=home_path, nargs='?', const=home_path,
                        help="path to the configuration file config.yaml. Use 'None' to prevent the generation of "
                             "a config file and use the default configuration. Default: {0}".format(home_path))

    result = parser.parse_args()
    setup_config = vars(result)

    if not setup_config['sm_path']:
        logger.error("You have to specify a valid state machine path")
        exit(-1)

    signal.signal(signal.SIGINT, sm_singletons.signal_handler)

    global_config.load(path=setup_config['config_path'])

    # Initialize libraries
    sm_singletons.library_manager.initialize()

    # Set base path of global storage
    sm_singletons.global_storage.base_path = "/tmp"

    sm = StatemachineExecutionEngine.execute_state_machine_from_path(setup_config['sm_path'],
                                                                     wait_for_execution_finished=False)

    while sm.root_state.state_execution_status is not StateExecutionState.INACTIVE:
        try:
            sm.root_state.concurrency_queue.get(timeout=1.0)
        except Empty, e:
            pass
        # no logger output here to make it easier for the parser
        print "RAFCON live signal"

    sm.root_state.join()
    rafcon.statemachine.singleton.state_machine_execution_engine.stop()
    logger.info("State machine execution finished!")

