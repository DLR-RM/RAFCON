#!/usr/bin/env python

"""
.. module:: preemptive_concurrency_state
   :platform: Unix, Windows
   :synopsis: A module to start arbitrary state machines without the GUI and several configurations options

.. moduleauthor:: Sebastian Brunner


"""

from rafcon.utils.constants import RAFCON_TEMP_PATH_STORAGE

import os
import glib
import argparse
from os.path import realpath, dirname, join, exists
import signal
import time
from Queue import Empty
import threading

import rafcon
from rafcon.utils.config import config_path
import rafcon.utils.filesystem as filesystem

from rafcon.statemachine.config import global_config
import rafcon.statemachine.singleton as sm_singletons
from rafcon.statemachine.storage.storage import StateMachineStorage
# needed for yaml parsing
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from rafcon.statemachine.states.barrier_concurrency_state import BarrierConcurrencyState
from rafcon.statemachine.execution.statemachine_execution_engine import StatemachineExecutionEngine
from rafcon.statemachine.enums import StateExecutionState

from plugins import *

from rafcon.utils import log
logger = log.get_logger("start-no-gui")
logger.info("initialize RAFCON ... ")


def state_machine_path(path):
    sm_root_file = join(path, StateMachineStorage.STATEMACHINE_FILE)
    if exists(sm_root_file):
        return path
    else:
        raise argparse.ArgumentTypeError("Failed to open {0}: {1} not found in path".format(path,
                                                                                StateMachineStorage.STATEMACHINE_FILE))


def start_state_machine(setup_config):
    time.sleep(1.0)
    sm = StatemachineExecutionEngine.execute_state_machine_from_path(setup_config['sm_path'],
                                                                     start_state_path=setup_config['start_state_path'],
                                                                     wait_for_execution_finished=False)
    sm_thread = threading.Thread(target=check_for_sm_finished, args=[sm, ])
    sm_thread.start()
    return sm


def check_for_sm_finished(sm, monitoring_manager=None):
    while sm.root_state.state_execution_status is not StateExecutionState.INACTIVE:
        try:
            sm.root_state.concurrency_queue.get(timeout=10.0)
        except Empty, e:
            pass
        # no logger output here to make it easier for the parser
        print "RAFCON live signal"

    sm.root_state.join()

    # stop the networking if the monitoring plugin is enabled
    if monitoring_manager:
        reactor.callFromThread(reactor.stop)


if __name__ == '__main__':

    rafcon_root_path = dirname(realpath(rafcon.__file__))
    if not os.environ.get('RAFCON_PATH', None):
        # set env variable RAFCON_PATH to the root directory of RAFCON
        os.environ['RAFCON_PATH'] = rafcon_root_path

    if not os.environ.get('RAFCON_LIB_PATH', None):
        # set env variable RAFCON_LIB_PATH to the library directory of RAFCON (when not using RMPM)
        os.environ['RAFCON_LIB_PATH'] = join(dirname(rafcon_root_path), 'libraries')

    home_path = filesystem.get_home_path()

    logger.info("parse arguments ... ")
    parser = sm_singletons.argument_parser
    parser.add_argument('-o', '--open', action='store', type=state_machine_path, dest='sm_path', metavar='path',
                        help="specify a directory of a state-machine that shall be opened and started. The path must contain a "
                             "statemachine.yaml file")
    parser.add_argument('-c', '--config', action='store', type=config_path, metavar='path', dest='config_path',
                        default=home_path, nargs='?', const=home_path,
                        help="path to the configuration file config.yaml. Use 'None' to prevent the generation of "
                             "a config file and use the default configuration. Default: {0}".format(home_path))
    parser.add_argument('-s', '--start_state_path', action='store', metavar='path', dest='start_state_path',
                        default=None, nargs='?', help="path of to the state that should be launched")

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
    sm_singletons.global_storage.base_path = RAFCON_TEMP_PATH_STORAGE

    sm = start_state_machine(setup_config)

    try:
        # check if monitoring plugin is loaded
        from plugins.monitoring.monitoring_manager import global_monitoring_manager
        if global_monitoring_manager.networking_enabled():
            global_monitoring_manager.initialize(setup_config)
            from twisted.internet import reactor
            reactor.run()
    except ImportError, e:
        # plugin not found
        pass

    sm.root_state.join()

    rafcon.statemachine.singleton.state_machine_execution_engine.stop()
    logger.info("State machine execution finished!")

