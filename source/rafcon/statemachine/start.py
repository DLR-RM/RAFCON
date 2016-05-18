#!/usr/bin/env python

"""
.. module: a module to enable state machine execution from the command line
   :platform: Unix, Windows
   :synopsis: A module to start arbitrary state machines without the GUI and several configurations options

.. moduleauthor:: Sebastian Brunner


"""


import os
import argparse
from os.path import realpath, dirname, join, exists
import signal
import time
from Queue import Empty
import threading
import sys

import rafcon
from yaml_configuration.config import config_path
import rafcon.utils.filesystem as filesystem

from rafcon.statemachine.config import global_config
import rafcon.statemachine.singleton as sm_singletons
from rafcon.statemachine.storage import storage
from rafcon.statemachine.execution.state_machine_execution_engine import StateMachineExecutionEngine
from rafcon.statemachine.enums import StateExecutionState

from rafcon.utils import log
from rafcon.utils import plugins


def state_machine_path(path):
    sm_root_file = join(path, storage.STATEMACHINE_FILE)
    if exists(sm_root_file):
        return path
    else:
        sm_root_file = join(path, storage.STATEMACHINE_FILE_OLD)
        if exists(sm_root_file):
            return path
        raise argparse.ArgumentTypeError("Failed to open {0}: {1} not found in path".format(path,
                                                                                            storage.STATEMACHINE_FILE))


def start_state_machine(setup_config):
    time.sleep(1.0)
    sm = StateMachineExecutionEngine.execute_state_machine_from_path(setup_config['sm_path'],
                                                                     start_state_path=setup_config['start_state_path'],
                                                                     wait_for_execution_finished=False)
    sm_thread = threading.Thread(target=check_for_sm_finished, args=[sm, ])
    sm_thread.start()
    return sm


def check_for_sm_finished(sm):

    # wait for the state machine to start
    while len(sm.execution_history_container[0].history_items) < 1:
        time.sleep(0.1)

    while sm.root_state.state_execution_status is not StateExecutionState.INACTIVE:
        try:
            sm.root_state.concurrency_queue.get(timeout=10.0)
        except Empty:
            pass
        # no logger output here to make it easier for the parser
        print "RAFCON live signal"

    if "twisted" in sys.modules.keys():
        from twisted.internet import reactor
        reactor.callFromThread(reactor.stop)


def start_profiler(logger):
    profiler_run = global_config.get_config_value("PROFILER_RUN", False)
    if profiler_run:
        try:
            import profiling.tracing
            profiler = profiling.tracing.TracingProfiler()
            logger.debug("The profiler has been started")
            profiler.start()
        except ImportError:
            profiler = None
            logger.error("Cannot run profiler due to missing Python package 'profiling'")
        return profiler


def stop_profiler(profiler, logger):
    profiler.stop()

    if global_config.get_config_value("PROFILER_VIEWER", True):
        profiler.run_viewer()

    result_path = global_config.get_config_value("PROFILER_RESULT_PATH")
    if os.path.isdir(os.path.dirname(result_path)):
        import pickle
        result = profiler.result()
        with open(result_path, 'wb') as f:
            pickle.dump((profiler.__class__, result), f, pickle.HIGHEST_PROTOCOL)
        logger.info("The profiler result has been dumped. Run the following command for inspection:")
        logger.info("$ profiling view {}".format(result_path))


if __name__ == '__main__':

    logger = log.get_logger("start_core")
    logger.info("initialize RAFCON ... ")

    # load all plugins specified in the RAFCON_PLUGIN_PATH
    plugins.load_plugins()

    plugins.run_pre_inits()

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

    plugins.run_post_inits(setup_config)
    
    profiler = start_profiler(logger)

    try:
        sm = start_state_machine(setup_config)

        if "twisted" in sys.modules.keys():
            from twisted.internet import reactor
            reactor.run()

        rafcon.statemachine.singleton.state_machine_execution_engine.join()
        logger.info("State machine execution finished!")

    finally:
        if profiler:
            stop_profiler(profiler, logger)

