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

from rafcon.utils import plugins
from rafcon.utils import log
logger = log.get_logger("start core")


def pre_setup_plugins():
    """Loads plugins and calls the pre init hooks
    """
    plugins.load_plugins()
    plugins.run_pre_inits()


def post_setup_plugins(parser_result):
    """Calls the post init hubs

    :param dict parser_result: Dictionary with the parsed arguments
    """
    if not isinstance(parser_result, dict):
        parser_result = vars(parser_result)
    plugins.run_post_inits(parser_result)


def setup_environment():
    """Ensures that the environmental variables RAFCON_PATH and RAFCON_LIB_PATH are existent
    """
    rafcon_root_path = dirname(realpath(rafcon.__file__))
    if not os.environ.get('RAFCON_PATH', None):
        # set env variable RAFCON_PATH to the root directory of RAFCON
        os.environ['RAFCON_PATH'] = rafcon_root_path

    if not os.environ.get('RAFCON_LIB_PATH', None):
        # set env variable RAFCON_LIB_PATH to the library directory of RAFCON (when not using RMPM)
        os.environ['RAFCON_LIB_PATH'] = join(dirname(rafcon_root_path), 'libraries')


def parse_state_machine_path(path):
    """Parser for argparse checking pfor a proper state machine path

    :param str path: Input path from the user
    :return: The path
    :raises argparse.ArgumentTypeError: if the path does not contain a statemachine.json file
    """
    sm_root_file = join(path, storage.STATEMACHINE_FILE)
    if exists(sm_root_file):
        return path
    else:
        sm_root_file = join(path, storage.STATEMACHINE_FILE_OLD)
        if exists(sm_root_file):
            return path
        raise argparse.ArgumentTypeError("Failed to open {0}: {1} not found in path".format(path,
                                                                                            storage.STATEMACHINE_FILE))


def setup_argument_parser():
    """Sets up teh parser with the required arguments

    :return: The parser object
    """
    home_path = filesystem.get_home_path()

    parser = sm_singletons.argument_parser
    parser.add_argument('-o', '--open', type=parse_state_machine_path, dest='state_machine_path', metavar='path',
                        help="specify a directory of a state-machine that shall be opened and started. The path must "
                             "contain a statemachine.json file")
    parser.add_argument('-c', '--config', type=config_path, metavar='path', dest='config_path', default=home_path,
                        nargs='?', const=home_path,
                        help="path to the configuration file config.yaml. Use 'None' to prevent the generation of "
                             "a config file and use the default configuration. Default: {0}".format(home_path))
    parser.add_argument('-s', '--start_state_path', metavar='path', dest='start_state_path',
                        default=None, nargs='?', help="path of to the state that should be launched")
    return parser


def setup_configuration(config_path):
    """Loads the core configuration from the specified path and uses its content for further setup

    :param config_path: Path to the core config file
    """
    global_config.load(path=config_path)

    # Initialize libraries
    sm_singletons.library_manager.initialize()


def start_state_machine(state_machine_path, start_state_path=None):
    """Executes the specified state machine

    :param str state_machine_path: The file path to the state machine
    :param str start_state_path: The state path to the desired first state
    :return StateMachine: The loaded state machine
    """
    state_machine = StateMachineExecutionEngine.execute_state_machine_from_path(path=state_machine_path,
                                                                                start_state_path=start_state_path,
                                                                                wait_for_execution_finished=False)

    if reactor_required():
        sm_thread = threading.Thread(target=stop_reactor_on_state_machine_finish, args=[state_machine, ])
        sm_thread.start()
    return state_machine


def stop_reactor_on_state_machine_finish(state_machine):

    # wait for the state machine to start
    while len(state_machine.execution_history_container.execution_histories[0].history_items) < 1:
        time.sleep(0.1)

    while state_machine.root_state.state_execution_status is not StateExecutionState.INACTIVE:
        try:
            state_machine.root_state.concurrency_queue.get(timeout=10)
        except Empty:
            pass
        # no logger output here to make it easier for the parser
        print "RAFCON live signal"

    if reactor_required():
        from twisted.internet import reactor
        reactor.callFromThread(reactor.stop)


def reactor_required():
    if "twisted" in sys.modules.keys():
        return True
    return False


def start_profiler():
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


def stop_profiler(profiler):
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

    signal.signal(signal.SIGINT, sm_singletons.signal_handler)

    logger.info("initialize RAFCON ... ")

    pre_setup_plugins()

    setup_environment()

    logger.info("parse arguments ... ")
    parser = setup_argument_parser()
    user_input = parser.parse_args()
    if not user_input.state_machine_path:
        logger.error("You have to specify a valid state machine path")
        exit(-1)

    setup_configuration(user_input.config_path)

    post_setup_plugins(user_input)

    profiler = start_profiler()
    try:

        sm = start_state_machine(user_input.state_machine_path, user_input.start_state_path)

        if reactor_required():
            from twisted.internet import reactor
            # Blocking call, return when state machine execution finishes
            reactor.run()

        rafcon.statemachine.singleton.state_machine_execution_engine.join()
        logger.info("State machine execution finished!")
    finally:
        if profiler:
            stop_profiler(profiler)
