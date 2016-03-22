#!/usr/bin/env python

"""
.. module:: preemptive_concurrency_state
   :platform: Unix, Windows
   :synopsis: A module to start arbitrary state machines without the GUI and several configurations options

.. moduleauthor:: Sebastian Brunner


"""

import os
import glib
import argparse
import signal
import time
from Queue import Empty
import threading


def check_for_sm_finished(sm, monitoring_manager=None):
    from rafcon.statemachine.enums import StateExecutionState
    while sm.root_state.state_execution_status is not StateExecutionState.INACTIVE:
        try:
            sm.root_state.concurrency_queue.get(timeout=10.0)
        except Empty, e:
            pass
        # no logger output here to make it easier for the parser
        print "RAFCON live signal"

    sm.root_state.join()

    # stop the network_test if the monitoring plugin is enabled
    if monitoring_manager:
        from twisted.internet import reactor
        reactor.callFromThread(reactor.stop)


def start_server(interacting_function, queue_dict):

    from rafcon.utils.constants import RAFCON_TEMP_PATH_STORAGE
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

    from plugins import *

    from rafcon.utils import log
    logger = log.get_logger("start-no-gui")
    logger.info("initialize RAFCON ... ")

    signal.signal(signal.SIGINT, sm_singletons.signal_handler)

    global_config.load(path=".")

    # Initialize libraries
    sm_singletons.library_manager.initialize()

    # Set base path of global storage
    sm_singletons.global_storage.base_path = RAFCON_TEMP_PATH_STORAGE

    [state_machine, version, creation_time] = rafcon.statemachine.singleton.global_storage.\
        load_statemachine_from_path("../../test_scripts/unit_test_state_machines/99_bottles_of_beer_monitoring")
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)

    sm_thread = threading.Thread(target=check_for_sm_finished, args=[state_machine, ])
    sm_thread.start()

    setup_config = dict()
    setup_config["net_config_path"] = os.path.abspath("./server")

    try:
        # check if monitoring plugin is loaded
        from plugins.monitoring.monitoring_manager import global_monitoring_manager
        if global_monitoring_manager.networking_enabled():
            global_monitoring_manager.initialize(setup_config)

            interacting_thread = threading.Thread(target=interacting_function,
                                                  args=[queue_dict, ])
            interacting_thread.start()

            from twisted.internet import reactor
            reactor.run()
    except ImportError, e:
        # plugin not found
        pass

    state_machine.root_state.join()

    rafcon.statemachine.singleton.state_machine_execution_engine.stop()
    logger.info("State machine execution finished!")


def print_object(queue_dict):
    print "dummy prints:"
    print queue_dict


if __name__ == '__main__':
    start_server(print_object, "multiprocessing_queue_dict")

