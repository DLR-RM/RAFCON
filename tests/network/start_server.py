"""
.. module:: start server
   :platform: Unix, Windows
   :synopsis: A module to start an unit test RAFCON server instance

.. moduleauthor:: Sebastian Brunner


"""
import os
import signal
from queue import Empty
import threading


def check_for_sm_finished(sm, monitoring_manager=None):
    from rafcon.core.states.state import StateExecutionStatus
    while sm.root_state.state_execution_status is not StateExecutionStatus.INACTIVE:
        try:
            sm.root_state.concurrency_queue.get(timeout=10.0)
        except Empty as e:
            pass
        # no logger output here to make it easier for the parser
        print("RAFCON live signal")

    sm.root_state.join()

    # stop the network if the monitoring plugin is enabled
    if monitoring_manager:
        from twisted.internet import reactor
        reactor.callFromThread(reactor.stop)


def start_server(interacting_function, queue_dict):
    import sys
    import os

    import rafcon
    from rafcon.utils import log
    from rafcon.utils import plugins

    from rafcon.core.config import global_config
    import rafcon.core.singleton as core_singletons
    from rafcon.core.storage import storage as global_storage
    # needed for yaml parsing
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState
    from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState

    from rafcon.core.start import signal_handler, register_signal_handlers
    register_signal_handlers(signal_handler)

    logger = log.get_logger("start-no-gui")
    logger.info("initialize RAFCON ... ")

    plugins.load_plugins()
    plugins.run_pre_inits()

    global_config.load(path=os.path.dirname(os.path.abspath(__file__)))

    # Initialize libraries
    core_singletons.library_manager.initialize()

    from tests import utils as testing_utils
    state_machine = global_storage.load_state_machine_from_path(
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "99_bottles_of_beer_monitoring")))
    sm_id = rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)

    sm_thread = threading.Thread(target=check_for_sm_finished, args=[state_machine, ])
    sm_thread.start()

    setup_config = dict()
    setup_config["net_config_path"] = os.path.abspath(path=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                        "server"))

    plugins.run_post_inits(setup_config)

    if "twisted" in sys.modules:
        print("################# twisted found #######################")
        interacting_thread = threading.Thread(target=interacting_function,
                                              args=[queue_dict, sm_id])
        interacting_thread.start()
        from twisted.internet import reactor
        reactor.run()
    else:
        logger.error("Server: Twisted is not in sys.modules or twisted is not working! Exiting program ... !")
        import os
        os._exit(0)

    state_machine.root_state.join()

    rafcon.core.singleton.state_machine_execution_engine.stop()
    logger.info("State machine execution finished!")


def print_object(queue_dict):
    print("dummy prints:")
    print(queue_dict)


if __name__ == '__main__':
    start_server(print_object, "multiprocessing_queue_dict")

