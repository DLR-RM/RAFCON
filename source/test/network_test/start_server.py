"""
.. module:: start server
   :platform: Unix, Windows
   :synopsis: A module to start an unit test RAFCON server instance

.. moduleauthor:: Sebastian Brunner


"""

import os
import signal
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
    import sys
    import os
    from rafcon.utils.constants import RAFCON_TEMP_PATH_STORAGE
    import rafcon
    from yaml_configuration.config import config_path
    import rafcon.utils.filesystem as filesystem

    from rafcon.statemachine.config import global_config
    import rafcon.statemachine.singleton as sm_singletons
    from rafcon.statemachine.storage import storage as global_storage
    # needed for yaml parsing
    from rafcon.statemachine.states.hierarchy_state import HierarchyState
    from rafcon.statemachine.states.execution_state import ExecutionState
    from rafcon.statemachine.states.preemptive_concurrency_state import PreemptiveConcurrencyState
    from rafcon.statemachine.states.barrier_concurrency_state import BarrierConcurrencyState
    from rafcon.statemachine.execution.statemachine_execution_engine import StatemachineExecutionEngine

    from rafcon.utils import log
    from rafcon.utils import plugins
    logger = log.get_logger("start-no-gui")
    logger.info("initialize RAFCON ... ")

    plugins.load_plugins()
    plugins.run_pre_inits()

    signal.signal(signal.SIGINT, sm_singletons.signal_handler)

    global_config.load(path=os.path.dirname(os.path.abspath(__file__)))

    # Initialize libraries
    sm_singletons.library_manager.initialize()

    state_machine = global_storage.load_state_machine_from_path(
            rafcon.__path__[0] + "/../test_scripts/unit_test_state_machines/99_bottles_of_beer_monitoring")
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)

    sm_thread = threading.Thread(target=check_for_sm_finished, args=[state_machine, ])
    sm_thread.start()

    setup_config = dict()
    setup_config["net_config_path"] = os.path.abspath(path=os.path.dirname(os.path.abspath(__file__))+"/server")

    plugins.run_post_inits(setup_config)

    if "twisted" in sys.modules.keys():
        interacting_thread = threading.Thread(target=interacting_function,
                                              args=[queue_dict, ])
        interacting_thread.start()
        from twisted.internet import reactor
        reactor.run()
    else:
        logger.error("Something went seriously wrong!")
        import os
        os._exit(0)

    state_machine.root_state.join()

    rafcon.statemachine.singleton.state_machine_execution_engine.stop()
    logger.info("State machine execution finished!")


def print_object(queue_dict):
    print "dummy prints:"
    print queue_dict


if __name__ == '__main__':
    start_server(print_object, "multiprocessing_queue_dict")

