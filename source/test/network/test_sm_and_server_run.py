import test_utils

# system libs
from multiprocessing import Process, Queue
import os
import glib
import time
from Queue import Empty


from rafcon.statemachine.enums import StateExecutionState


setup_config = dict()
setup_config['sm_path'] = "../../test_scripts/unit_test_state_machines/99_bottles_of_beer_no_wait"
setup_config['config_path'] = "./"
setup_config['net_config_path'] = "./"


def info(title):
    print(title)
    print('module name:', __name__)
    if hasattr(os, 'getppid'):  # only available on Unix
        print('parent process:', os.getppid())
    print('process id:', os.getpid())


def check_for_sm_finished(sm, reactor):
    while sm.root_state.state_execution_status is not StateExecutionState.INACTIVE:
        try:
            sm.root_state.concurrency_queue.get(timeout=1.0)
        except Empty, e:
            pass
        # no logger output here to make it easier for the parser
        print "RAFCON live signal"

    sm.root_state.join()
    reactor.stop()


def start_state_machine(reactor, network_connections, global_net_config, sm_execution_engine):
    time.sleep(1.0)
    # Note: The rafcon_server has to be started before the statemachine is launched
    if not global_net_config.get_config_value("SPACEBOT_CUP_MODE"):
        network_connections.connect_tcp()

    glib.idle_add(network_connections.register_udp)

    sm = sm_execution_engine.execute_state_machine_from_path(setup_config['sm_path'],
                                                             wait_for_execution_finished=False)
    import threading
    sm_thread = threading.Thread(target=check_for_sm_finished, args=[sm, reactor])
    sm_thread.start()


def start_rafcon(name, q):
    info(name)
    #########################################################################
    # import section for process
    #########################################################################
    from twisted.internet import gtk2reactor
    gtk2reactor.install()
    from twisted.internet import reactor
    # singletons
    import rafcon.mvc.singleton as mvc_singletons
    import rafcon.statemachine.singleton
    from rafcon.network.singleton import network_connections

    # rafcon.statemachine
    from rafcon.statemachine.execution.statemachine_execution_engine import StatemachineExecutionEngine
    from rafcon.statemachine.states.execution_state import ExecutionState
    from rafcon.statemachine.states.hierarchy_state import HierarchyState
    from rafcon.statemachine.storage.storage import StateMachineStorage
    from rafcon.statemachine.config import global_config

    # rafcon.utils
    from rafcon.utils import log
    from rafcon.utils.config import config_path

    # rafcon.network
    from rafcon.network.network_config import global_net_config

    #########################################################################
    # code section for process
    #########################################################################
    logger = log.get_logger("start-no-gui")
    logger.info("initialize RAFCON ... ")

    # rafcon.statemachine.singleton.state_machine_manager.delete_all_state_machines()
    # variables_for_pytest.test_multithrading_lock.acquire()
    #
    # sm = StatemachineExecutionEngine.execute_state_machine_from_path(
    #     "../../test_scripts/test_libraries/99_bottles_of_beer_no_wait")
    # # rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(sm.state_machine_id)
    #
    # variables_for_pytest.test_multithrading_lock.release()

    # signal.signal(signal.SIGINT, rafcon.statemachine.singleton.signal_handler)

    global_config.load(path=setup_config['config_path'])
    global_net_config.load(path=setup_config['net_config_path'])

    # the network connections cannot be initialized before the network configuration was loaded
    network_connections.initialize()

    # Initialize libraries
    rafcon.statemachine.singleton.library_manager.initialize()

    # Set base path of global storage
    rafcon.statemachine.singleton.global_storage.base_path = "/tmp"

    start_state_machine(reactor, network_connections, global_net_config, rafcon.statemachine.singleton.state_machine_execution_engine)

    # setup network connections
    reactor.run()

    rafcon.statemachine.singleton.state_machine_execution_engine.stop()
    logger.info("State machine execution finished!")

    print "process 1 is going to exit"
    exit(0)


def check_for_stop_signal(q, reactor):
    data = q.get()
    if data == "stop":
        print "stop signal received ... going to stop the reactor"
        reactor.stop()


def start_rafcon_server(name, q, execution_signal_queue):
    info(name)
    #########################################################################
    # import section for process
    #########################################################################
    from twisted.internet import gtk2reactor
    gtk2reactor.install()
    from twisted.internet import reactor
    # singletons
    import rafcon.mvc.singleton as mvc_singletons
    import rafcon.statemachine.singleton
    from rafcon.network.singleton import network_connections
    # rafcon server imports
    from rafcon_server.mvc.views.debug_view import DebugView
    from rafcon_server.mvc.controller.debug_view import DebugViewController
    from rafcon_server.mvc.controller.connection_manager import ConnectionManager
    from rafcon_server.mvc.models.connection_manager import ConnectionManagerModel

    # rafcon.statemachine
    from rafcon.statemachine.execution.statemachine_execution_engine import StatemachineExecutionEngine
    from rafcon.statemachine.states.execution_state import ExecutionState
    from rafcon.statemachine.states.hierarchy_state import HierarchyState
    from rafcon.statemachine.storage.storage import StateMachineStorage
    from rafcon.statemachine.config import global_config

    # rafcon.utils
    from rafcon.utils import log
    from rafcon.utils.config import config_path

    # rafcon.network
    from rafcon.network.network_config import global_net_config

    #########################################################################
    # code section for process
    #########################################################################

    logger = log.get_logger("rafcon_server")
    logger.info("initialize RAFCON ... ")

    global_config.load(path=setup_config['config_path'])
    global_net_config.load(path=setup_config['net_config_path'])

    # initialize the logging view
    os.chdir("../../rafcon_server/mvc/")
    debug_view = DebugView()

    rafcon.statemachine.singleton.library_manager.initialize()

    [state_machine, version, creation_time] = rafcon.statemachine.singleton.global_storage.\
        load_statemachine_from_path(setup_config['sm_path'])
    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)

    logger.debug("The following statemachine was loaded: {0}".
                 format(rafcon.statemachine.singleton.state_machine_manager.get_active_state_machine().file_system_path))

    connection_manager = ConnectionManager()
    connection_manager_model = ConnectionManagerModel(connection_manager)

    debug_view_ctrl = DebugViewController(connection_manager_model, debug_view, multiprocessing_queue=q)

    execution_signal_queue.put("ready")
    import threading
    stop_signal_checker = threading.Thread(target=check_for_stop_signal, args=[execution_signal_queue, reactor])
    stop_signal_checker.start()
    reactor.run()
    print "process 2 is going to exit"
    exit(0)


def test_sm_and_server():

    unit_test_message_queue = Queue()
    execution_signal_queue = Queue()

    rafcon_server_process = Process(target=start_rafcon_server, args=("rafcon_server", unit_test_message_queue, execution_signal_queue))
    rafcon_server_process.start()

    data = execution_signal_queue.get()
    assert data == "ready"

    rafcon_process = Process(target=start_rafcon, args=("rafcon", unit_test_message_queue))
    rafcon_process.start()

    test_sequence = [
        "99 Bottles of Beer: registered",
        "99 Bottles of Beer: GLSUJY/PXTKIH",
        "99 Bottles of Beer: ------------------------------------",
        "99 Bottles of Beer: GLSUJY/PXTKIH",
        "99 Bottles of Beer: ------------------------------------",
        "99 Bottles of Beer: GLSUJY/NDIVLD",
        "99 Bottles of Beer: ------------------------------------",
        "99 Bottles of Beer: GLSUJY/SFZGMH",
        "99 Bottles of Beer: ------------------------------------",
        "99 Bottles of Beer: GLSUJY/PXTKIH",
        "99 Bottles of Beer: ------------------------------------",
        "99 Bottles of Beer: GLSUJY/NDIVLD",
        "99 Bottles of Beer: ------------------------------------",
        "99 Bottles of Beer: GLSUJY/SFZGMH",
        "99 Bottles of Beer: ------------------------------------",
        "99 Bottles of Beer: GLSUJY/PXTKIH",
        "99 Bottles of Beer: ------------------------------------",
        "99 Bottles of Beer: GLSUJY/NDIVLD",
        "99 Bottles of Beer: ------------------------------------",
        "99 Bottles of Beer: GLSUJY/SFZGMH",
        "99 Bottles of Beer: ------------------------------------",
        "99 Bottles of Beer: STATE_MACHINE_EXECUTION_STATUS.STOPPED"
    ]

    for i in range(len(test_sequence)):
        data = unit_test_message_queue.get()
        # print test_sequence[i]
        # print data
        assert data == test_sequence[i]

    # data = q.get()
    # assert data == "something"

    rafcon_process.join()
    execution_signal_queue.put("stop")
    rafcon_server_process.join()

if __name__ == '__main__':
    test_sm_and_server()
