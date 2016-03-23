import testing_utils

# system libs
from multiprocessing import Process, Queue
import os
import glib
import time
import pytest
from Queue import Empty


def info(title):
    print(title)
    print('module name:', __name__)
    if hasattr(os, 'getppid'):  # only available on Unix
        print('parent process:', os.getppid())
    print('process id:', os.getpid())


def check_for_sm_finished(sm, reactor):
    from rafcon.statemachine.enums import StateExecutionState
    from rafcon.statemachine.singleton import state_machine_execution_engine
    while sm.root_state.state_execution_status is not StateExecutionState.INACTIVE:
        try:
            sm.root_state.concurrency_queue.get(timeout=1.0)
        except Empty, e:
            pass
        # no logger output here to make it easier for the parser
        print "RAFCON live signal"

    state_machine_execution_engine.join()
    reactor.stop()


def start_state_machine(reactor, network_connections, global_net_config, sm_execution_engine, setup_config=None):
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
    from rafcon.statemachine.config import global_config

    # rafcon.utils
    from rafcon.utils import log
    from rafcon.utils.config import config_path

    # rafcon.network
    from rafcon.network.network_config import global_net_config

    import rafcon
    rafcon_path, rafcon_file = os.path.split(rafcon.__file__)
    setup_config = dict()
    setup_config['sm_path'] = rafcon_path + "/../test_scripts/unit_test_state_machines/99_bottles_of_beer_no_wait"
    setup_config['config_path'] = rafcon_path + "/../test/network"
    setup_config['net_config_path'] = rafcon_path + "/../test/network"

    #########################################################################
    # code section for process
    #########################################################################
    logger = log.get_logger("start-no-gui")
    logger.info("initialize RAFCON ... ")

    global_config.load(path=setup_config['config_path'])
    global_net_config.load(path=setup_config['net_config_path'])

    # the network connections cannot be initialized before the network configuration was loaded
    network_connections.initialize()

    # Initialize libraries
    rafcon.statemachine.singleton.library_manager.initialize()

    start_state_machine(reactor,
                        network_connections,
                        global_net_config,
                        rafcon.statemachine.singleton.state_machine_execution_engine,
                        setup_config=setup_config)

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
    from rafcon.statemachine.storage import storage
    # rafcon server imports
    from rafcon_server.mvc.views.debug_view import DebugView
    from rafcon_server.mvc.controller.debug_view import DebugViewController
    from rafcon_server.mvc.controller.connection_manager import ConnectionManager
    from rafcon_server.mvc.models.connection_manager import ConnectionManagerModel

    # rafcon.statemachine
    from rafcon.statemachine.execution.statemachine_execution_engine import StatemachineExecutionEngine
    from rafcon.statemachine.states.execution_state import ExecutionState
    from rafcon.statemachine.states.hierarchy_state import HierarchyState
    from rafcon.statemachine.config import global_config

    # rafcon.utils
    from rafcon.utils import log
    from rafcon.utils.config import config_path

    # rafcon.network
    from rafcon.network.network_config import global_net_config

    import rafcon
    rafcon_path, rafcon_file = os.path.split(rafcon.__file__)
    setup_config = dict()
    setup_config['sm_path'] = rafcon_path + "/../test_scripts/unit_test_state_machines/99_bottles_of_beer_no_wait"
    setup_config['config_path'] = rafcon_path + "/../test/network"
    setup_config['net_config_path'] = rafcon_path + "/../test/network"

    #########################################################################
    # code section for process
    #########################################################################

    logger = log.get_logger("rafcon_server")
    logger.info("initialize RAFCON ... ")

    global_config.load(path=setup_config['config_path'])
    global_net_config.load(path=setup_config['net_config_path'])

    # initialize the logging view
    import rafcon_server
    rs_path, rs_file = os.path.split(rafcon_server.__file__)
    os.chdir(rs_path + "/mvc/")
    debug_view = DebugView()

    rafcon.statemachine.singleton.library_manager.initialize()

    state_machine = storage.load_statemachine_from_path(setup_config['sm_path'])
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


    # ths stop lines sometimes don't come in the correct order, or the start or end stop line is skipped
    # current stop line: "99 Bottles of Beer: ------------------------------------",
    test_sequence = [
        "99 Bottles of Beer: registered",
        "99 Bottles of Beer: GLSUJY/PXTKIH",
        "99 Bottles of Beer: GLSUJY/PXTKIH",
        "99 Bottles of Beer: GLSUJY/NDIVLD",
        "99 Bottles of Beer: GLSUJY/SFZGMH",
        "99 Bottles of Beer: GLSUJY/PXTKIH",
        "99 Bottles of Beer: GLSUJY/NDIVLD",
        "99 Bottles of Beer: GLSUJY/SFZGMH",
        "99 Bottles of Beer: GLSUJY/PXTKIH",
        "99 Bottles of Beer: GLSUJY/NDIVLD",
        "99 Bottles of Beer: GLSUJY/SFZGMH",
        "99 Bottles of Beer: STATE_MACHINE_EXECUTION_STATUS.STOPPED"
    ]

    offset = 0
    for i in range(len(test_sequence)):
        data = unit_test_message_queue.get(timeout=10.0)
        # print test_sequence[i]
        # print data
        while data == "99 Bottles of Beer: ------------------------------------":
            data = unit_test_message_queue.get()
        # it seems like normally the first state is sent twice; under cpu load the first state is only sent once; both
        # variants are accepted right now but
        # TODO: check this test again
        if i == 2:
            if data == "99 Bottles of Beer: GLSUJY/NDIVLD":
                offset += 1
        assert data == test_sequence[i + offset]

    rafcon_process.join()
    execution_signal_queue.put("stop")
    rafcon_server_process.join()


if __name__ == '__main__':
    # test_sm_and_server()
    pytest.main([__file__])
