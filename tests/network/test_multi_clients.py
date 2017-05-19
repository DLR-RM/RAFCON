"""
.. module:: test multi clients
   :platform: Unix, Windows
   :synopsis: A module to test the RAFCON monitoring setup with multiple clients

.. moduleauthor:: Sebastian Brunner


"""

from multiprocessing import Process, Queue

import multiprocessing
import threading
import time
import os
import pytest

# communication queues
CLIENT1_TO_SERVER_QUEUE = "client1_to_server"
SERVER_TO_CLIENT1_QUEUE = "server_to_client1"
CLIENT2_TO_SERVER_QUEUE = "client2_to_server"
SERVER_TO_CLIENT2_QUEUE = "server_to_client2"
CLIENT1_TO_CLIENT2 = "client1_to_client2"
CLIENT2_TO_CLIENT1 = "client2_to_client1"
MAIN_QUEUE = "main_queue"
KILL_SERVER_QUEUE = "kill_server"
KILL_CLIENT1_QUEUE = "kill_client1"
KILL_CLIENT2_QUEUE = "kill_client2"

# messages
START_PAUSE_RESUME_SUCCESSFUL = "Start pause resume"
TEST_ERROR = "Test error"

# Sing: PXTKIH
# Decimate Bottles: NDIVLD
# Count bottles: SFZGMH


sleep_time = 0.01


def synchronize_with_root_state(root_state, execution_engine, state_id_to_join_first):
    global sleep_time
    root_state.states[state_id_to_join_first].join()
    # wait until the root state is is ready for the next execution command i.e.
    # waiting at the self._status.execution_condition_variable
    while not execution_engine.status.execution_condition_variable.get_number_of_waiting_threads() > 0:
        time.sleep(sleep_time)


def synchronize_with_client_threads(queue_dict, execution_engine):
    from rafcon.core.singleton import global_variable_manager as gvm
    from rafcon.core.execution.execution_status import StateMachineExecutionStatus
    from rafcon.core.singleton import state_machine_manager
    from rafcon.core.execution.execution_engine import ExecutionEngine
    assert isinstance(execution_engine, ExecutionEngine)
    active_sm = state_machine_manager.get_active_state_machine()
    root_state = active_sm.root_state
    sleep_time = 0.01

    # wait for the client to start
    queue_dict[CLIENT1_TO_SERVER_QUEUE].get()
    queue_dict[SERVER_TO_CLIENT1_QUEUE].put("ready")

    # start pause resume test
    # assuming that inter process communication is faster than networking the state machine should be started,
    # paused and resumed in the meantime

    # wait until execution engine is started
    while execution_engine.status.execution_mode is not StateMachineExecutionStatus.STARTED:
        time.sleep(sleep_time)
    # let the state machine run for a while
    time.sleep(0.5)
    # synchronize with client client2
    queue_dict[CLIENT2_TO_SERVER_QUEUE].get()
    queue_dict[SERVER_TO_CLIENT2_QUEUE].put("ready")
    # wait until execution is finished
    while not execution_engine.finished_or_stopped():
        time.sleep(sleep_time)

    queue_dict[SERVER_TO_CLIENT1_QUEUE].put("state machine executed successfully")  # synchronize with client1
    queue_dict[SERVER_TO_CLIENT2_QUEUE].put("state machine executed successfully")  # synchronize with client2
    print "server: start pause resume test successful\n\n"

    execution_engine.stop()
    execution_engine.join()
    queue_dict[KILL_SERVER_QUEUE].get(3)
    os._exit(0)


def interacting_function_server(queue_dict):
    # from rafcon.utils import log
    # logger = log.get_logger("Interacting server")

    for id, queue in queue_dict.iteritems():
        assert isinstance(queue, multiprocessing.queues.Queue)

    import rafcon.core.singleton as core_singletons
    execution_engine = core_singletons.state_machine_execution_engine

    active_sm = core_singletons.state_machine_manager.get_active_state_machine()
    # root state is a hierarchy state
    for key, sv in active_sm.root_state.scoped_variables.iteritems():
        if sv.name == "bottles":
            sv.default_value = 8

    sm_thread = threading.Thread(target=synchronize_with_client_threads, args=[queue_dict, execution_engine])
    sm_thread.start()


def interacting_function_client1(main_window_controller, global_monitoring_manager, queue_dict):
    import Queue
    from rafcon.utils import log
    logger = log.get_logger("Interacting client1")

    logger.info("Start interacting with server\n\n")

    for id, queue in queue_dict.iteritems():
        assert isinstance(queue, multiprocessing.queues.Queue)

    while not global_monitoring_manager.endpoint_initialized:
        logger.warn("global_monitoring_manager not initialized yet!")
        time.sleep(0.01)

    import rafcon.core.singleton as core_singletons
    remote_execution_engine = core_singletons.state_machine_execution_engine

    # tell the server that client 1 is ready
    queue_dict[CLIENT1_TO_SERVER_QUEUE].put("ready")
    queue_dict[SERVER_TO_CLIENT1_QUEUE].get()

    # test start (from server) + pause (from client1) + resume (from client2)
    remote_execution_engine.start()
    try:
        # Wait for client2 to pause the state machine
        queue_dict[CLIENT2_TO_CLIENT1].get(timeout=10)  # get paused signal from client2
    except Queue.Empty:
        queue_dict[MAIN_QUEUE].put(TEST_ERROR)
        logger.exception("Client2 did not respond")
        os._exit(0)

    remote_execution_engine.start()  # resume execution

    queue_dict[SERVER_TO_CLIENT1_QUEUE].get()  # synchronize to server
    queue_dict[MAIN_QUEUE].put(START_PAUSE_RESUME_SUCCESSFUL)
    queue_dict[KILL_CLIENT1_QUEUE].get(3)  # synchronize to main process
    os._exit(0)


def interacting_function_client2(main_window_controller, global_monitoring_manager, queue_dict):
    import rafcon.core.singleton as core_singletons
    from rafcon.utils import log
    logger = log.get_logger("Interacting client2")

    for id, queue in queue_dict.iteritems():
        assert isinstance(queue, multiprocessing.queues.Queue)

    while not global_monitoring_manager.endpoint_initialized:
        time.sleep(0.01)

    remote_execution_engine = core_singletons.state_machine_execution_engine

    # synchronize with server
    queue_dict[CLIENT2_TO_SERVER_QUEUE].put("ready")
    queue_dict[SERVER_TO_CLIENT2_QUEUE].get()

    logger.info("Pausing state machine on remote server")
    remote_execution_engine.pause()
    time.sleep(0.5)  # let the state machine pause for a while
    queue_dict[CLIENT2_TO_CLIENT1].put("State machine paused for some seconds!")

    queue_dict[SERVER_TO_CLIENT2_QUEUE].get(2)  # synchronize to server
    queue_dict[KILL_CLIENT2_QUEUE].get(3)  # synchronize to main process
    os._exit(0)


def test_multi_clients():
    from network.test_single_client import launch_client
    from network.test_single_client import launch_server
    from test_single_client import check_if_ports_are_open
    if not check_if_ports_are_open():
        print "Address already in use by another server!"
        assert True == False

    test_successful = True

    queue_dict = dict()
    queue_dict[CLIENT1_TO_SERVER_QUEUE] = Queue()
    queue_dict[SERVER_TO_CLIENT1_QUEUE] = Queue()
    queue_dict[CLIENT2_TO_SERVER_QUEUE] = Queue()
    queue_dict[SERVER_TO_CLIENT2_QUEUE] = Queue()
    queue_dict[CLIENT1_TO_CLIENT2] = Queue()
    queue_dict[CLIENT2_TO_CLIENT1] = Queue()
    queue_dict[MAIN_QUEUE] = Queue()
    queue_dict[KILL_SERVER_QUEUE] = Queue()
    queue_dict[KILL_CLIENT1_QUEUE] = Queue()
    queue_dict[KILL_CLIENT2_QUEUE] = Queue()

    server = launch_server(interacting_function_server, queue_dict)
    server.start()

    client1 = Process(target=launch_client, args=(interacting_function_client1, queue_dict))
    client1.start()

    client2 = Process(target=launch_client, args=(interacting_function_client2, queue_dict))
    client2.start()

    try:
        data = queue_dict[MAIN_QUEUE].get(timeout=20)
        # print "===========================================", data
    except:
        server.terminate()
        client1.terminate()
        client2.terminate()
        server.join(timeout=10)
        client1.join(timeout=10)
        client2.join(timeout=10)
        raise
    try:
        assert data == START_PAUSE_RESUME_SUCCESSFUL
        print "Test successful"
    except AssertionError, e:
        print "Test not successful"
        test_successful = False

    queue_dict[KILL_SERVER_QUEUE].put("Kill", timeout=10)
    queue_dict[KILL_CLIENT1_QUEUE].put("Kill", timeout=10)
    queue_dict[KILL_CLIENT2_QUEUE].put("Kill", timeout=10)

    print "Joining processes"
    server.join(timeout=10)
    client1.join(timeout=10)
    client2.join(timeout=10)

    assert not server.is_alive(), "Server is still alive"
    assert not client1.is_alive(), "Client1 is still alive"
    assert not client2.is_alive(), "Client2 is still alive"
    assert test_successful is True


if __name__ == '__main__':
    test_multi_clients()
    # pytest.main([__file__])

