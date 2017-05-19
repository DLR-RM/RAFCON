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

import sys
sys.path.insert(1, '/volume/software/common/packages/python_acknowledged_udp/latest/lib/python2.7')

# communication queues
CLIENT1_TO_SERVER_QUEUE = "client1_to_server"
SERVER_TO_CLIENT1_QUEUE = "server_to_client1"
MAIN_QUEUE = "main_queue"
KILL_SERVER_QUEUE = "kill_server"
KILL_CLIENT1_QUEUE = "kill_client1"

# messages
STEPPING_SUCCESSFUL = "Stepping successful"
STOP_START_SUCCESSFUL = "Stop start successful"
RUN_UNTIL_SUCCESSFUL = "Run until successful"
START_FROM_SUCCESSFUL = "Start from successful"

# test steps
TestSteps = {0: "STEP_MODE",
             1: "STEP_OVER",
             2: "STEP_INTO",
             3: "BACKWARD_STEP",
             4: "STEP_OUT",
             5: "START_AND_WAIT_UNTIL_FINISHED",
             6: "RUN_UNTIL",
             7: "STOP",
             8: "START_FROM"
             }

# Sing: PXTKIH
# Decimate Bottles: NDIVLD
# Count bottles: SFZGMH


def custom_assert(statement1, statement2):
    try:
        assert statement1 == statement2
    except Exception as e:
        print statement1 + " is not equal " + statement2
        exit()


def print_highlight(statement):
    print "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
    print statement
    print "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"


def reset_global_variable_manager(global_variable_manager):
    global_variable_manager.set_variable("sing_counter", 0)
    global_variable_manager.set_variable("decimate_counter", 0)
    global_variable_manager.set_variable("count_counter", 0)

sleep_time = 0.01


def synchronize_with_root_state(root_state, execution_engine, state_id_to_join_first):
    global sleep_time
    root_state.states[state_id_to_join_first].join()
    # wait until the root state is is ready for the next execution command i.e.
    # waiting at the self._status.execution_condition_variable
    while not execution_engine.status.execution_condition_variable.get_number_of_waiting_threads() > 0:
        time.sleep(sleep_time)


def synchronize_with_clients_threads(queue_dict, execution_engine):
    from rafcon.core.singleton import global_variable_manager as gvm
    from rafcon.core.execution.execution_status import StateMachineExecutionStatus
    from rafcon.core.singleton import state_machine_manager
    from rafcon.core.execution.execution_engine import ExecutionEngine
    assert isinstance(execution_engine, ExecutionEngine)
    active_sm = state_machine_manager.get_active_state_machine()
    root_state = active_sm.root_state
    sleep_time = 0.01

    # check when run to is finished
    while gvm.get_variable("sing_counter") < 1:
        time.sleep(sleep_time)
    synchronize_with_root_state(root_state, execution_engine, "PXTKIH")

    # wait for the client to start
    queue_dict[CLIENT1_TO_SERVER_QUEUE].get()

    queue_dict[SERVER_TO_CLIENT1_QUEUE].put("start stepping")
    print "starting tests\n\n"

    print "server: cp0"
    # step test
    while gvm.get_variable("decimate_counter") < 1:
        time.sleep(sleep_time)  # sleep just to prevent busy loop
    synchronize_with_root_state(root_state, execution_engine, "NDIVLD")
    queue_dict[SERVER_TO_CLIENT1_QUEUE].put(TestSteps[0])  # step mode also means a step into

    print "server: cp1"
    while gvm.get_variable("count_counter") < 1:
        time.sleep(sleep_time)
    synchronize_with_root_state(root_state, execution_engine, "SFZGMH")
    queue_dict[SERVER_TO_CLIENT1_QUEUE].put(TestSteps[1])  # step over

    print "server: cp2"
    while gvm.get_variable("sing_counter") < 2:
        time.sleep(sleep_time)
    synchronize_with_root_state(root_state, execution_engine, "PXTKIH")
    queue_dict[SERVER_TO_CLIENT1_QUEUE].put(TestSteps[2])  # step into

    print "server: cp3"
    while gvm.get_variable("sing_counter") > 1:
        time.sleep(sleep_time)
    # wait until the backward execution of the state is done
    synchronize_with_root_state(root_state, execution_engine, "PXTKIH")
    queue_dict[SERVER_TO_CLIENT1_QUEUE].put(TestSteps[3])  # backward step

    print "server: cp4"
    while not execution_engine.finished_or_stopped():
        time.sleep(sleep_time)
    execution_engine.join()
    queue_dict[SERVER_TO_CLIENT1_QUEUE].put(TestSteps[4])  # step out and run until the end
    reset_global_variable_manager(gvm)
    print "server: step test successful\n\n"

    # start and finish execution test
    while execution_engine.status.execution_mode is not StateMachineExecutionStatus.STARTED:
        time.sleep(sleep_time)
    # wait until the state machine executed successfully
    while not execution_engine.finished_or_stopped():
        time.sleep(sleep_time)
    reset_global_variable_manager(gvm)
    # as the state machine run to the end this is safe
    execution_engine.stop()  # reset state machine before the next test
    # do not notify the client before the state machine is really stopped
    execution_engine.join()
    print "server: start and wait until finished test successful\n\n"
    # old_sync_counter = execution_engine.synchronization_counter
    queue_dict[SERVER_TO_CLIENT1_QUEUE].put(TestSteps[5])  # run until end

    # run-until test
    # this is dangerous: the child state is still being executed when setting the "decimate_counter"
    # global variable and thus the root_state=Hierarchy state is not YET handling the execution mode
    # while gvm.get_variable("decimate_counter") < 1 and not root_state.handling_execution_mode:
    #     time.sleep(sleep_time)
    # this also can produce race conditions
    # while not(execution_engine.synchronization_counter == old_sync_counter + 3):
    #     time.sleep(sleep_time)
    # this is safe:
    while gvm.get_variable("decimate_counter") < 1:
        time.sleep(sleep_time)
    synchronize_with_root_state(root_state, execution_engine, "NDIVLD")
    assert gvm.get_variable("count_counter") == 0
    queue_dict[SERVER_TO_CLIENT1_QUEUE].put(TestSteps[6])

    while not execution_engine.finished_or_stopped():
        time.sleep(sleep_time)
    execution_engine.join()
    reset_global_variable_manager(gvm)
    print "server: run until test successful\n\n"
    queue_dict[SERVER_TO_CLIENT1_QUEUE].put(TestSteps[7])

    # start from test
    while execution_engine.status.execution_mode is not StateMachineExecutionStatus.STARTED:
        time.sleep(sleep_time)
    while not execution_engine.finished_or_stopped():
        time.sleep(sleep_time)
    assert gvm.get_variable("sing_counter") == 2
    assert gvm.get_variable("decimate_counter") == 3
    # as the state machine run to the end this is safe
    execution_engine.stop()
    execution_engine.join()
    print "server: start from test successful\n"
    queue_dict[SERVER_TO_CLIENT1_QUEUE].put(TestSteps[8])

    execution_engine.stop()
    execution_engine.join()
    print "server: wait for sync message from client\n"
    queue_dict[CLIENT1_TO_SERVER_QUEUE].get()
    print "server: send sync message to client\n"
    queue_dict[SERVER_TO_CLIENT1_QUEUE].put("sync")
    print "server: wait for kill command from main queue\n"
    # set a timeout of 3 seconds
    queue_dict[KILL_SERVER_QUEUE].get(3)
    os._exit(0)
    # normal exit does not work
    # exit(0)


def interacting_function_server(queue_dict):
    from rafcon.utils import log
    logger = log.get_logger("Interacting server")

    for id, queue in queue_dict.iteritems():
        assert isinstance(queue, multiprocessing.queues.Queue)

    import rafcon.core.singleton as core_singletons
    execution_engine = core_singletons.state_machine_execution_engine

    sm_thread = threading.Thread(target=synchronize_with_clients_threads, args=[queue_dict, execution_engine])
    sm_thread.start()

    active_sm = core_singletons.state_machine_manager.get_active_state_machine()
    # root state is a hierarchy state
    for key, sv in active_sm.root_state.scoped_variables.iteritems():
        if sv.name == "bottles":
            sv.default_value = 3
    # execution_engine.run_to_selected_state("GLSUJY/PXTKIH") # wait before Sing
    execution_engine.run_to_selected_state("GLSUJY/NDIVLD")  # wait before Decimate Bottles
    # execution_engine.run_to_selected_state("GLSUJY/SFZGMH")  # wait before Count Bottles
    sm_thread.join()
    print_highlight("Joined server worker thread")


def interacting_function_client1(main_window_controller, global_monitoring_manager, queue_dict):
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
    queue_dict[SERVER_TO_CLIENT1_QUEUE].get()  # synchronize, when to start stepping

    # test stepping
    # print "client: cp0"
    remote_execution_engine.step_mode()  # this will trigger a step into = triggers decimate
    custom_assert(queue_dict[SERVER_TO_CLIENT1_QUEUE].get(), TestSteps[0])

    # print "client: cp1"
    remote_execution_engine.step_over()  # triggers count
    custom_assert(queue_dict[SERVER_TO_CLIENT1_QUEUE].get(), TestSteps[1])

    # print "client: cp2"
    remote_execution_engine.step_into()  # triggers sing
    custom_assert(queue_dict[SERVER_TO_CLIENT1_QUEUE].get(), TestSteps[2])

    # print "client: cp3"
    remote_execution_engine.backward_step()  # backward triggers sing
    custom_assert(queue_dict[SERVER_TO_CLIENT1_QUEUE].get(), TestSteps[3])
    queue_dict[MAIN_QUEUE].put(STEPPING_SUCCESSFUL)

    # print "client: cp4"
    remote_execution_engine.step_out()  # triggers run until end
    custom_assert(queue_dict[SERVER_TO_CLIENT1_QUEUE].get(), TestSteps[4])

    # start execution test
    remote_execution_engine.start()
    custom_assert(queue_dict[SERVER_TO_CLIENT1_QUEUE].get(), TestSteps[5])
    queue_dict[MAIN_QUEUE].put(STOP_START_SUCCESSFUL)

    # run-until test
    remote_execution_engine.run_to_selected_state("GLSUJY/SFZGMH")  # run to decimate bottles inclusively
    custom_assert(queue_dict[SERVER_TO_CLIENT1_QUEUE].get(), TestSteps[6])
    remote_execution_engine.stop()
    custom_assert(queue_dict[SERVER_TO_CLIENT1_QUEUE].get(), TestSteps[7])
    queue_dict[MAIN_QUEUE].put(RUN_UNTIL_SUCCESSFUL)

    # start from test
    # directly start with decimate bottles and jump over the sing state
    remote_execution_engine.start(start_state_path="GLSUJY/NDIVLD")
    custom_assert(queue_dict[SERVER_TO_CLIENT1_QUEUE].get(), TestSteps[8])

    print "client: send sync message to server\n"
    queue_dict[CLIENT1_TO_SERVER_QUEUE].put("sync")
    print "client: wait for sync message from server\n"
    queue_dict[SERVER_TO_CLIENT1_QUEUE].get()
    print "client: tell the main queue that tests were successful\n"
    queue_dict[MAIN_QUEUE].put(START_FROM_SUCCESSFUL)
    print "client: wait for kill command from main queue\n"
    # set a timeout of 3 seconds
    queue_dict[KILL_CLIENT1_QUEUE].get(3)
    os._exit(0)
    # normal exit does not work
    # exit(0)


def launch_client(interacting_function_client, multiprocessing_queue_dict):
    # explicitly add the monitoring plugin to the RAFCON_PLUGIN_PATH
    if os.environ.get('RAFCON_PLUGIN_PATH', False):
        del os.environ['RAFCON_PLUGIN_PATH']
    os.environ['RAFCON_PLUGIN_PATH'] = \
        "/volume/software/common/packages/rafcon_monitoring_plugin/latest/lib/python2.7/monitoring"

    import network.start_client
    network.start_client.start_client(interacting_function_client, multiprocessing_queue_dict)

    # import sys
    # sys.path.insert(1, '/volume/software/common/packages/python_acknowledged_udp/latest/lib/python2.7')


def launch_server(interacting_function_handle_server_, multiprocessing_queue_dict):
    # explicitly add the monitoring plugin to the RAFCON_PLUGIN_PATH
    if os.environ.get('RAFCON_PLUGIN_PATH', False):
        del os.environ['RAFCON_PLUGIN_PATH']
    os.environ['RAFCON_PLUGIN_PATH'] = \
        "/volume/software/common/packages/rafcon_monitoring_plugin/latest/lib/python2.7/monitoring"

    import network.start_server
    server = Process(target=network.start_server.start_server,
                     args=(interacting_function_handle_server_, multiprocessing_queue_dict))

    return server


def check_if_ports_are_open():
    import psutil
    from acknowledged_udp.config import global_network_config
    config_path = os.path.abspath(path=os.path.dirname(os.path.abspath(__file__)) + "/server")
    global_network_config.load(path=config_path)
    for connection in psutil.net_connections():
        if int(global_network_config.get_config_value("SERVER_UDP_PORT")) == connection.laddr[1]:
            return False
    return True


def test_single_client():

    if not check_if_ports_are_open():
        print "Address already in use by another server!"
        assert True == False

    test_successful = True

    queue_dict = dict()
    queue_dict[CLIENT1_TO_SERVER_QUEUE] = Queue()
    queue_dict[SERVER_TO_CLIENT1_QUEUE] = Queue()
    queue_dict[MAIN_QUEUE] = Queue()
    queue_dict[KILL_SERVER_QUEUE] = Queue()
    queue_dict[KILL_CLIENT1_QUEUE] = Queue()

    server = launch_server(interacting_function_server, queue_dict)
    server.start()

    client1 = Process(target=launch_client, args=(interacting_function_client1, queue_dict))
    client1.start()

    try:
        data = queue_dict[MAIN_QUEUE].get(timeout=10)
        # print "===========================================", data
        assert data == STEPPING_SUCCESSFUL

        data = queue_dict[MAIN_QUEUE].get(timeout=10)
        # print "===========================================", data
        assert data == STOP_START_SUCCESSFUL

        data = queue_dict[MAIN_QUEUE].get(timeout=10)
        # print "===========================================", data
        assert data == RUN_UNTIL_SUCCESSFUL

        data = queue_dict[MAIN_QUEUE].get(timeout=10)
        # print "===========================================", data

    except:
        server.terminate()
        client1.terminate()
        server.join(timeout=10)
        client1.join(timeout=10)
        raise
    try:
        assert data == START_FROM_SUCCESSFUL
        print "Test successful"
    except AssertionError, e:
        print "Test not successful"
        test_successful = False

    queue_dict[KILL_SERVER_QUEUE].put("Kill", timeout=10)
    queue_dict[KILL_CLIENT1_QUEUE].put("Kill", timeout=10)

    print "Joining processes"

    # wait for each process a maximum of 10 seconds
    client1.join(timeout=10)
    server.join(timeout=10)

    assert not client1.is_alive(), "Client1 is still alive"
    assert not server.is_alive(), "Server is still alive"
    assert test_successful is True


if __name__ == '__main__':
    test_single_client()
    # pytest.main([__file__])

