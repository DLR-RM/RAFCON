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

from rafcon.utils import log
logger = log.get_logger(__name__)

FINAL_MESSAGE = "final_message"
CLIENT1_QUEUE = "client1"
CLIENT2_QUEUE = "client2"
CLIENT1_TO_CLIENT2 = "client1_to_client2"
CLIENT2_TO_CLIENT1 = "client2_to_client1"
MAIN_QUEUE = "main_queue"
KILL_SERVER_QUEUE = "kill_server"
KILL_CLIENT1_QUEUE = "kill_client1"
KILL_CLIENT2_QUEUE = "kill_client2"

STEPPING_SUCCESSFUL = "Stepping successful"
STOP_START_SUCCESSFUL = "Stop start successful"
RUN_UNTIL_SUCCESSFUL = "Run until successful"
START_FROM_SUCCESSFUL = "Start from successful"
START_PAUSE_RESUME_SUCCESSFUL = "Start pause resume"
TEST_ERROR = "Test error"


def reset_global_variable_manager(global_variable_manager):
    global_variable_manager.set_variable("sing_counter", 0)
    global_variable_manager.set_variable("decimate_counter", 0)
    global_variable_manager.set_variable("count_counter", 0)


def synchronize_with_clients_threads(queue_dict, execution_engine):
    from rafcon.core.singleton import global_variable_manager as gvm
    from rafcon.core.enums import StateMachineExecutionStatus
    from rafcon.core.singleton import state_machine_manager
    active_sm = state_machine_manager.get_active_state_machine()
    root_state = active_sm.root_state

    # check when run to is finished
    while gvm.get_variable("sing_counter") < 1:
        time.sleep(0.01)

    queue_dict[CLIENT1_QUEUE].put("start stepping")
    print "starting tests\n\n"

    print "server: cp0"
    # step test
    while gvm.get_variable("decimate_counter") < 1 and not root_state.handling_execution_mode:
        time.sleep(0.01)  # sleep just to prevent busy loop
    queue_dict[CLIENT1_QUEUE].put("step mode and thus a step into received")

    print "server: cp1"
    while gvm.get_variable("count_counter") < 1 and not root_state.handling_execution_mode:
        time.sleep(0.01)
    queue_dict[CLIENT1_QUEUE].put("step_received2")

    print "server: cp2"
    while gvm.get_variable("sing_counter") < 2 and not root_state.handling_execution_mode:
        time.sleep(0.01)
    queue_dict[CLIENT1_QUEUE].put("step_received3")

    print "server: cp3"
    while gvm.get_variable("decimate_counter") < 2 and not root_state.handling_execution_mode:
        time.sleep(0.01)
    queue_dict[CLIENT1_QUEUE].put("step_received3")

    print "server: cp4"
    while gvm.get_variable("decimate_counter") > 1 and not root_state.handling_execution_mode:
        time.sleep(0.01)
    queue_dict[CLIENT1_QUEUE].put("backward_step_received")
    reset_global_variable_manager(gvm)
    print "server: step test successful\n\n"

    # stop start test
    while execution_engine.status.execution_mode is not StateMachineExecutionStatus.STOPPED:
        time.sleep(0.01)

    queue_dict[CLIENT1_QUEUE].put("stop received")
    # wait until state machine started
    while execution_engine.status.execution_mode is not StateMachineExecutionStatus.STARTED:
        time.sleep(0.01)

    while execution_engine.status.execution_mode is not StateMachineExecutionStatus.STOPPED:
        time.sleep(0.01)

    reset_global_variable_manager(gvm)
    execution_engine.stop()  # reset state machine before the next test
    queue_dict[CLIENT1_QUEUE].put("start received and successfully ran the state machine")
    print "server: stop start test successful\n\n"

    # run-until test
    while gvm.get_variable("decimate_counter") < 1 and not root_state.handling_execution_mode:
        time.sleep(0.01)

    assert gvm.get_variable("count_counter") == 0

    execution_engine.stop()  # reset state machine before the next test
    reset_global_variable_manager(gvm)
    queue_dict[CLIENT1_QUEUE].put("run until received")
    print "server: run until test successful\n\n"

    # start from test
    while execution_engine.status.execution_mode is not StateMachineExecutionStatus.STARTED:
        time.sleep(0.01)

    while execution_engine.status.execution_mode is not StateMachineExecutionStatus.STOPPED:
        time.sleep(0.01)
    assert gvm.get_variable("sing_counter") == 2
    assert gvm.get_variable("decimate_counter") == 3

    execution_engine.stop()  # reset
    queue_dict[CLIENT1_QUEUE].put("start from test successful")
    print "server: start from test successful\n\n"

    # start pause resume test
    # assuming that inter process communication is faster than networking the state machine should be started,
    # paused and resumed in the meantime

    # wait until execution engine is started
    while execution_engine.status.execution_mode is not StateMachineExecutionStatus.STARTED:
        time.sleep(0.01)
    # wait until execution is finished
    while execution_engine.status.execution_mode is not StateMachineExecutionStatus.STOPPED:
        time.sleep(0.01)

    queue_dict[CLIENT1_QUEUE].put("state machine executed successfully")  # synchronize with client1
    queue_dict[CLIENT2_QUEUE].put("state machine executed successfully")  # synchronize with client2
    print "server: start pause resume test successful\n\n"

    execution_engine.stop()
    queue_dict[KILL_SERVER_QUEUE].get()
    os._exit(0)


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

    queue_dict[CLIENT1_QUEUE].get()  # synchronize, when to start stepping

    # test stepping
    print "client: cp0"
    remote_execution_engine.step_mode()  # this will trigger a step into = triggers decimate
    queue_dict[CLIENT1_QUEUE].get()

    print "client: cp1"
    remote_execution_engine.step_over()  # triggers count
    queue_dict[CLIENT1_QUEUE].get()

    print "client: cp2"
    remote_execution_engine.step_into()  # triggers sing
    queue_dict[CLIENT1_QUEUE].get()

    print "client: cp3"
    remote_execution_engine.step_out()  # triggers decimate
    queue_dict[CLIENT1_QUEUE].get()

    print "client: cp4"
    remote_execution_engine.backward_step()  # backward triggers decimate
    queue_dict[CLIENT1_QUEUE].get()
    queue_dict[MAIN_QUEUE].put(STEPPING_SUCCESSFUL)

    # test stop start
    remote_execution_engine.stop()
    queue_dict[CLIENT1_QUEUE].get()
    remote_execution_engine.start()
    queue_dict[CLIENT1_QUEUE].get()
    queue_dict[MAIN_QUEUE].put(STOP_START_SUCCESSFUL)

    # test stop run_until
    remote_execution_engine.run_to_selected_state("GLSUJY/SFZGMH")  # run to decimate bottles inclusively
    queue_dict[CLIENT1_QUEUE].get()
    queue_dict[MAIN_QUEUE].put(RUN_UNTIL_SUCCESSFUL)

    # test start from
    # directly start with decimate bottles and jump over the sing state
    remote_execution_engine.start(start_state_path="GLSUJY/NDIVLD")
    queue_dict[CLIENT1_QUEUE].get()
    queue_dict[MAIN_QUEUE].put(START_FROM_SUCCESSFUL)

    # test start (from server) + pause (from client1) + resume (from client2)
    remote_execution_engine.start()

    queue_dict[CLIENT1_TO_CLIENT2].put("Started state machine")
    logger.info("Wait for client2 to pause the state machine")
    import Queue
    try:
        queue_dict[CLIENT2_TO_CLIENT1].get(timeout=10)  # get paused signal from client2
    except Queue.Empty, e:
        queue_dict[MAIN_QUEUE].put(TEST_ERROR)
        logger.exception("Client2 did not respond")
        os._exit(0)

    remote_execution_engine.start()  # resume execution
    queue_dict[CLIENT1_QUEUE].get()  # synchronize to server
    queue_dict[MAIN_QUEUE].put(START_PAUSE_RESUME_SUCCESSFUL)

    queue_dict[KILL_CLIENT1_QUEUE].get()  # synchronize to main process
    os._exit(0)


def interacting_function_client2(main_window_controller, global_monitoring_manager, queue_dict):
    from rafcon.utils import log
    logger = log.get_logger("Interacting client2")

    for id, queue in queue_dict.iteritems():
        assert isinstance(queue, multiprocessing.queues.Queue)
    while not global_monitoring_manager.endpoint_initialized:
        time.sleep(0.01)
    import rafcon.core.singleton as core_singletons
    remote_execution_engine = core_singletons.state_machine_execution_engine

    queue_dict[CLIENT1_TO_CLIENT2].get()
    logger.info("Pausing state machine on remote server")
    remote_execution_engine.pause()
    time.sleep(0.2)  # let state machine pause for some time
    queue_dict[CLIENT2_TO_CLIENT1].put("State machine paused")
    queue_dict[CLIENT2_QUEUE].get()  # synchronize to server

    queue_dict[KILL_CLIENT2_QUEUE].get()  # synchronize to main process
    os._exit(0)


def launch_client(interacting_function_client, multiprocessing_queue_dict):
    # explicitly add the monitoring plugin to the RAFCON_PLUGIN_PATH
    if os.environ.get('RAFCON_PLUGIN_PATH', False):
        del os.environ['RAFCON_PLUGIN_PATH']
    os.environ['RAFCON_PLUGIN_PATH'] =\
        "/volume/software/common/packages/rafcon_monitoring_plugin/latest/lib/python2.7/monitoring"

    import test.network_test.start_client
    test.network_test.start_client.start_client(interacting_function_client, multiprocessing_queue_dict)

    import sys
    sys.path.insert(1, '/volume/software/common/packages/python_acknowledged_udp/latest/lib/python2.7')


def launch_server(interacting_function_handle_server_, multiprocessing_queue_dict):
    # explicitly add the monitoring plugin to the RAFCON_PLUGIN_PATH
    if os.environ.get('RAFCON_PLUGIN_PATH', False):
        del os.environ['RAFCON_PLUGIN_PATH']
    os.environ['RAFCON_PLUGIN_PATH'] =\
        "/volume/software/common/packages/rafcon_monitoring_plugin/latest/lib/python2.7/monitoring"

    import test.network_test.start_server
    server = Process(target=test.network_test.start_server.start_server,
                     args=(interacting_function_handle_server_, multiprocessing_queue_dict))

    import sys
    sys.path.insert(1, '/volume/software/common/packages/python_acknowledged_udp/latest/lib/python2.7')

    return server


def test_multi_clients():
    test_successful = True

    queue_dict = dict()
    queue_dict[CLIENT1_QUEUE] = Queue()
    queue_dict[CLIENT2_QUEUE] = Queue()
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

    data = queue_dict[MAIN_QUEUE].get(timeout=30)
    assert data == STEPPING_SUCCESSFUL

    data = queue_dict[MAIN_QUEUE].get(timeout=30)
    assert data == STOP_START_SUCCESSFUL

    data = queue_dict[MAIN_QUEUE].get(timeout=30)
    assert data == RUN_UNTIL_SUCCESSFUL

    data = queue_dict[MAIN_QUEUE].get(timeout=30)
    assert data == START_FROM_SUCCESSFUL

    data = queue_dict[MAIN_QUEUE].get(timeout=30)
    try:
        assert data == START_PAUSE_RESUME_SUCCESSFUL
        print "Test successful"
    except AssertionError, e:
        print "Test not successful"
        test_successful = False

    queue_dict[KILL_SERVER_QUEUE].put("Kill", timeout=30)
    queue_dict[KILL_CLIENT1_QUEUE].put("Kill", timeout=30)
    queue_dict[KILL_CLIENT2_QUEUE].put("Kill", timeout=30)

    print "Joining processes"
    server.join(timeout=30)
    client1.join(timeout=30)
    client2.join(timeout=30)

    assert not server.is_alive(), "Server is still alive"
    assert not client1.is_alive(), "Client1 is still alive"
    assert not client2.is_alive(), "Client2 is still alive"
    assert test_successful is True


if __name__ == '__main__':
    test_multi_clients()
    # pytest.main([__file__])