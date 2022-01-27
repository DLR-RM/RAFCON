"""
.. module:: test network gui
   :platform: Unix, Windows
   :synopsis: A module to test the RAFCON monitoring setup with gui

.. moduleauthor:: Voggenreiter Benno


"""
from multiprocessing import Process, Queue
import multiprocessing
import threading
import time
import os
import pytest
from tests.network.test_single_client import reset_global_variable_manager
from tests.network.test_single_client import launch_client
from tests.network.test_single_client import launch_server

from rafcon.utils import log
logger = log.get_logger(__name__)

SERVER_TO_CLIENT = "client1"
CLIENT_TO_SERVER = "client_1_to_server"
MAIN_QUEUE = "main_queue"
KILL_SERVER_QUEUE = "kill_server"
KILL_CLIENT1_QUEUE = "kill_client1"

STOP_START_SUCCESSFUL = "Stop start successful"
DISCONNECTED_RUN_SUCCESSFUL = "Disconnected run until successful"
DISABLED_RUN_SUCCESSFUL = "Disabled test"
TEST_ERROR = "Test error"
APPLY_CONFIG_SUCCESSFUL = "Apply test successful"


def server_interaction_worker(queue_dict, execution_engine, state_machine_id):
    from rafcon.core.singleton import global_variable_manager as gvm
    from rafcon.core.execution.execution_status import StateMachineExecutionStatus
    from rafcon.core.singleton import state_machine_manager
    from monitoring import server
    from monitoring.monitoring_manager import global_monitoring_manager

    sm = state_machine_manager.state_machines[state_machine_id]
    print("current state machine", sm)
    root_state = sm.root_state
    sleep_time = 0.99

    for key, sv in sm.root_state.scoped_variables.items():
        if sv.name == "bottles":
            sv.default_value = 3

    #######################################################
    print("\n\n\nserver TEST1 stop - start - wait_for_stop")
    #######################################################

    queue_element = queue_dict[CLIENT_TO_SERVER].get()
    assert queue_element == "start_test_1"
    print("received: ", queue_element)

    # wait before Decimate Bottles
    execution_engine.run_to_selected_state("GLSUJY/NDIVLD", state_machine_id=state_machine_id)
    # check when run to is finished; run_to is issued from the server thread
    while gvm.get_variable("sing_counter") < 1:
        print("wait for client")
        time.sleep(sleep_time)
    print("starting tests\n\n")

    print("put: started")
    queue_dict[SERVER_TO_CLIENT].put("started")

    # step1: stop start test
    while not execution_engine.finished_or_stopped():
        print("wait until state machine finished!")
        time.sleep(sleep_time)
    print("put: stop_received")
    queue_dict[SERVER_TO_CLIENT].put("stop_received")

    # step2: wait until state machine started
    while execution_engine.status.execution_mode is not StateMachineExecutionStatus.STARTED:
        print("execution_engine.status.execution_mode: ", execution_engine.status.execution_mode)
        # this time has to be very small otherwise the state machine start and finish during one sleep period
        # each state has a sleep of 0.01s
        time.sleep(0.01)
    print("put: started")
    queue_dict[SERVER_TO_CLIENT].put("restarted")

    # stop the state machine locally
    execution_engine.stop()  # reset state machine before the next test
    execution_engine.join()

    reset_global_variable_manager(gvm)
    queue_dict[SERVER_TO_CLIENT].put("successfully stopped the state machine")

    # queue_dict[KILL_SERVER_QUEUE].get()
    # os._exit(0)
    # return

    #######################################################
    print("\n\n\nserver TEST2 disconnect -> run sm -> connect -> run sm")
    #######################################################

    print("server: starting test 2")
    reset_global_variable_manager(gvm)

    queue_element = queue_dict[CLIENT_TO_SERVER].get()
    assert queue_element == "start_test_2"
    print("received: ", queue_element)

    # step 1
    # test disconnect by client run
    print("disconnect/connect by client test")
    queue_dict[SERVER_TO_CLIENT].put("disconnect_me_and_run")
    queue_element = queue_dict[CLIENT_TO_SERVER].get()
    assert queue_element == "disconnected_and_executed"
    print("received: ", queue_element)

    # the client executed a start command, which must not do anything as the client disconnected beforehand

    assert gvm.get_variable("count_counter") == 0
    assert gvm.get_variable("sing_counter") == 0
    assert gvm.get_variable("decimate_counter") == 0

    # step 2
    queue_dict[SERVER_TO_CLIENT].put("reconnect_me_and_run")
    queue_element = queue_dict[CLIENT_TO_SERVER].get()
    assert queue_element == "reconnected_and_executed"
    print("received: ", queue_element)

    execution_engine.join()
    print("server sm finished")
    assert gvm.get_variable("count_counter") == 3
    assert gvm.get_variable("sing_counter") == 3
    assert gvm.get_variable("decimate_counter") == 3
    queue_dict[SERVER_TO_CLIENT].put("succeeded")

    execution_engine.stop()
    execution_engine.join()
    reset_global_variable_manager(gvm)
    print("server: disconnect/connect by client test successful\n\n")

    # queue_dict[KILL_SERVER_QUEUE].get()
    # os._exit(0)
    # return

    #######################################################
    print("\n\n\nserver TEST3 disable -> run sm -> enable -> run sm ")
    #######################################################

    reset_global_variable_manager(gvm)

    queue_element = queue_dict[CLIENT_TO_SERVER].get()
    assert queue_element == "start_test_3"
    print("server received: ", queue_element)

    for address in server.network_manager_model.connected_ip_port:
        global_monitoring_manager.disable(address)
        # while not server.network_manager_model.get_connected_status(address) == "disabled":
        #     time.sleep(0.01)
    queue_dict[SERVER_TO_CLIENT].put("you_are_disabled")

    queue_element = queue_dict[CLIENT_TO_SERVER].get()
    assert queue_element == "reached end"
    print("server received: ", queue_element)

    assert gvm.get_variable("count_counter") == 0
    assert gvm.get_variable("sing_counter") == 0
    assert gvm.get_variable("decimate_counter") == 0

    queue_element = queue_dict[CLIENT_TO_SERVER].get()
    assert queue_element == "enabled_again"
    print("server received: ", queue_element)

    queue_element = queue_dict[CLIENT_TO_SERVER].get()
    assert queue_element == "started_execution"
    print("server received: ", queue_element)

    if not execution_engine.finished_or_stopped():
        execution_engine.join()
    print("server sm finished")
    assert gvm.get_variable("count_counter") == 3
    assert gvm.get_variable("sing_counter") == 3
    assert gvm.get_variable("decimate_counter") == 3
    queue_dict[SERVER_TO_CLIENT].put("succeeded")
    execution_engine.stop()
    execution_engine.join()
    reset_global_variable_manager(gvm)
    print("server: dis- /enabled by server test successful\n\n")

    # queue_dict[KILL_SERVER_QUEUE].get()
    # os._exit(0)
    # return

    #######################################################
    print("server TEST4 change client ID in config -> apply -> check connected client ID")
    #######################################################
    reset_global_variable_manager(gvm)

    queue_element = queue_dict[CLIENT_TO_SERVER].get()
    assert queue_element == "start_test_4"
    print("server received: ", queue_element)

    queue_dict[SERVER_TO_CLIENT].put("ready_to_change_config")

    queue_element = queue_dict[CLIENT_TO_SERVER].get()
    assert queue_element == "on_apply_button_clicked"
    print("server received: ", queue_element)

    client_id = []
    for address in server.network_manager_model.connected_ip_port:
        client_id.append(server.network_manager_model.get_connected_id(address))
    assert "apply_test_client_id" in client_id
    queue_dict[SERVER_TO_CLIENT].put("succeeded")
    print("apply config test successful\n\n")

    execution_engine.stop()
    execution_engine.join()
    queue_dict[KILL_SERVER_QUEUE].get()
    os._exit(0)


def interacting_function_server(queue_dict, state_machine_id):
    for id, queue in queue_dict.items():
        assert isinstance(queue, multiprocessing.queues.Queue)

    import rafcon.core.singleton as core_singletons
    execution_engine = core_singletons.state_machine_execution_engine

    sm_thread = threading.Thread(target=server_interaction_worker,
                                 args=[queue_dict, execution_engine, state_machine_id])
    sm_thread.start()

    sm_thread.join()
    print("sm_thread joined!")


def client_interaction(main_window_controller, global_monitoring_manager, queue_dict, state_machine_id):
    import rafcon.core.singleton as core_singletons  # be careful, could be replaced before
    import rafcon.gui.singleton as gui_singletons
    from monitoring.monitoring_execution_engine import MonitoringExecutionEngine
    from rafcon.core.execution.execution_status import StateMachineExecutionStatus
    from rafcon.core.execution.execution_engine import ExecutionEngine
    from monitoring.model.network_model import network_manager_model
    from monitoring.monitoring_manager import MonitoringManager
    from rafcon.utils import log

    client_controller = gui_singletons.main_window_controller.get_controller('monitoring_manager_ctrl')

    assert isinstance(global_monitoring_manager, MonitoringManager)
    remote_execution_engine = core_singletons.state_machine_execution_engine
    assert isinstance(remote_execution_engine, MonitoringExecutionEngine)

    logger = log.get_logger("Interacting client1")
    sleep_time = 0.01
    logger.info("Start interacting with server\n\n")

    for id, queue in queue_dict.items():
        assert isinstance(queue, multiprocessing.queues.Queue)

    while not global_monitoring_manager.endpoint_initialized:
        logger.warning("global_monitoring_manager not initialized yet!")
        time.sleep(sleep_time)

    while len(network_manager_model.connected_ip_port) < 1:
        time.sleep(sleep_time)

    while not global_monitoring_manager.endpoint.registered_to_server:
        time.sleep(sleep_time)

    #######################################################
    print("\n\n\nTEST1 stop - start - wait_for_stop")
    #######################################################

    queue_dict[CLIENT_TO_SERVER].put("start_test_1")

    # step 1: stop sm and synchronize
    queue_element = queue_dict[SERVER_TO_CLIENT].get()  # synchronize, when to start stepping
    assert queue_element == "started"
    print("received: ", queue_element)
    # test stop start
    remote_execution_engine.stop()
    queue_element = queue_dict[SERVER_TO_CLIENT].get()
    assert queue_element == "stop_received"
    print("received: ", queue_element)

    # step 2: start sm and synchronize
    remote_execution_engine.start(state_machine_id)
    queue_element = queue_dict[SERVER_TO_CLIENT].get()  # synchronize, when to start stepping
    assert queue_element == "restarted"
    print("received: ", queue_element)

    # step 3: synchronize with stopping of execution engine
    queue_element = queue_dict[SERVER_TO_CLIENT].get()
    assert queue_element == "successfully stopped the state machine"
    print("received: ", queue_element)
    queue_dict[MAIN_QUEUE].put(STOP_START_SUCCESSFUL)

    # queue_dict[KILL_CLIENT1_QUEUE].get()
    # os._exit(0)
    # return

    #######################################################
    print("\n\n\nTEST2 disconnect -> run sm -> connect -> run sm")
    #######################################################

    queue_dict[CLIENT_TO_SERVER].put("start_test_2")

    # step 1
    queue_element = queue_dict[SERVER_TO_CLIENT].get()
    assert queue_element == "disconnect_me_and_run"
    print("received: ", queue_element)

    address = None
    for addr in network_manager_model.connected_ip_port:
        global_monitoring_manager.disconnect(addr)
        address = addr

    while global_monitoring_manager.endpoint.registered_to_server:
        time.sleep(sleep_time)

    # TODO: why is this in a while loop here?
    while not isinstance(remote_execution_engine, ExecutionEngine):
        remote_execution_engine = core_singletons.state_machine_execution_engine
        time.sleep(sleep_time)

    # the start command won't do anything
    remote_execution_engine.start(state_machine_id)
    while remote_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STARTED:
        time.sleep(sleep_time)

    queue_dict[CLIENT_TO_SERVER].put("disconnected_and_executed")

    # joining and stopping the execution engine won't do anything
    remote_execution_engine.join()
    remote_execution_engine.stop()

    # step 2
    queue_element = queue_dict[SERVER_TO_CLIENT].get()
    assert queue_element == "reconnect_me_and_run"
    print("received: ", queue_element)

    global_monitoring_manager.reconnect(address)
    while not global_monitoring_manager.endpoint.registered_to_server:
        time.sleep(sleep_time)
    while not network_manager_model.get_connected_status(address) == "connected":
        time.sleep(sleep_time)
    print("status connected")

    while not isinstance(remote_execution_engine, MonitoringExecutionEngine):
        remote_execution_engine = core_singletons.state_machine_execution_engine
        time.sleep(sleep_time)

    print("sm monitoring")
    remote_execution_engine.start(state_machine_id)
    while remote_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STARTED:
        time.sleep(sleep_time)
    print("sm started")

    queue_dict[CLIENT_TO_SERVER].put("reconnected_and_executed")
    queue_element = queue_dict[SERVER_TO_CLIENT].get()
    assert queue_element == "succeeded"
    print("received: ", queue_element)

    # stop is executed by the server itself, but it is not forwarded to the remote server!
    # TODO: forward it

    queue_dict[MAIN_QUEUE].put(DISCONNECTED_RUN_SUCCESSFUL)
    print("client disconnection test succeeded")

    # queue_dict[KILL_CLIENT1_QUEUE].get()
    # os._exit(0)
    # return

    #######################################################
    print("\n\n\nclient TEST3 disable -> run sm -> enable -> run sm")
    #######################################################

    queue_dict[CLIENT_TO_SERVER].put("start_test_3")

    queue_element = queue_dict[SERVER_TO_CLIENT].get()
    assert queue_element == "you_are_disabled"
    print("client received: ", queue_element)

    for address in network_manager_model.connected_ip_port:
        while not network_manager_model.get_connected_status(address) == "disabled":
            time.sleep(sleep_time)

    print("start remote_execution_engine 1")

    # since the engine changed to local after disabling, we need to import it again
    remote_execution_engine = core_singletons.state_machine_execution_engine
    while not isinstance(remote_execution_engine, ExecutionEngine):
        time.sleep(sleep_time)

    print("start remote_execution_engine 2")
    remote_execution_engine.start(state_machine_id)

    print("wait for started")
    while remote_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STARTED:
        time.sleep(sleep_time)

    print("wait for stopped")
    while not remote_execution_engine.finished_or_stopped():
        time.sleep(sleep_time)

    print("stop engine")
    remote_execution_engine.stop()
    if not remote_execution_engine.finished_or_stopped():
        remote_execution_engine.join()

    print("disabled run stopped")
    queue_dict[CLIENT_TO_SERVER].put("reached end")

    global_monitoring_manager.reinitialize(network_manager_model.connected_ip_port)

    for address in network_manager_model.connected_ip_port:
        # TODO: reconnect does not work
        # global_monitoring_manager.reconnect(address)
        while not network_manager_model.get_connected_status(address) == "connected":
            print("network_manager_model.get_connected_status(address)", network_manager_model.get_connected_status(address))
            time.sleep(0.5)

    queue_dict[CLIENT_TO_SERVER].put("enabled_again")

    while not isinstance(remote_execution_engine, MonitoringExecutionEngine):
        remote_execution_engine = core_singletons.state_machine_execution_engine
        time.sleep(sleep_time)

    remote_execution_engine.start(state_machine_id)
    print("started enabled sm")
    while remote_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STARTED:
        time.sleep(sleep_time)
    print("start enabled run")
    queue_dict[CLIENT_TO_SERVER].put("started_execution")

    queue_element = queue_dict[SERVER_TO_CLIENT].get()
    assert queue_element == "succeeded"
    print("client received: ", queue_element)

    remote_execution_engine.stop()
    remote_execution_engine.join()

    queue_dict[MAIN_QUEUE].put(DISABLED_RUN_SUCCESSFUL)

    # queue_dict[KILL_CLIENT1_QUEUE].get()
    # os._exit(0)
    # return

    #######################################################
    print("client TEST4 change client ID in config -> apply -> check connected client ID")
    #######################################################

    queue_dict[CLIENT_TO_SERVER].put("start_test_4")

    queue_element = queue_dict[SERVER_TO_CLIENT].get()
    assert queue_element == "ready_to_change_config"
    print("client received: ", queue_element)

    network_manager_model.set_config_value('CLIENT_ID', 'apply_test_client_id')
    while not client_controller.global_network_config.get_config_value('CLIENT_ID') == 'apply_test_client_id':
        time.sleep(sleep_time)
    global_monitoring_manager.endpoint.registered_to_server = False
    global_monitoring_manager.reinitialize(network_manager_model.connected_ip_port)

    while not global_monitoring_manager.endpoint_initialized:
        logger.warning("global_monitoring_manager not initialized yet!")
        time.sleep(sleep_time)

    logger.info("Wait until registered to server")
    while not global_monitoring_manager.endpoint:
        time.sleep(sleep_time)
    while not global_monitoring_manager.endpoint.registered_to_server:
        time.sleep(sleep_time)
    # here is a function needed which ensures that the client is disconnected and reconnected again
    queue_dict[CLIENT_TO_SERVER].put("on_apply_button_clicked")

    queue_element = queue_dict[SERVER_TO_CLIENT].get()
    assert queue_element == "succeeded"
    print("client received: ", queue_element)

    while not isinstance(remote_execution_engine, MonitoringExecutionEngine):
        # import rafcon.core.singleton as core_singletons
        remote_execution_engine = core_singletons.state_machine_execution_engine
        time.sleep(sleep_time)
    queue_dict[MAIN_QUEUE].put(APPLY_CONFIG_SUCCESSFUL)

    queue_dict[KILL_CLIENT1_QUEUE].get()  # synchronize to main process
    os._exit(0)


def test_network_gui():

    from tests.network.test_single_client import check_if_ports_are_open
    assert check_if_ports_are_open(), "Address already in use by another server!"

    test_successful = True

    queue_dict = dict()
    queue_dict[CLIENT_TO_SERVER] = Queue()
    queue_dict[SERVER_TO_CLIENT] = Queue()
    queue_dict[MAIN_QUEUE] = Queue()
    queue_dict[KILL_SERVER_QUEUE] = Queue()
    queue_dict[KILL_CLIENT1_QUEUE] = Queue()

    server = launch_server(interacting_function_server, queue_dict)
    server.start()

    client1 = Process(target=launch_client, args=(client_interaction, queue_dict))
    client1.start()

    try:
        data = queue_dict[MAIN_QUEUE].get(timeout=10)
        assert data == STOP_START_SUCCESSFUL

        data = queue_dict[MAIN_QUEUE].get(timeout=10)
        assert data == DISCONNECTED_RUN_SUCCESSFUL

        data = queue_dict[MAIN_QUEUE].get(timeout=10)
        assert data == DISABLED_RUN_SUCCESSFUL

        data = queue_dict[MAIN_QUEUE].get(timeout=10)
        assert data == APPLY_CONFIG_SUCCESSFUL
        #
        queue_dict[KILL_SERVER_QUEUE].put("Kill", timeout=10)
        queue_dict[KILL_CLIENT1_QUEUE].put("Kill", timeout=10)
    except:
        server.terminate()
        client1.terminate()
        raise
    finally:
        print("Joining processes")
        server.join(timeout=10)
        client1.join(timeout=10)

    assert not server.is_alive(), "Server is still alive"
    assert not client1.is_alive(), "Client1 is still alive"
    assert test_successful is True


if __name__ == '__main__':
    test_network_gui()
    # pytest.main([__file__])
