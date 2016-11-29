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

from rafcon.utils import log
logger = log.get_logger(__name__)

FINAL_MESSAGE = "final_message"
CLIENT1_QUEUE = "client1"
CLIENT1_TO_SERVER = "client_1_to_server"
MAIN_QUEUE = "main_queue"
KILL_SERVER_QUEUE = "kill_server"
KILL_CLIENT1_QUEUE = "kill_client1"

STOP_START_SUCCESSFUL = "Stop start successful"
DISCONNECTED_RUN_SUCCESSFUL = "Disconnected run until successful"
DISABLED_RUN_SUCCESSFUL = "Disabled test"
TEST_ERROR = "Test error"
APPLY_CONFIG_SUCCESSFUL = "Apply test successful"


def reset_global_variable_manager(global_variable_manager):
    global_variable_manager.set_variable("sing_counter", 0)
    global_variable_manager.set_variable("decimate_counter", 0)
    global_variable_manager.set_variable("count_counter", 0)


def synchronize_with_clients_threads(queue_dict, execution_engine):
    from rafcon.core.singleton import global_variable_manager as gvm
    from rafcon.core.execution.execution_status import StateMachineExecutionStatus
    from rafcon.core.singleton import state_machine_manager
    active_sm = state_machine_manager.get_active_state_machine()
    root_state = active_sm.root_state

    sleep_time = 0.01

    # check when run to is finished
    while gvm.get_variable("sing_counter") < 1:
        time.sleep(sleep_time)

    queue_dict[CLIENT1_QUEUE].put("start")
    print "starting tests\n\n"

    # stop start test
    while not execution_engine.finished_or_stopped():
        time.sleep(sleep_time)

    queue_dict[CLIENT1_QUEUE].put("stop received")
    # wait until state machine started
    while execution_engine.status.execution_mode is not StateMachineExecutionStatus.STARTED:
        time.sleep(sleep_time)

    while not execution_engine.finished_or_stopped():
        time.sleep(sleep_time)

    reset_global_variable_manager(gvm)
    execution_engine.stop()  # reset state machine before the next test
    if execution_engine.status.execution_mode is not StateMachineExecutionStatus.STOPPED:
        execution_engine.join()
    queue_dict[CLIENT1_QUEUE].put("start received and successfully ran the state machine")
    print "server: stop start test successful\n\n"

    """
    disconnect -> run sm -> connect -> run sm
    """
    from monitoring import server
    from monitoring.monitoring_manager import global_monitoring_manager

    # test disconnect by client run
    print "disconnect/connect by client test"
    queue_dict[CLIENT1_QUEUE].put("disconnect_me_and_run")
    queue_dict[CLIENT1_TO_SERVER].get()
    assert gvm.get_variable("count_counter") == 0
    assert gvm.get_variable("sing_counter") == 0
    assert gvm.get_variable("decimate_counter") == 0
    queue_dict[CLIENT1_QUEUE].put("reconnect_me_and_run")
    queue_dict[CLIENT1_TO_SERVER].get()
    if not execution_engine.finished_or_stopped():
        execution_engine.join()
    print "server sm finished"
    assert gvm.get_variable("count_counter") == 3
    assert gvm.get_variable("sing_counter") == 3
    assert gvm.get_variable("decimate_counter") == 3
    queue_dict[CLIENT1_QUEUE].put("succeeded")
    execution_engine.stop()
    if not execution_engine.finished_or_stopped():
        execution_engine.join()
    reset_global_variable_manager(gvm)
    print "server: disconnect/connect by client test successful\n\n"

    """
    disable -> run sm -> enable -> run sm
    """

    # test dis- enabled by server run
    print "dis- /enabled test by server"
    for address in server.network_manager_model.connected_ip_port:
        global_monitoring_manager.disable(address)
        # while not server.network_manager_model.get_connected_status(address) == "disabled":
        #     time.sleep(0.01)
    queue_dict[CLIENT1_QUEUE].put("you are disabled")
    queue_dict[CLIENT1_TO_SERVER].get()
    print "sm on client executed and stopped"
    assert gvm.get_variable("count_counter") == 0
    assert gvm.get_variable("sing_counter") == 0
    assert gvm.get_variable("decimate_counter") == 0
    for address in server.network_manager_model.connected_ip_port:
        global_monitoring_manager.disable(address)
    queue_dict[CLIENT1_QUEUE].put("you are enabled")
    queue_dict[CLIENT1_TO_SERVER].get()
    if not execution_engine.finished_or_stopped():
        execution_engine.join()
    print "server sm finished"
    assert gvm.get_variable("count_counter") == 3
    assert gvm.get_variable("sing_counter") == 3
    assert gvm.get_variable("decimate_counter") == 3
    queue_dict[CLIENT1_QUEUE].put("succeeded")
    execution_engine.stop()
    if not execution_engine.finished_or_stopped():
        execution_engine.join()
    reset_global_variable_manager(gvm)

    print "server: dis- /enabled by server test successful\n\n"

    """
    change client ID in config -> apply -> check connected client ID
    """

    print "apply config test"
    queue_dict[CLIENT1_QUEUE].put("ready to change config")
    queue_dict[CLIENT1_TO_SERVER].get()
    client_id = []
    for address in server.network_manager_model.connected_ip_port:
        client_id.append(server.network_manager_model.get_connected_id(address))
    assert "apply_test_client_id" in client_id
    queue_dict[CLIENT1_QUEUE].put("succeeded")
    print "apply config test successful\n\n"

    execution_engine.stop()
    execution_engine.join()
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

    sleep_time = 0.01

    logger.info("Start interacting with server\n\n")

    for id, queue in queue_dict.iteritems():
        assert isinstance(queue, multiprocessing.queues.Queue)

    while not global_monitoring_manager.endpoint_initialized:
        logger.warn("global_monitoring_manager not initialized yet!")
        time.sleep(sleep_time)

    import rafcon.core.singleton as core_singletons  # be careful, could be replaced before
    remote_execution_engine = core_singletons.state_machine_execution_engine

    queue_dict[CLIENT1_QUEUE].get()  # synchronize, when to start stepping
    # test stop start
    remote_execution_engine.stop()
    queue_dict[CLIENT1_QUEUE].get()
    remote_execution_engine.start()
    queue_dict[CLIENT1_QUEUE].get()
    queue_dict[MAIN_QUEUE].put(STOP_START_SUCCESSFUL)

    from monitoring.controllers import client_controller
    from rafcon.core.execution.execution_status import StateMachineExecutionStatus
    from rafcon.core.execution.execution_engine import ExecutionEngine
    from monitoring.monitoring_execution_engine import MonitoringExecutionEngine
    from monitoring.model.network_model import network_manager_model

    # test disconnect
    queue_dict[CLIENT1_QUEUE].get()

    for address in network_manager_model.connected_ip_port:
        global_monitoring_manager.disconnect(address)

    while global_monitoring_manager.endpoint.registered_to_server:
        time.sleep(sleep_time)

    # import rafcon.core.singleton as core_singletons
    # remote_execution_engine = core_singletons.state_machine_execution_engine
    while not isinstance(remote_execution_engine, ExecutionEngine):
        remote_execution_engine = core_singletons.state_machine_execution_engine
        time.sleep(sleep_time)

    remote_execution_engine.start()
    while remote_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STARTED:
        time.sleep(sleep_time)

    remote_execution_engine.join()
    # while remote_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STOPPED:
    #     time.sleep(0.01)
    queue_dict[CLIENT1_TO_SERVER].put("disconnected_and_executed")

    remote_execution_engine.stop()
    if not remote_execution_engine.finished_or_stopped():
        remote_execution_engine.join()

    queue_dict[CLIENT1_QUEUE].get()
    global_monitoring_manager.reconnect(address)

    while not global_monitoring_manager.endpoint.registered_to_server:
        time.sleep(sleep_time)

    while not network_manager_model.get_connected_status(address) == "connected":
        time.sleep(sleep_time)
    print "status connected"

    # import rafcon.core.singleton as core_singletons
    while not isinstance(remote_execution_engine, MonitoringExecutionEngine):
        remote_execution_engine = core_singletons.state_machine_execution_engine
        time.sleep(sleep_time)

    print "sm monitoring"
    remote_execution_engine.start()
    while remote_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STARTED:
        time.sleep(sleep_time)
    print "sm started"

    queue_dict[CLIENT1_TO_SERVER].put("reconnected_and_executed")
    queue_dict[CLIENT1_QUEUE].get()

    remote_execution_engine.stop()
    if not remote_execution_engine.finished_or_stopped():
        remote_execution_engine.join()

    queue_dict[MAIN_QUEUE].put(DISCONNECTED_RUN_SUCCESSFUL)
    print "client disconnection test succeeded"
    #######################################################################################
    # test disabled
    queue_dict[CLIENT1_QUEUE].get()
    print "client disabled test begin"
    for address in network_manager_model.connected_ip_port:
        while not network_manager_model.get_connected_status(address) == "disabled":
            time.sleep(sleep_time)

    # since the engine changed to local after disabling, we need to import it again
    # import rafcon.core.singleton as core_singletons
    remote_execution_engine = core_singletons.state_machine_execution_engine
    while not isinstance(remote_execution_engine, ExecutionEngine):
        time.sleep(sleep_time)

    remote_execution_engine.start()

    while remote_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STARTED:
        time.sleep(sleep_time)

    while not remote_execution_engine.finished_or_stopped():
        time.sleep(sleep_time)

    remote_execution_engine.stop()
    if not remote_execution_engine.finished_or_stopped():
        remote_execution_engine.join()

    print "disabled run stopped"
    queue_dict[CLIENT1_TO_SERVER].put("reached end")
    queue_dict[CLIENT1_QUEUE].get()
    print "ended disabled run"
    # for address in network_manager_model.connected_ip_port:
    #     while not network_manager_model.get_connected_status(address) == "connected":
    #         time.sleep(0.01)

    # since the engine changed to remote after enabling, we need to import it again

    while not isinstance(remote_execution_engine, MonitoringExecutionEngine):
        remote_execution_engine = core_singletons.state_machine_execution_engine
        time.sleep(sleep_time)

    remote_execution_engine.start()
    print "started enabled sm"
    while remote_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STARTED:
        time.sleep(sleep_time)
    print "start enabled run"
    queue_dict[CLIENT1_TO_SERVER].put("started execution")
    queue_dict[CLIENT1_QUEUE].get()

    remote_execution_engine.stop()
    if not remote_execution_engine.finished_or_stopped():
        remote_execution_engine.join()

    queue_dict[MAIN_QUEUE].put(DISABLED_RUN_SUCCESSFUL)

    ####################################################
    # apply config test
    queue_dict[CLIENT1_QUEUE].get()
    network_manager_model.set_config_value('CLIENT_ID', 'apply_test_client_id')
    while not client_controller.global_network_config.get_config_value('CLIENT_ID') == 'apply_test_client_id':
        time.sleep(sleep_time)
    global_monitoring_manager.endpoint.registered_to_server = False
    global_monitoring_manager.reinitialize(network_manager_model.connected_ip_port)

    while not global_monitoring_manager.endpoint_initialized:
        logger.warn("global_monitoring_manager not initialized yet!")
        time.sleep(sleep_time)

    logger.info("Wait until registered to server")
    while not global_monitoring_manager.endpoint:
        time.sleep(sleep_time)
    while not global_monitoring_manager.endpoint.registered_to_server:
        time.sleep(sleep_time)
    # here is a function needed which ensures that the client is disconnected and reconnected again
    queue_dict[CLIENT1_TO_SERVER].put("on_apply_button clicked")
    queue_dict[CLIENT1_QUEUE].get()

    while not isinstance(remote_execution_engine, MonitoringExecutionEngine):
        # import rafcon.core.singleton as core_singletons
        remote_execution_engine = core_singletons.state_machine_execution_engine
        time.sleep(sleep_time)
    queue_dict[MAIN_QUEUE].put(APPLY_CONFIG_SUCCESSFUL)

    queue_dict[KILL_CLIENT1_QUEUE].get()  # synchronize to main process
    os._exit(0)


def launch_client(interacting_function_client, multiprocessing_queue_dict):
    # explicitly add the monitoring plugin to the RAFCON_PLUGIN_PATH
    if os.environ.get('RAFCON_PLUGIN_PATH', False):
        del os.environ['RAFCON_PLUGIN_PATH']
    os.environ['RAFCON_PLUGIN_PATH'] = \
        "/volume/software/common/packages/rafcon_monitoring_plugin/latest/lib/python2.7/monitoring"

    import test.network_test.start_client
    test.network_test.start_client.start_client(interacting_function_client, multiprocessing_queue_dict)

    import sys
    sys.path.insert(1, '/volume/software/common/packages/python_acknowledged_udp/latest/lib/python2.7')


def launch_server(interacting_function_handle_server_, multiprocessing_queue_dict):
    # explicitly add the monitoring plugin to the RAFCON_PLUGIN_PATH
    if os.environ.get('RAFCON_PLUGIN_PATH', False):
        del os.environ['RAFCON_PLUGIN_PATH']
    os.environ['RAFCON_PLUGIN_PATH'] = \
        "/volume/software/common/packages/rafcon_monitoring_plugin/latest/lib/python2.7/monitoring"

    import test.network_test.start_server
    server = Process(target=test.network_test.start_server.start_server,
                     args=(interacting_function_handle_server_, multiprocessing_queue_dict))

    import sys
    sys.path.insert(1, '/volume/software/common/packages/python_acknowledged_udp/latest/lib/python2.7')

    return server


def test_network_gui():
    test_successful = True

    queue_dict = dict()
    queue_dict[CLIENT1_TO_SERVER] = Queue()
    queue_dict[CLIENT1_QUEUE] = Queue()
    queue_dict[MAIN_QUEUE] = Queue()
    queue_dict[KILL_SERVER_QUEUE] = Queue()
    queue_dict[KILL_CLIENT1_QUEUE] = Queue()

    server = launch_server(interacting_function_server, queue_dict)
    server.start()

    client1 = Process(target=launch_client, args=(interacting_function_client1, queue_dict))
    client1.start()

    data = queue_dict[MAIN_QUEUE].get(timeout=30)
    assert data == STOP_START_SUCCESSFUL

    data = queue_dict[MAIN_QUEUE].get(timeout=30)
    assert data == DISCONNECTED_RUN_SUCCESSFUL

    data = queue_dict[MAIN_QUEUE].get(timeout=30)
    assert data == DISABLED_RUN_SUCCESSFUL

    data = queue_dict[MAIN_QUEUE].get(timeout=30)
    assert data == APPLY_CONFIG_SUCCESSFUL

    queue_dict[KILL_SERVER_QUEUE].put("Kill", timeout=30)
    queue_dict[KILL_CLIENT1_QUEUE].put("Kill", timeout=30)

    print "Joining processes"
    server.join(timeout=30)
    client1.join(timeout=30)

    assert not server.is_alive(), "Server is still alive"
    assert not client1.is_alive(), "Client1 is still alive"
    assert test_successful is True


if __name__ == '__main__':
    test_network_gui()
    # pytest.main([__file__])