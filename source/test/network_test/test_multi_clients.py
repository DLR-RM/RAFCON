from multiprocessing import Process, Queue
import multiprocessing
import os
import threading
import time

FINAL_MESSAGE = "final_message"
CLIENT1_QUEUE = "client1"
CLIENT2_QUEUE = "client2"
MAIN_QUEUE = "main_queue"
KILL_SERVER_QUEUE = "kill_server"
KILL_CLIENT1_QUEUE = "kill_client1"
KILL_CLIENT2_QUEUE = "kill_client2"


def synchronize_with_clients_threads(queue_dict, execution_engine):
    from rafcon.statemachine.singleton import global_variable_manager
    # sing_counter, decimate_counter and count_counter
    while global_variable_manager.get_variable("decimate_counter") < 1:
        time.sleep(0.5)
    queue_dict[CLIENT1_QUEUE].put("step_received1")

    while global_variable_manager.get_variable("count_counter") < 1:
        time.sleep(0.1)
    queue_dict[CLIENT1_QUEUE].put("step_received2")

    while global_variable_manager.get_variable("sing_counter") < 2:
        time.sleep(0.1)
    queue_dict[CLIENT1_QUEUE].put("step_received3")

    while global_variable_manager.get_variable("sing_counter") > 1:
        time.sleep(0.1)
    queue_dict[CLIENT1_QUEUE].put("backward_step_received")

    print "exiting server process"
    execution_engine.stop()

    queue_dict[KILL_SERVER_QUEUE].get()

    import os
    os._exit(0)


def interacting_function_server(queue_dict):
    from rafcon.utils import log
    logger = log.get_logger("Interacting server")

    for id, queue in queue_dict.iteritems():
        assert isinstance(queue, multiprocessing.queues.Queue)

    import rafcon.statemachine.singleton as core_singletons
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

    for id, queue in queue_dict.iteritems():
        assert isinstance(queue, multiprocessing.queues.Queue)

    while not global_monitoring_manager.endpoint_initialized:
        logger.warn("global_monitoring_manager not initialized yet!")
        time.sleep(0.5)

    import rafcon.statemachine.singleton as core_singletons
    remote_execution_engine = core_singletons.state_machine_execution_engine

    remote_execution_engine.step_over()  # triggers decimate bottles
    queue_dict[CLIENT1_QUEUE].get()

    remote_execution_engine.step_into()  # triggers count
    queue_dict[CLIENT1_QUEUE].get()

    remote_execution_engine.step_out()  # triggers sing
    queue_dict[CLIENT1_QUEUE].get()

    remote_execution_engine.backward_step()  # backward triggers sing
    print "client1: waiting for last message"
    queue_dict[CLIENT1_QUEUE].get()

    # sing_counter, decimate_counter and count_counter
    print "client1: got last message! Put final message to main queue"
    queue_dict[MAIN_QUEUE].put(FINAL_MESSAGE)

    queue_dict[KILL_CLIENT1_QUEUE].get()

    import os
    os._exit(0)


def interacting_function_client2(main_window_controller, global_monitoring_manager, queue_dict):
    from rafcon.utils import log
    logger = log.get_logger("Interacting client1")

    for id, queue in queue_dict.iteritems():
        assert isinstance(queue, multiprocessing.queues.Queue)
    while not global_monitoring_manager.endpoint_initialized:
        time.sleep(0.1)


if __name__ == '__main__':
    import test.network_test.start_server
    import test.network_test.start_client

    queue_dict = dict()
    queue_dict[CLIENT1_QUEUE] = Queue()
    queue_dict[CLIENT2_QUEUE] = Queue()
    queue_dict[MAIN_QUEUE] = Queue()
    queue_dict[KILL_SERVER_QUEUE] = Queue()
    queue_dict[KILL_CLIENT1_QUEUE] = Queue()
    queue_dict[KILL_CLIENT2_QUEUE] = Queue()
    server = Process(target=test.network_test.start_server.start_server,
                     args=(interacting_function_server, queue_dict))
    server.start()

    client1 = Process(target=test.network_test.start_client.start_client,
                      args=(interacting_function_client1, queue_dict))
    client1.start()

    # client2 = Process(target=test.network_test.start_client.start_client,
    #                  args=(interacting_function_client2, q))
    # client2.start()

    data = queue_dict[MAIN_QUEUE].get()
    assert data == FINAL_MESSAGE
    # q.put(FINAL_MESSAGE)
    # q.put(FINAL_MESSAGE)
    print "Test successfull"

    queue_dict[KILL_SERVER_QUEUE].put("Kill")
    queue_dict[KILL_CLIENT1_QUEUE].put("Kill")

    # server.join()
    # client1.join()
    # client2.join()

