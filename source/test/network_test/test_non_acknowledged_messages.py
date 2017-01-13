from multiprocessing import Process, Queue
import os
import threading
import time
import pytest

import sys
sys.path.insert(1, '/volume/software/common/packages/python_acknowledged_udp/latest/lib/python2.7')

from twisted.internet import reactor

from acknowledged_udp.udp_client import UdpClient
from acknowledged_udp.udp_server import UdpServer
from acknowledged_udp.config import global_network_config
from acknowledged_udp.protocol import Protocol, MessageType

from rafcon.utils import log
logger = log.get_logger(__name__)


FINAL_MESSAGE = "final_message"
DESTROY_MESSAGE = "destroy"
# CLIENT_TO_SERVER_QUEUE = "client_to_server"
# SERVER_TO_CLIENT_QUEUE = "server_to_client"
SERVER_TO_MAIN_QUEUE = "client_to_server"
# CLIENT_TO_MAIN_QUEUE = "server_to_client"
MAIN_TO_SERVER_QUEUE = "client_to_server"
MAIN_TO_CLIENT_QUEUE = "server_to_client"


def info(title):
    print(title)
    print('module name:', __name__)
    if hasattr(os, 'getppid'):  # only available on Unix
        print('parent process:', os.getppid())
    print('process id:', os.getpid())


def wait_for_test_finished(queue_dict, udp_endpoint, connector, server):
    if server:
        destroy_message = queue_dict[MAIN_TO_SERVER_QUEUE].get()
    else:
        destroy_message = queue_dict[MAIN_TO_CLIENT_QUEUE].get()
    print('process with id {0} will stop reactor'.format(str(os.getpid())))
    reactor.callFromThread(reactor.stop)
    print('process with id {0} did stop reactor'.format(str(os.getpid())))
    exit()
    # this could probably destroy twisted transport protocols
    # os._exit(0)



##########################################################
# server
##########################################################

global_udp_server = None
server_to_main_queue = None


def write_back_message(protocol, address):
    logger.info("Server received datagram {0} from address: {1}".format(str(protocol), str(address)))
    global_udp_server.send_message_non_acknowledged(protocol, address)
    # small sleep to let burst send all messages
    time.sleep(0.1)
    if protocol.message_content == FINAL_MESSAGE:
        logger.info("Server puts final message to multiprocessing queue")
        server_to_main_queue.put(FINAL_MESSAGE)


def start_udp_server(name, queue_dict):
    info(name)
    udp_server = UdpServer()
    connector = reactor.listenUDP(global_network_config.get_config_value("SERVER_UDP_PORT"), udp_server)
    global global_udp_server
    global_udp_server = udp_server
    global server_to_main_queue
    server_to_main_queue = queue_dict[SERVER_TO_MAIN_QUEUE]
    udp_server.datagram_received_function = write_back_message

    wait_for_test_finish = threading.Thread(target=wait_for_test_finished, args=[queue_dict,
                                                                                 udp_server,
                                                                                 connector,
                                                                                 True])
    wait_for_test_finish.start()

    # reactor.addSystemEventTrigger('before', 'shutdown', udp_server.disconnect)
    reactor.run()

    wait_for_test_finish.join()
    logger.info("Server joined wait_for_test_finish")

##########################################################
# client
##########################################################


def send_test_data(udp_client):
    protocols = []
    protocols.append(Protocol(MessageType.COMMAND, "This is a command"))
    protocols.append(Protocol(MessageType.STATE_ID, "This is a state_id"))
    protocols.append(Protocol(MessageType.COMMAND, FINAL_MESSAGE))

    while True:
        protocol = protocols.pop(0)
        logger.debug("For unit test send datagram: {0}".format(str(protocol)))
        udp_client.send_message_non_acknowledged(protocol)

        if protocol.message_content is FINAL_MESSAGE:
            break

        time.sleep(0.1)
    logger.debug("Sender thread finished")


def start_udp_client(name, queue_dict):
    info(name)
    udp_client = UdpClient()
    connector = reactor.listenUDP(0, udp_client)

    sender_thread = threading.Thread(target=send_test_data, args=[udp_client, ])
    sender_thread.start()

    wait_for_test_finish = threading.Thread(target=wait_for_test_finished, args=[queue_dict,
                                                                                 udp_client,
                                                                                 connector,
                                                                                 False])
    wait_for_test_finish.start()

    reactor.run()

    sender_thread.join()
    logger.info("Client joined sender_thread")
    wait_for_test_finish.join()
    logger.info("Client joint wait_for_test_finish")


def test_non_acknowledged_messages():

    from test_single_client import check_if_ports_are_open
    assert check_if_ports_are_open(), "Address already in use by another server!"

    queue_dict = dict()
    # queue_dict[CLIENT_TO_SERVER_QUEUE] = Queue()
    # queue_dict[SERVER_TO_CLIENT_QUEUE] = Queue()
    queue_dict[SERVER_TO_MAIN_QUEUE] = Queue()
    # queue_dict[CLIENT_TO_MAIN_QUEUE] = Queue()
    queue_dict[MAIN_TO_SERVER_QUEUE] = Queue()
    queue_dict[MAIN_TO_CLIENT_QUEUE] = Queue()
    server = Process(target=start_udp_server, args=("udp_server", queue_dict))
    server.start()

    client = Process(target=start_udp_client, args=("udp_client1", queue_dict))
    client.start()

    try:
        data = queue_dict[SERVER_TO_MAIN_QUEUE].get(timeout=10)
        assert data == FINAL_MESSAGE
        # send destroy commands to other processes
        queue_dict[MAIN_TO_SERVER_QUEUE].put(DESTROY_MESSAGE)
        queue_dict[MAIN_TO_CLIENT_QUEUE].put(DESTROY_MESSAGE)
    except:
        server.terminate()
        client.terminate()
        raise
    finally:
        server.join(10)
        client.join(10)

    assert not server.is_alive(), "Server is still alive"
    assert not client.is_alive(), "Client is still alive"

    print "Test successful"


if __name__ == '__main__':
    test_non_acknowledged_messages()
    # pytest.main([__file__])