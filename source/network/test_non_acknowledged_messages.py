from multiprocessing import Process, Queue
import os
import threading
import time
from os.path import realpath, dirname, join, exists, expanduser, expandvars, isdir
from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor

from udp_client import UdpClient
from udp_server import UdpServer
from config import global_network_config
from protocol import Protocol, MessageType

import log
logger = log.get_logger(__name__)


FINAL_MESSAGE = "final_message"


def info(title):
    print(title)
    print('module name:', __name__)
    if hasattr(os, 'getppid'):  # only available on Unix
        print('parent process:', os.getppid())
    print('process id:', os.getpid())


def wait_for_test_finished(queue, udp_endpoint, connector):
    finished = queue.get()
    print('process with id {0} will stop reactor'.format(str(os.getpid())))
    reactor.callFromThread(reactor.stop)
    # os._exit(0)


##########################################################
# server
##########################################################

global_udp_server = None
server_queue = None


def write_back_message(protocol, address):
    logger.info("Server received datagram {0} from address: {1}".format(str(protocol), str(address)))
    global_udp_server.send_message_non_acknowledged(protocol, address)
    # small sleep to let burst send all messages
    time.sleep(0.1)
    if protocol.message_content == FINAL_MESSAGE:
        logger.info("Server puts final message to multiprocessing queue")
        server_queue.put(FINAL_MESSAGE)


def start_udp_server(name, multi_processing_queue):
    info(name)
    udp_server = UdpServer()
    connector = reactor.listenUDP(global_network_config.get_config_value("SERVER_UDP_PORT"), udp_server)
    global global_udp_server
    global_udp_server = udp_server
    global server_queue
    server_queue = multi_processing_queue
    udp_server.datagram_received_function = write_back_message

    wait_for_test_finish = threading.Thread(target=wait_for_test_finished, args=[multi_processing_queue,
                                                                                 udp_server,
                                                                                 connector])
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


def start_udp_client(name, multi_processing_queue):
    info(name)
    udp_client = UdpClient()
    connector = reactor.listenUDP(0, udp_client)

    sender_thread = threading.Thread(target=send_test_data, args=[udp_client, ])
    sender_thread.start()

    wait_for_test_finish = threading.Thread(target=wait_for_test_finished, args=[multi_processing_queue,
                                                                                 udp_client,
                                                                                 connector])
    wait_for_test_finish.start()

    # reactor.addSystemEventTrigger('before', 'shutdown', udp_client.disconnect)
    reactor.run()

    sender_thread.join()
    logger.info("Client joined sender_thread")
    wait_for_test_finish.join()
    logger.info("Client joint wait_for_test_finish")


if __name__ == '__main__':
    q = Queue()
    server = Process(target=start_udp_server, args=("udp_server", q))
    server.start()

    client = Process(target=start_udp_client, args=("udp_client1", q))
    client.start()

    # #working with arbitrary number of clients
    # client = Process(target=start_udp_client, args=("udp_client2", q))
    # client.start()

    data = q.get()
    assert data == FINAL_MESSAGE
    q.put(FINAL_MESSAGE)
    q.put(FINAL_MESSAGE)
    print "Test successfull"

    server.join()
    client.join()


