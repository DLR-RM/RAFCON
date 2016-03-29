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
    logger.info('process with id {0} will stop reactor'.format(str(os.getpid())))
    reactor.callFromThread(reactor.stop)
    # logger.info('process with id {0} did stop reactor'.format(str(os.getpid())))
    # os._exit(0)


##########################################################
# server
##########################################################

server_transport = None


def write_back_message(datagram, address):
    logger.info("Server received datagram {0} from address: {1}".format(str(datagram), str(address)))
    # server_transport.write(datagram, address)


def start_udp_server(name, multi_processing_queue):
    info(name)
    udp_server = UdpServer()
    connector = reactor.listenUDP(global_network_config.get_config_value("SERVER_UDP_PORT"), udp_server)
    udp_server.datagram_received_function = write_back_message

    global server_transport
    server_transport = udp_server.get_transport()

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

number_of_dropped_messages = 0


def send_test_data(udp_client, multi_processing_queue):
    protocols = []

    # Here just register messages are sent: as register messages are acknowledged per default
    # send_message_acknowledged should be successful
    protocols.append(Protocol(MessageType.REGISTER, "registering_with_acks"))
    protocols.append(Protocol(MessageType.REGISTER, "this_is_a_state_id"))
    protocols.append(Protocol(MessageType.REGISTER, "not_the_final_message"))
    # protocols.append(Protocol(MessageType.REGISTER, FINAL_MESSAGE))

    # register for acknowledges in the first message, all subsequent message should then be acknowledged
    protocols.append(Protocol(MessageType.REGISTER_WITH_ACKNOWLEDGES, "Registering with acks"))
    protocols.append(Protocol(MessageType.STATE_ID, "This is a state_id"))
    protocols.append(Protocol(MessageType.COMMAND, FINAL_MESSAGE))

    while True:
        protocol = protocols.pop(0)
        logger.debug("For unit test send datagram: {0}".format(str(protocol)))
        # TODO: how does twisted know to which endpoint the message should be sent?
        udp_client.send_message_acknowledged(protocol,
                                             (global_network_config.get_config_value("SERVER_IP"),
                                              global_network_config.get_config_value("SERVER_UDP_PORT")),
                                             blocking=True)

        if protocol.message_content == FINAL_MESSAGE:
            break

        time.sleep(0.1)
    logger.debug("Sender thread finished")

    while udp_client.messages_to_be_acknowledged_pending():
        time.sleep(0.5)

    if udp_client.number_of_dropped_messages == 0:
        multi_processing_queue.put("Success")
    else:
        multi_processing_queue.put("Failure")


def start_udp_client(name, multi_processing_queue):
    info(name)
    udp_client = UdpClient()
    connector = reactor.listenUDP(0, udp_client)

    sender_thread = threading.Thread(target=send_test_data, args=[udp_client, multi_processing_queue])
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

    client = Process(target=start_udp_client, args=("udp_client", q))
    client.start()

    # working with arbitrary number of clients
    # client = Process(target=start_udp_client, args=("udp_client", q))
    # client.start()

    data = q.get()
    if data == "Success":
        logger.info("Test successfull\n\n")
    else:
        logger.error("Test failed\n\n")

    q.put(FINAL_MESSAGE)
    q.put(FINAL_MESSAGE)

    server.join()
    client.join()

    assert data == "Success"