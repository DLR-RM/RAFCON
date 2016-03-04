#!/usr/bin/env python

import threading
from config import global_config
from protocol import Protocol, MessageType

from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from multiprocessing import Queue
from communication_endpoint import CommunicationEndpoint
from rafcon.utils import log
logger = log.get_logger(__name__)


class UdpClient(CommunicationEndpoint):

    def __init__(self):
        CommunicationEndpoint.__init__(self)
        self.check_acknowledgements_thread.start()
        self.datagram_received_function = self.print_message

    def startProtocol(self):
        self.transport.connect(global_config.get_config_value("SERVER_IP"),
                               global_config.get_config_value("SERVER_UDP_PORT"))

    @staticmethod
    def print_message(message, address):
        logger.info("Received datagram {0} from address: {1}".format(str(message), str(address)))
