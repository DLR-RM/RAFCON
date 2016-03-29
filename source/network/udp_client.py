#!/usr/bin/env python

import threading
from config import global_network_config
from protocol import Protocol, MessageType

from twisted.internet.protocol import DatagramProtocol
from multiprocessing import Queue
from communication_endpoint import CommunicationEndpoint
import log
logger = log.get_logger(__name__)


class UdpClient(CommunicationEndpoint):

    def __init__(self):
        CommunicationEndpoint.__init__(self)
        self.check_acknowledgements_thread.start()
        self.datagram_received_function = self.print_message

    def startProtocol(self):
        CommunicationEndpoint.startProtocol(self)
        self.transport.connect(global_network_config.get_config_value("SERVER_IP"),
                               global_network_config.get_config_value("SERVER_UDP_PORT"))
        logger.info("self.transport {0}".format(str(self.transport)))
        logger.info("Protocol started")

    def stopProtocol(self):
        pass
        # CommunicationEndpoint.stopProtocol(self)
        # logger.warn("Protocol stopped")

    @staticmethod
    def print_message(message, address):
        logger.info("Received datagram {0} from address: {1}".format(str(message), str(address)))
