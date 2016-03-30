#!/usr/bin/env python

from twisted.internet.protocol import DatagramProtocol

from communication_endpoint import CommunicationEndpoint
import log
logger = log.get_logger(__name__)


# Here's a UDP version of the simplest possible protocol
class UdpServer(CommunicationEndpoint):

    def __init__(self):
        CommunicationEndpoint.__init__(self)
        self.check_acknowledgements_thread.start()
        self.datagram_received_function = self.print_message

    def startProtocol(self):
        CommunicationEndpoint.startProtocol(self)
        logger.info("Protocol started")

    def stopProtocol(self):
        CommunicationEndpoint.stopProtocol(self)
        logger.warn("Protocol stopped")

    @staticmethod
    def print_message(message, address):
        logger.info("Received datagram {0} from address: {1}".format(str(message), str(address)))