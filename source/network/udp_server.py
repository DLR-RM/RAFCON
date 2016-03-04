#!/usr/bin/env python

# Copyright (c) Twisted Matrix Laboratories.
# See LICENSE for details.

from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from communication_endpoint import CommunicationEndpoint
from rafcon.utils import log
logger = log.get_logger(__name__)


# Here's a UDP version of the simplest possible protocol
class UdpServer(CommunicationEndpoint):

    def __init__(self):
        self.datagram_received_function = self.print_message

    def print_message(self, datagram, address):
        logger.info("Received datagram {0} from address: {1}".format(str(datagram), str(address)))

    def datagramReceived(self, datagram, address):
        self.datagram_received_function(datagram, address)

    def get_transport(self):
        return self.transport