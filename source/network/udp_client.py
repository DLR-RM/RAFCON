#!/usr/bin/env python

# Copyright (c) Twisted Matrix Laboratories.
# See LICENSE for details.

from config import global_config

from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from multiprocessing import Queue
from communication_endpoint import CommunicationEndpoint
from rafcon.utils import log
logger = log.get_logger(__name__)


class UdpClient(CommunicationEndpoint):

    def __init__(self):
        self.thread = None
        self.datagram_received_function = self.print_message
    
    def startProtocol(self):
        self.transport.connect(global_config.get_config_value("SERVER_IP"), 8000)

    @staticmethod
    def print_message(message, addr):
        logger.info("Datagram received: {0}".format(str(message)))

    def datagramReceived(self, datagram, addr):
        self.datagram_received_function(datagram, addr)

    def send_message_non_acknowledged(self, message):
        self.transport.write(message)

    def send_message_acknowledged(self, message):
        self.transport.write(message)
