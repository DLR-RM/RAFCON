"""
.. module:: monitoring client
   :platform: Unix, Windows
   :synopsis: A module to care about receiving execution status from another RAFCON instance and showing it in its own
            RAFCON instance

.. moduleauthor:: Sebastian Brunner


"""

import time

from network.udp_client import UdpClient
from network.protocol import Protocol, MessageType, STATE_EXECUTION_STATUS_SEPARATOR
from network.config import global_network_config

from rafcon.utils import log
logger = log.get_logger(__name__)


class MonitoringClient(UdpClient):

    def __init__(self):
        UdpClient.__init__(self)
        self.connector = None
        self.initialized = False
        # self.datagram_received_function = self.print_message

    def connect(self):
        from twisted.internet import reactor
        self.connector = reactor.listenUDP(0, self)
        self.initialized = True
        protocol = Protocol(MessageType.REGISTER, "Registering")
        server_address = (global_network_config.get_config_value("SERVER_IP"),
                          global_network_config.get_config_value("SERVER_UDP_PORT"))

        logger.info("Connect to server {0}!".format(str(server_address)))
        print "sending protocol " + str(protocol)
        return_value = self.send_message_acknowledged(protocol, address=server_address, blocking=True)
        if return_value:
            logger.info("Connected to server!")
        else:
            logger.error("Connection to server {0} timeout".format(str(server_address)))

    @staticmethod
    def print_message(message, address):
        logger.info("Received datagram {0} from address: {1}".format(str(message), str(address)))
