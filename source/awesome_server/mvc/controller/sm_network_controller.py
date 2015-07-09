from enum import Enum

from awesome_server.connections.tcp_connection import TCPFactory
from awesome_server.connections.udp_connection import UDPConnection

from awesome_tool.utils import log
logger = log.get_logger(__name__)


# --------------------------------------------
#               TWISTED
# --------------------------------------------
from twisted.internet import reactor
from twisted.internet.error import CannotListenError

NetworkMode = Enum('NETWORK_MODE', 'TCP UDP')


class SmNetworkController:
    """
    This class is responsible to coordinate all network connections via TCP or UDP.
    :param network_mode: The used network mode the current instance handles: TCP or UDP
    """

    def __init__(self, network_mode):
        assert isinstance(network_mode, NetworkMode)
        self.network_mode = network_mode
        self.udp_connections = {}
        self.tcp_connections = {}

    def start(self, port):
        """
        This method starts the TCP/UDP server and tells it to listen on the specified port for incoming connections.
        :param port: Port to listen to incoming connections
        :return: added connection if server started correctly, None if not
        """
        if self.network_mode == NetworkMode.TCP:
            if self.add_tcp_connection(port):
                return self.tcp_connections[port]
            else:
                return None
        elif self.network_mode == NetworkMode.UDP:
            if self.add_udp_connection(port):
                return self.udp_connections[port]
            else:
                return None
        else:
            logger.debug("Unrecognized NetworkMode detected")
            return None

    def add_udp_connection(self, port):
        """
        Adds a new UDP connection to the server if not already existing
        :param port: Port to listen to incoming connections
        :return: True if port was added, no if port already existing
        """
        if port not in self.udp_connections.keys():
            udp_con = UDPConnection()
            self.udp_connections[port] = udp_con
            try:
                reactor.listenUDP(port, udp_con)
            except CannotListenError:
                self.print_port_in_use(port)
                del self.udp_connections[port]
                return False
            return True
        else:
            self.print_port_in_use(port)
            return False

    def add_tcp_connection(self, port):
        """
        Adds a new TCP connection to the server if not already existing
        :param port: Port to listen to incoming connections
        :return: True if port was added, no if port already existing
        """
        if port not in self.tcp_connections.keys():
            tcp_fac = TCPFactory()
            self.tcp_connections[port] = tcp_fac
            try:
                reactor.listenTCP(port, tcp_fac)
            except CannotListenError:
                self.print_port_in_use(port)
                del self.tcp_connections[port]
                return False
            return True
        else:
            self.print_port_in_use(port)
            return False

    def print_port_in_use(self, port):
        """
        Print message to textview stating the port is already in use
        :param port: Port in use
        """
        logger.debug("Port %d already in use for %s" % (port, self.network_mode))

    def get_listened_ports(self):
        """
        Returns all ports the server listens to for its network mode
        :return: Listened ports
        """
        if self.network_mode == NetworkMode.TCP:
            return self.tcp_connections.keys()
        elif self.network_mode == NetworkMode.UDP:
            return self.udp_connections.keys()

    def get_connections(self):
        """
        Returns the list of connections to the server
        :return: List of connections
        """
        if self.network_mode == NetworkMode.TCP:
            return self.tcp_connections
        elif self.network_mode == NetworkMode.UDP:
            return self.udp_connections